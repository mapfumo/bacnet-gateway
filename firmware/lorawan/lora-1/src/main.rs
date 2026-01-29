#![no_std]
#![no_main]

// LoRaWAN interface variant module for RF switch control
mod iv;

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::{
    bind_interrupts,
    gpio::{Level, Output, Pin, Speed},
    i2c::{Config as I2cConfig, EventInterruptHandler, ErrorInterruptHandler, I2c},
    peripherals::{self, I2C2, PA11, PA12},
    rcc::*,
    rng::{self, Rng},
    spi::Spi,
    time::Hertz,
    Config,
};
use embassy_time::{Delay, Timer};
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    text::Text,
};
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};
use {defmt_rtt as _, panic_probe as _};

// LoRaWAN imports (not used yet, but prepared)
use lora_phy::lorawan_radio::LorawanRadio;
use lora_phy::sx126x::{self, Stm32wl, Sx126x, TcxoCtrlVoltage};
use lora_phy::LoRa;
use lorawan_device::async_device::{region, Device, EmbassyTimer, JoinMode, JoinResponse};
use lorawan_device::default_crypto::DefaultFactory;
use lorawan_device::region::{Subband, AU915};
use lorawan_device::{AppEui, AppKey, DevEui};

use self::iv::{InterruptHandler, Stm32wlInterfaceVariant, SubghzSpiDevice};

bind_interrupts!(struct I2c2Irqs {
    I2C2_EV => EventInterruptHandler<peripherals::I2C2>;
    I2C2_ER => ErrorInterruptHandler<peripherals::I2C2>;
});

bind_interrupts!(struct Irqs{
    SUBGHZ_RADIO => InterruptHandler;
    RNG => rng::InterruptHandler<peripherals::RNG>;
});

// SHT41 sensor constants
const SHT41_ADDR: u8 = 0x44;
const CMD_MEASURE_HIGH_PRECISION: u8 = 0xFD;

// LoRaWAN configuration constants
const MAX_TX_POWER: u8 = 14; // AU915 max TX power

// LoRaWAN credentials (from gateway TOT application)
// Note: EUIs are stored in LITTLE-ENDIAN for over-the-air transmission
const DEV_EUI: [u8; 8] = [0xAC, 0x1F, 0x09, 0xFF, 0xFE, 0x1B, 0xCE, 0x23]; // 23ce1bfeff091fac reversed
const APP_EUI: [u8; 8] = [0x56, 0x53, 0x29, 0xC5, 0x64, 0xA8, 0x30, 0xB1]; // b130a864c5295356 reversed
const APP_KEY: [u8; 16] = [
    0xB7, 0x26, 0x73, 0x9B, 0x78, 0xEC, 0x4B, 0x9E,
    0x92, 0x34, 0xE5, 0xD3, 0x5E, 0xA9, 0x68, 0x1B,
]; // AppKey stays in big-endian (MSB first)

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("====================================");
    info!("  STM32WL55 LoRa-1 - SHT41");
    info!("  Temperature & Humidity Sensor + LoRaWAN");
    info!("====================================");

    // Clock configuration matching working solution (HSE + PLL for radio stability)
    let mut config = Config::default();
    {
        config.rcc.hse = Some(Hse {
            freq: Hertz(32_000_000),
            mode: HseMode::Bypass,
            prescaler: HsePrescaler::DIV1,
        });
        config.rcc.sys = Sysclk::PLL1_R;
        config.rcc.pll = Some(Pll {
            source: PllSource::HSE,
            prediv: PllPreDiv::DIV2,
            mul: PllMul::MUL6,
            divp: None,
            divq: Some(PllQDiv::DIV2),
            divr: Some(PllRDiv::DIV2),
        });
    }
    let p = embassy_stm32::init(config);

    info!("STM32WL55 initialized with HSE + PLL clock");

    // Test I2C2 bus and detect devices
    {
        info!("Testing I2C2: PA12 (SCL), PA11 (SDA)");
        let mut i2c_config = I2cConfig::default();
        i2c_config.sda_pullup = true;
        i2c_config.scl_pullup = true;

        // SAFETY: This is the first and only time we're using these peripherals
        let mut i2c = unsafe {
            I2c::new_blocking(
                I2C2::steal(),
                PA12::steal(),
                PA11::steal(),
                Hertz(100_000),
                i2c_config,
            )
        };

        // Try to wake up SHT41 first
        info!("Attempting to wake up SHT41 @ 0x{:02X}...", SHT41_ADDR);
        let wake_result = i2c.blocking_write(SHT41_ADDR, &[CMD_MEASURE_HIGH_PRECISION]);
        match wake_result {
            Ok(_) => info!("✓ SHT41 wake command sent successfully"),
            Err(_) => info!("✗ SHT41 wake failed"),
        }

        Timer::after_millis(100).await;

        // Scan I2C bus - full scan to find all devices
        info!("Scanning I2C2 bus (full scan)...");
        let mut found_count = 0;
        for addr in 0x00..=0x7F {
            let mut buf = [0u8; 1];
            if i2c.blocking_read(addr, &mut buf).is_ok() {
                info!("✓ Device at 0x{:02X}", addr);
                found_count += 1;
            }
        }
        info!("Total devices found: {}", found_count);
    }

    // ============================================
    // Initialize LoRaWAN Radio Hardware
    // ============================================
    info!("Initializing LoRaWAN radio hardware...");

    // RF switch control pins (NUCLEO-WL55JC1 board)
    let ctrl1 = Output::new(p.PC4.degrade(), Level::Low, Speed::High);
    let ctrl2 = Output::new(p.PC5.degrade(), Level::Low, Speed::High);
    let ctrl3 = Output::new(p.PC3.degrade(), Level::High, Speed::High);
    info!("✓ RF switch pins configured (PC3, PC4, PC5)");

    // Initialize SubGHz SPI
    let spi = Spi::new_subghz(p.SUBGHZSPI, p.DMA1_CH1, p.DMA1_CH2);
    let spi = SubghzSpiDevice(spi);
    info!("✓ SubGHz SPI initialized");

    // Configure radio
    let use_high_power_pa = true; // Use high power PA for better range
    let config = sx126x::Config {
        chip: Stm32wl { use_high_power_pa },
        tcxo_ctrl: Some(TcxoCtrlVoltage::Ctrl1V7),
        use_dcdc: true,
        rx_boost: false,
    };

    // Create interface variant with RF switch control
    let iv = Stm32wlInterfaceVariant::new(
        Irqs,
        use_high_power_pa,
        Some(ctrl1),
        Some(ctrl2),
        Some(ctrl3),
    )
    .unwrap();
    info!("✓ RF switch interface variant created");

    // Initialize LoRa radio (this will be used for LoRaWAN later)
    let lora = LoRa::new(Sx126x::new(spi, iv, config), true, Delay)
        .await
        .unwrap();
    info!("✓ LoRa radio initialized");

    // Convert to LorawanRadio wrapper
    let radio: LorawanRadio<_, _, MAX_TX_POWER> = lora.into();

    // Configure AU915 region
    let mut au915 = AU915::new();
    au915.set_join_bias(Subband::_1); // Sub-band 1 (915.2-916.6 MHz channels 0-7)
    let region: region::Configuration = au915.into();
    info!("✓ AU915 region configured (sub-band 1)");

    // Initialize RNG for crypto
    let rng = Rng::new(p.RNG, Irqs);
    info!("✓ RNG initialized for crypto");

    // Create LoRaWAN device
    let mut device: Device<_, DefaultFactory, _, _> =
        Device::new(region, radio, EmbassyTimer::new(), rng);
    info!("✓ LoRaWAN device created");

    info!("========================================");
    info!("  LoRaWAN stack initialized!");
    info!("  Ready to join network");
    info!("========================================");

    // Initialize LED
    let mut led = Output::new(p.PB15, Level::Low, Speed::Low);

    // Text style for display
    let text_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);

    // Sensor and display state
    let mut temp_int = 0i16;
    let mut hum_int = 0i16;
    let mut tx_count = 0u32;
    let mut uplink_counter = 0u32;
    const UPLINK_INTERVAL: u32 = 15;

    // LoRaWAN state
    let mut is_joined = false;
    let mut join_attempt = 0u8;
    const MAX_JOIN_ATTEMPTS: u8 = 5;

    // Stale session detection: count consecutive failed confirmed uplinks
    let mut confirmed_fail_count = 0u8;
    const MAX_CONFIRMED_FAILS: u8 = 3;  // Rejoin after 3 failed confirmed uplinks
    const CONFIRMED_UPLINK_INTERVAL: u32 = 5;  // Every 5th uplink is confirmed

    let join_mode = JoinMode::OTAA {
        deveui: DevEui::from(DEV_EUI),
        appeui: AppEui::from(APP_EUI),
        appkey: AppKey::from(APP_KEY),
    };

    info!("Starting OTAA join procedure...");
    info!("DevEUI: {:02X}", DEV_EUI);
    info!("AppEUI: {:02X}", APP_EUI);

    // Track if display needs initial clear (only on first update)
    let mut display_initialized = false;

    // Macro-like closure to read sensor and update display without flicker
    // Uses fixed-width formatting and overwrites previous content
    let mut read_sensor_and_update_display = |temp_int: &mut i16, hum_int: &mut i16, tx_count: u32, is_joined: bool, join_attempt: u8, snr: i8, rssi: i16, display_initialized: &mut bool| {
        let mut i2c_config = I2cConfig::default();
        i2c_config.sda_pullup = true;
        i2c_config.scl_pullup = true;

        let mut i2c = unsafe {
            I2c::new_blocking(
                I2C2::steal(),
                PA12::steal(),
                PA11::steal(),
                Hertz(100_000),
                i2c_config,
            )
        };

        // Read SHT41 sensor
        if i2c.blocking_write(SHT41_ADDR, &[CMD_MEASURE_HIGH_PRECISION]).is_ok() {
            // Small blocking delay for sensor (can't use async in closure)
            cortex_m::asm::delay(480_000); // ~10ms at 48MHz
            let mut data = [0u8; 6];
            if i2c.blocking_read(SHT41_ADDR, &mut data).is_ok() {
                let temp_raw = ((data[0] as u16) << 8) | (data[1] as u16);
                let hum_raw = ((data[3] as u16) << 8) | (data[4] as u16);
                *temp_int = -45 + ((175 * temp_raw as i32) / 65535) as i16;
                *hum_int = -6 + ((125 * hum_raw as i32) / 65535) as i16;
                info!("✓ SHT41: {}°C, {}% RH", *temp_int, *hum_int);
            }
        }

        // Update OLED display
        let interface = I2CDisplayInterface::new(i2c);
        let mut display = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();

        // Initialize display only on first call
        if !*display_initialized {
            if display.init().is_ok() {
                let _ = display.clear(BinaryColor::Off);
                let _ = display.flush();
                *display_initialized = true;
            }
        }

        // Clear buffer and redraw (buffer clear doesn't flicker - only flush sends to display)
        let _ = display.clear(BinaryColor::Off);

        // Line 1: Title + TX count or join status (fixed width to fill line)
        let mut line1 = heapless::String::<32>::new();
        if is_joined {
            let _ = core::fmt::write(&mut line1, format_args!("LoRa-1   TX:{:>4}    ", tx_count));
        } else {
            let _ = core::fmt::write(&mut line1, format_args!("LoRa-1 Join:{:>2}      ", join_attempt));
        }
        let _ = Text::new(&line1, Point::new(0, 7), text_style).draw(&mut display);

        // Line 2: Temperature and Humidity (fixed width)
        let mut line2 = heapless::String::<32>::new();
        let _ = core::fmt::write(&mut line2, format_args!("{:>3}C  {:>3}%          ", *temp_int, *hum_int));
        let _ = Text::new(&line2, Point::new(0, 17), text_style).draw(&mut display);

        // Line 3: Connection status (fixed width)
        let mut line3 = heapless::String::<32>::new();
        if !is_joined {
            let _ = core::fmt::write(&mut line3, format_args!("Connecting...       "));
        } else if rssi != 0 {
            let _ = core::fmt::write(&mut line3, format_args!("S:{:>3} R:{:>4}       ", snr, rssi));
        } else {
            let _ = core::fmt::write(&mut line3, format_args!("Joined              "));
        }
        let _ = Text::new(&line3, Point::new(0, 27), text_style).draw(&mut display);

        // Single flush sends complete frame to display (no flicker)
        let _ = display.flush();
    };

    // ============================================
    // Join Phase: Try to join while updating display
    // ============================================
    while !is_joined && join_attempt < MAX_JOIN_ATTEMPTS {
        join_attempt += 1;
        info!("OTAA join attempt {}/{}", join_attempt, MAX_JOIN_ATTEMPTS);

        // Update display before join attempt
        read_sensor_and_update_display(
            &mut temp_int, &mut hum_int, tx_count,
            is_joined, join_attempt, 0, 0, &mut display_initialized,
        );

        // Try to join with a timeout - use select to race join against display updates
        match device.join(&join_mode).await {
            Ok(JoinResponse::JoinSuccess) => {
                info!("✓ LoRaWAN network joined successfully!");
                is_joined = true;
            }
            Ok(JoinResponse::NoJoinAccept) => {
                error!("✗ Join failed (attempt {}): No join accept received", join_attempt);
                if join_attempt < MAX_JOIN_ATTEMPTS {
                    // Wait 10 seconds, updating display every 2 seconds
                    for _ in 0..5 {
                        Timer::after_secs(2).await;
                        read_sensor_and_update_display(
                            &mut temp_int, &mut hum_int, tx_count,
                            is_joined, join_attempt, 0, 0, &mut display_initialized,
                        );
                    }
                }
            }
            Err(err) => {
                error!("✗ Join error (attempt {}): {:?}", join_attempt, err);
                if join_attempt < MAX_JOIN_ATTEMPTS {
                    // Wait 10 seconds, updating display every 2 seconds
                    for _ in 0..5 {
                        Timer::after_secs(2).await;
                        read_sensor_and_update_display(
                            &mut temp_int, &mut hum_int, tx_count,
                            is_joined, join_attempt, 0, 0, &mut display_initialized,
                        );
                    }
                }
            }
        }
    }

    if is_joined {
        info!("========================================");
        info!("  LoRaWAN OTAA Join Complete!");
        info!("  Device is now connected");
        info!("========================================");
    } else {
        info!("========================================");
        info!("  LoRaWAN join failed after {} attempts", MAX_JOIN_ATTEMPTS);
        info!("  Continuing in offline mode");
        info!("========================================");
    }

    // ============================================
    // Main Loop: Sensor + Display + (optional) LoRaWAN
    // ============================================
    info!("Starting sensor + display loop...");

    loop {
        uplink_counter += 1;

        // Read sensor and update display
        let snr = if is_joined { device.last_snr() as i8 } else { 0 };
        let rssi = if is_joined { device.last_rssi() } else { 0 };

        read_sensor_and_update_display(
            &mut temp_int, &mut hum_int, tx_count,
            is_joined, join_attempt, snr, rssi, &mut display_initialized,
        );

        // Send LoRaWAN uplink if joined and interval reached
        if is_joined && uplink_counter >= UPLINK_INTERVAL {
            uplink_counter = 0;

            let temp_encoded = (temp_int * 100) as i16;
            let hum_encoded = (hum_int * 100) as u16;

            let payload: [u8; 4] = [
                (temp_encoded >> 8) as u8,
                temp_encoded as u8,
                (hum_encoded >> 8) as u8,
                hum_encoded as u8,
            ];

            // Every Nth uplink is confirmed to detect stale sessions
            let use_confirmed = (tx_count % CONFIRMED_UPLINK_INTERVAL as u32) == 0;

            if use_confirmed {
                info!("Sending CONFIRMED uplink: temp={}°C, hum={}%", temp_int, hum_int);
            } else {
                info!("Sending uplink: temp={}°C, hum={}%", temp_int, hum_int);
            }
            led.set_high();

            match device.send(&payload, 1, use_confirmed).await {
                Ok(response) => {
                    tx_count += 1;
                    info!("✓ Uplink sent successfully: {:?}", response);
                    info!("  SNR: {} dB, RSSI: {} dBm, TX count: {}", device.last_snr(), device.last_rssi(), tx_count);

                    // Reset fail counter on successful confirmed uplink
                    if use_confirmed {
                        confirmed_fail_count = 0;
                        info!("  Confirmed uplink ACK received - session valid");
                    }
                }
                Err(err) => {
                    error!("✗ Uplink failed: {:?}", err);

                    // Track confirmed uplink failures for stale session detection
                    if use_confirmed {
                        confirmed_fail_count += 1;
                        error!("  Confirmed uplink failed ({}/{})", confirmed_fail_count, MAX_CONFIRMED_FAILS);

                        if confirmed_fail_count >= MAX_CONFIRMED_FAILS {
                            error!("  Stale session detected - triggering rejoin");
                            is_joined = false;
                            join_attempt = 0;
                            confirmed_fail_count = 0;
                        }
                    }
                }
            }

            led.set_low();
            Timer::after_millis(100).await;
        }

        // If not joined, periodically retry joining (every 60 seconds)
        if !is_joined && uplink_counter >= 30 {
            uplink_counter = 0;
            join_attempt += 1;
            info!("Retrying OTAA join (attempt {})...", join_attempt);

            match device.join(&join_mode).await {
                Ok(JoinResponse::JoinSuccess) => {
                    info!("✓ LoRaWAN network joined successfully!");
                    is_joined = true;
                }
                Ok(JoinResponse::NoJoinAccept) => {
                    error!("✗ Join retry failed: No join accept received");
                }
                Err(err) => {
                    error!("✗ Join retry error: {:?}", err);
                }
            }
        }

        Timer::after_secs(2).await;
    }
}
