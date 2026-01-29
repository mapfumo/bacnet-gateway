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
use sh1106::{prelude::*, Builder};
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

// BME680 sensor constants
const BME680_ADDR_PRIMARY: u8 = 0x76;   // SDO pin LOW or floating
const BME680_ADDR_SECONDARY: u8 = 0x77; // SDO pin HIGH

// BME680 register addresses
const BME680_REG_CHIP_ID: u8 = 0xD0;
const BME680_REG_CTRL_MEAS: u8 = 0x74;
const BME680_REG_CTRL_HUM: u8 = 0x72;
const BME680_REG_CTRL_GAS_1: u8 = 0x71;  // Gas control register
const BME680_REG_CTRL_GAS_0: u8 = 0x70;  // Heater control
const BME680_REG_GAS_WAIT_0: u8 = 0x64;  // Gas wait time
const BME680_REG_RES_HEAT_0: u8 = 0x5A;  // Heater resistance
const BME680_REG_PRESS_MSB: u8 = 0x1F;
const BME680_REG_TEMP_MSB: u8 = 0x22;
const BME680_REG_HUM_MSB: u8 = 0x25;
const BME680_REG_GAS_R_MSB: u8 = 0x2A;   // Gas resistance MSB
const BME680_REG_GAS_R_LSB: u8 = 0x2B;   // Gas resistance LSB + range

// BME680 control values
const BME680_OSRS_H_X2: u8 = 0x02;  // Humidity oversampling x2

// LoRaWAN configuration constants
const MAX_TX_POWER: u8 = 14; // AU915 max TX power

// LoRaWAN credentials (from gateway TOT application)
// Note: EUIs are stored in LITTLE-ENDIAN for over-the-air transmission
// LoRa-2 uses different DevEUI (24ce... instead of 23ce...) to avoid conflicts
const DEV_EUI: [u8; 8] = [0xAC, 0x1F, 0x09, 0xFF, 0xFE, 0x1B, 0xCE, 0x24]; // 24ce1bfeff091fac reversed
const APP_EUI: [u8; 8] = [0x56, 0x53, 0x29, 0xC5, 0x64, 0xA8, 0x30, 0xB1]; // b130a864c5295356 reversed
const APP_KEY: [u8; 16] = [
    0xB7, 0x26, 0x73, 0x9B, 0x78, 0xEC, 0x4B, 0x9E,
    0x92, 0x34, 0xE5, 0xD3, 0x5E, 0xA9, 0x68, 0x1B,
]; // AppKey stays in big-endian (MSB first)

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("====================================");
    info!("  STM32WL55 LoRa-2 - BME688");
    info!("  Environmental Sensor + LoRaWAN");
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

        // Try to read BME688 chip ID at both addresses
        info!("Attempting to read BME680 chip ID...");
        let mut chip_id = [0u8; 1];

        // Try primary address (0x76)
        if i2c.blocking_write(BME680_ADDR_PRIMARY, &[BME680_REG_CHIP_ID]).is_ok()
            && i2c.blocking_read(BME680_ADDR_PRIMARY, &mut chip_id).is_ok() {
            info!("✓ BME680 found at 0x76 (primary), chip ID: 0x{:02X}", chip_id[0]);
        } else if i2c.blocking_write(BME680_ADDR_SECONDARY, &[BME680_REG_CHIP_ID]).is_ok()
            && i2c.blocking_read(BME680_ADDR_SECONDARY, &mut chip_id).is_ok() {
            info!("✓ BME680 found at 0x77 (secondary), chip ID: 0x{:02X}", chip_id[0]);
        } else {
            info!("✗ BME680 not responding at 0x76 or 0x77");
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
    let mut pressure_int = 0i16;
    let mut gas_int = 0u16;
    let mut tx_count = 0u32;
    let mut uplink_counter = 0u32;
    let mut snr = 0i8;
    let mut rssi = 0i16;
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

    // Closure to read BME680 sensor and update display without flicker
    // Uses fixed-width formatting and clears only line areas (not full screen)
    let read_sensor_and_update_display = |temp_int: &mut i16, hum_int: &mut i16, pressure_int: &mut i16, gas_int: &mut u16, tx_count: u32, is_joined: bool, join_attempt: u8, snr: i8, rssi: i16, display_initialized: &mut bool| {
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

        // Read BME680 sensor
        let mut bme_addr = BME680_ADDR_PRIMARY;
        if i2c.blocking_write(bme_addr, &[BME680_REG_CTRL_HUM, BME680_OSRS_H_X2]).is_err() {
            bme_addr = BME680_ADDR_SECONDARY;
        }

        if i2c.blocking_write(bme_addr, &[BME680_REG_CTRL_HUM, BME680_OSRS_H_X2]).is_ok() {
            if i2c.blocking_write(bme_addr, &[BME680_REG_CTRL_MEAS, 0x00]).is_ok() {
                // Small blocking delay
                cortex_m::asm::delay(960_000); // ~20ms at 48MHz

                // Configure gas heater
                let _ = i2c.blocking_write(bme_addr, &[BME680_REG_RES_HEAT_0, 0x73]);
                let _ = i2c.blocking_write(bme_addr, &[BME680_REG_GAS_WAIT_0, 0x59]);
                let _ = i2c.blocking_write(bme_addr, &[BME680_REG_CTRL_GAS_1, 0x10]);

                // Trigger forced measurement
                if i2c.blocking_write(bme_addr, &[BME680_REG_CTRL_MEAS, 0x25]).is_ok() {
                    // Wait for measurement (~2 seconds for gas heater)
                    cortex_m::asm::delay(96_000_000); // ~2000ms at 48MHz

                    // Read temperature
                    let mut temp_data = [0u8; 3];
                    if i2c.blocking_write(bme_addr, &[BME680_REG_TEMP_MSB]).is_ok()
                        && i2c.blocking_read(bme_addr, &mut temp_data).is_ok() {
                        let temp_raw = ((temp_data[0] as u32) << 12)
                                     | ((temp_data[1] as u32) << 4)
                                     | ((temp_data[2] as u32) >> 4);
                        *temp_int = (temp_raw / 18357) as i16;
                    }

                    // Read humidity
                    let mut hum_data = [0u8; 2];
                    if i2c.blocking_write(bme_addr, &[BME680_REG_HUM_MSB]).is_ok()
                        && i2c.blocking_read(bme_addr, &mut hum_data).is_ok() {
                        let hum_raw = ((hum_data[0] as u16) << 8) | (hum_data[1] as u16);
                        let hum_tenths = ((hum_raw as i32 * 10) / 386) as i16;
                        *hum_int = (hum_tenths / 10).clamp(0, 100);
                    }

                    // Read pressure
                    let mut press_data = [0u8; 3];
                    if i2c.blocking_write(bme_addr, &[BME680_REG_PRESS_MSB]).is_ok()
                        && i2c.blocking_read(bme_addr, &mut press_data).is_ok() {
                        let press_raw = ((press_data[0] as u32) << 12)
                                      | ((press_data[1] as u32) << 4)
                                      | ((press_data[2] as u32) >> 4);
                        let press_pa = ((press_raw * 295) / 1000) as u32;
                        *pressure_int = (press_pa / 100) as i16;
                    }

                    // Read gas resistance
                    let mut gas_data = [0u8; 2];
                    if i2c.blocking_write(bme_addr, &[BME680_REG_GAS_R_MSB]).is_ok()
                        && i2c.blocking_read(bme_addr, &mut gas_data).is_ok() {
                        let gas_adc = ((gas_data[0] as u16) << 2) | ((gas_data[1] as u16) >> 6);
                        let gas_range = gas_data[1] & 0x0F;
                        let gas_valid = (gas_data[1] >> 5) & 0x01;
                        let heat_stab = (gas_data[1] >> 4) & 0x01;

                        if gas_valid == 1 && heat_stab == 1 && gas_adc > 0 {
                            const GAS_RANGE_R2: [u32; 16] = [
                                4096000000, 2048000000, 1024000000, 512000000,
                                255744255, 127110228, 64000000, 32258064,
                                16016016, 8000000, 4000000, 2000000,
                                1000000, 500000, 250000, 125000,
                            ];
                            let range_idx = gas_range as usize;
                            if range_idx < 16 {
                                let var1 = GAS_RANGE_R2[range_idx] / (gas_adc as u32);
                                *gas_int = (var1 / 1000) as u16;
                            }
                        }
                    }

                    info!("✓ BME680: {}°C, {}% RH, {} hPa, {} kOhm", *temp_int, *hum_int, *pressure_int, *gas_int);
                }
            }
        }

        // Update OLED display (SH1106 128x64)
        let mut display: GraphicsMode<_> = Builder::new()
            .with_size(DisplaySize::Display128x64)
            .connect_i2c(i2c)
            .into();

        // Initialize display only on first call
        if !*display_initialized {
            if display.init().is_ok() {
                display.clear();
                let _ = display.flush();
                *display_initialized = true;
            }
        }

        // Clear buffer and redraw (buffer clear doesn't flicker - only flush sends to display)
        display.clear();

        // Line 1: Title + TX count or join status (fixed width)
        let mut line1 = heapless::String::<32>::new();
        if is_joined {
            let _ = core::fmt::write(&mut line1, format_args!("LoRa-2     Tx:{:>4}  ", tx_count));
        } else {
            let _ = core::fmt::write(&mut line1, format_args!("LoRa-2   Join:{:>2}    ", join_attempt));
        }
        let _ = Text::new(&line1, Point::new(0, 10), text_style).draw(&mut display);

        // Line 2: Temperature and Humidity (fixed width)
        let mut line2 = heapless::String::<32>::new();
        let _ = core::fmt::write(&mut line2, format_args!("Temp:{:>3}C  Hum:{:>3}%  ", *temp_int, *hum_int));
        let _ = Text::new(&line2, Point::new(0, 22), text_style).draw(&mut display);

        // Line 3: Pressure (fixed width)
        let mut line3 = heapless::String::<32>::new();
        let _ = core::fmt::write(&mut line3, format_args!("Press:{:>5} hPa     ", *pressure_int));
        let _ = Text::new(&line3, Point::new(0, 34), text_style).draw(&mut display);

        // Line 4: Gas resistance (fixed width)
        let mut line4 = heapless::String::<32>::new();
        let _ = core::fmt::write(&mut line4, format_args!("Gas:{:>5} kOhm      ", *gas_int));
        let _ = Text::new(&line4, Point::new(0, 46), text_style).draw(&mut display);

        // Line 5: Connection status (fixed width)
        let mut line5 = heapless::String::<32>::new();
        if !is_joined {
            let _ = core::fmt::write(&mut line5, format_args!("Connecting...       "));
        } else if rssi != 0 {
            let _ = core::fmt::write(&mut line5, format_args!("SNR:{:>3} RSSI:{:>4}  ", snr, rssi));
        } else {
            let _ = core::fmt::write(&mut line5, format_args!("Joined              "));
        }
        let _ = Text::new(&line5, Point::new(0, 58), text_style).draw(&mut display);

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
            &mut temp_int, &mut hum_int, &mut pressure_int, &mut gas_int,
            tx_count, is_joined, join_attempt, 0, 0, &mut display_initialized,
        );

        // Try to join
        match device.join(&join_mode).await {
            Ok(JoinResponse::JoinSuccess) => {
                info!("✓ LoRaWAN network joined successfully!");
                is_joined = true;
            }
            Ok(JoinResponse::NoJoinAccept) => {
                error!("✗ Join failed (attempt {}): No join accept received", join_attempt);
                if join_attempt < MAX_JOIN_ATTEMPTS {
                    // Wait ~6 seconds, updating display (BME680 takes ~2s per read)
                    for _ in 0..3 {
                        read_sensor_and_update_display(
                            &mut temp_int, &mut hum_int, &mut pressure_int, &mut gas_int,
                            tx_count, is_joined, join_attempt, 0, 0, &mut display_initialized,
                        );
                    }
                }
            }
            Err(err) => {
                error!("✗ Join error (attempt {}): {:?}", join_attempt, err);
                if join_attempt < MAX_JOIN_ATTEMPTS {
                    // Wait ~6 seconds, updating display
                    for _ in 0..3 {
                        read_sensor_and_update_display(
                            &mut temp_int, &mut hum_int, &mut pressure_int, &mut gas_int,
                            tx_count, is_joined, join_attempt, 0, 0, &mut display_initialized,
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
        let current_snr = if is_joined { snr } else { 0 };
        let current_rssi = if is_joined { rssi } else { 0 };

        read_sensor_and_update_display(
            &mut temp_int, &mut hum_int, &mut pressure_int, &mut gas_int,
            tx_count, is_joined, join_attempt, current_snr, current_rssi, &mut display_initialized,
        );

        // Send LoRaWAN uplink if joined and interval reached
        if is_joined && uplink_counter >= UPLINK_INTERVAL {
            uplink_counter = 0;

            let temp_encoded = (temp_int * 100) as i16;
            let hum_encoded = (hum_int * 100) as u16;
            let pressure_encoded = (pressure_int * 10) as u16;
            let gas_encoded = gas_int;

            let payload: [u8; 12] = [
                (temp_encoded >> 8) as u8,
                temp_encoded as u8,
                (hum_encoded >> 8) as u8,
                hum_encoded as u8,
                (pressure_encoded >> 8) as u8,
                pressure_encoded as u8,
                (gas_encoded >> 8) as u8,
                gas_encoded as u8,
                0, 0, 0, 0,
            ];

            // Every Nth uplink is confirmed to detect stale sessions
            let use_confirmed = (tx_count % CONFIRMED_UPLINK_INTERVAL as u32) == 0;

            if use_confirmed {
                info!("Sending CONFIRMED uplink: {}°C, {}%, {} hPa, {} kOhm", temp_int, hum_int, pressure_int, gas_int);
            } else {
                info!("Sending uplink: {}°C, {}%, {} hPa, {} kOhm", temp_int, hum_int, pressure_int, gas_int);
            }
            led.set_high();

            match device.send(&payload, 1, use_confirmed).await {
                Ok(response) => {
                    tx_count += 1;
                    snr = device.last_snr() as i8;
                    rssi = device.last_rssi();
                    info!("✓ Uplink sent successfully: {:?}", response);
                    info!("  SNR: {} dB, RSSI: {} dBm, TX count: {}", snr, rssi, tx_count);

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

        // If not joined, periodically retry joining (every ~40 seconds, accounting for ~2s sensor reads)
        if !is_joined && uplink_counter >= 20 {
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

        // No additional delay needed - BME680 sensor read already takes ~2 seconds
    }
}
