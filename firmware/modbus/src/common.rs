//! Common code shared between modbus_1 and modbus_2
//!
//! This module contains:
//! - Hardware initialization (W5500, SHT3x, OLED)
//! - Modbus TCP server implementation
//! - Sensor reading tasks
//! - OLED display tasks
//! - Modbus register map

use defmt::{info, warn};
use embassy_stm32::{
    bind_interrupts,
    gpio::{Level, Output, Speed},
    i2c::{Config as I2cConfig, EventInterruptHandler, ErrorInterruptHandler, I2c},
    peripherals,
    spi::{Config as SpiConfig, Spi},
    time::Hertz,
};
use embassy_time::Timer;

// OLED display imports
use ssd1306::{prelude::*, mode::BufferedGraphicsMode, I2CDisplayInterface, Ssd1306};
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    text::Text,
};
use core::fmt::Write;
use heapless::String;

// Bind I2C interrupts for SHT3x sensor (I2C1)
bind_interrupts!(struct I2c1Irqs {
    I2C1_EV => EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => ErrorInterruptHandler<peripherals::I2C1>;
});

// ============================================================================
// Modbus Register Map
// ============================================================================

/// Modbus register addresses (1-based addressing as per spec)
pub mod registers {
    pub const TEMP_REGISTERS: u16 = 40001;      // 40001-40002 (f32)
    pub const HUMIDITY_REGISTERS: u16 = 40003;  // 40003-40004 (f32)
    pub const STATUS_REGISTER: u16 = 40005;     // 40005 (u16)
    pub const UPTIME_REGISTERS: u16 = 40006;    // 40006-40007 (u32)
    pub const RESERVED_START: u16 = 40008;      // 40008-40010 (u16)
    pub const RESERVED_END: u16 = 40010;
}

/// Device status codes
pub mod status {
    pub const OK: u16 = 0;
    pub const SENSOR_ERROR: u16 = 1;
    pub const NETWORK_ERROR: u16 = 2;
}

// ============================================================================
// Hardware Initialization
// ============================================================================

/// Initialize hardware peripherals for Modbus TCP node
///
/// Returns (SPI, CS pin) so main loop can monitor socket status
///
/// # Arguments
/// * `board_id` - Identifier string for logging ("Board 1" or "Board 2")
/// * `ip_addr` - Static IP address [a, b, c, d]
/// * `mac_addr` - MAC address [a, b, c, d, e, f]
pub async fn init_hardware(
    board_id: &str,
    ip_addr: [u8; 4],
    mac_addr: [u8; 6],
) -> (
    Spi<'static, peripherals::SPI1, peripherals::DMA2_CH3, peripherals::DMA2_CH2>,
    Output<'static, peripherals::PB6>,
) {
    info!("Initializing hardware for {}", board_id);
    info!("IP: {}.{}.{}.{}", ip_addr[0], ip_addr[1], ip_addr[2], ip_addr[3]);

    // Initialize Embassy peripherals
    let p = embassy_stm32::init(Default::default());
    info!("Embassy peripherals initialized");

    // ========================================================================
    // W5500 SPI Configuration
    // ========================================================================
    // Pins: PA5 (SCK), PA6 (MISO), PA7 (MOSI), PB6 (CS), PC7 (RST)
    // NOTE: Using Morpho connector SPI1 pins, NOT Arduino connector!

    // Configure SPI1 at 10 MHz (W5500 supports up to 80 MHz, start conservative)
    let mut spi_config = SpiConfig::default();
    spi_config.frequency = Hertz(10_000_000); // 10 MHz

    info!("Configuring SPI1: SCK=PA5, MISO=PA6, MOSI=PA7");
    let mut spi = Spi::new(
        p.SPI1,
        p.PA5,  // SCK
        p.PA7,  // MOSI
        p.PA6,  // MISO
        p.DMA2_CH3, // TX DMA
        p.DMA2_CH2, // RX DMA
        spi_config,
    );
    info!("SPI1 initialized at 10 MHz");

    // Configure CS pin (PB6) - Active low, start HIGH (deselected)
    let mut cs_pin = Output::new(p.PB6, Level::High, Speed::VeryHigh);
    info!("CS pin configured: PB6 (initial: HIGH)");

    // Configure RST pin (PC7) - Active low, start HIGH (not in reset)
    let mut rst_pin = Output::new(p.PC7, Level::High, Speed::VeryHigh);
    info!("RST pin configured: PC7 (initial: HIGH)");

    // Test RST pin toggle (verify we can control it)
    info!("Testing RST pin: Pulsing LOW for 100ms");
    rst_pin.set_low();
    Timer::after_millis(100).await;
    rst_pin.set_high();
    info!("RST pin back HIGH - W5500 should be reset");

    // Wait for W5500 to complete reset (PLL lock time ~10ms typical)
    info!("Waiting 200ms for W5500 PLL to stabilize...");
    Timer::after_millis(200).await;

    // ========================================================================
    // Test W5500 Communication - Read Version Register
    // ========================================================================
    info!("Reading W5500 version register at 0x0039 (expecting 0x04)...");
    match w5500_read_register(&mut spi, &mut cs_pin, REG_VERSIONR).await {
        Ok(version) => {
            if version == 0x04 {
                info!("W5500 version: 0x{:02X} - CORRECT! SPI working! ✓", version);
            } else {
                warn!("W5500 version: 0x{:02X} - UNEXPECTED (expected 0x04)", version);
                warn!("This may indicate: wrong wiring, unpowered W5500, or SPI config issue");
                panic!("W5500 initialization failed - wrong version");
            }
        }
        Err(_) => {
            warn!("Failed to read W5500 version register - SPI communication error");
            panic!("W5500 SPI communication failed");
        }
    }

    // ========================================================================
    // Configure W5500 Network Settings
    // ========================================================================
    info!("Configuring W5500 network settings...");

    // Gateway address (10.10.10.1)
    let gateway = [10, 10, 10, 1];
    info!("Setting Gateway: {}.{}.{}.{}", gateway[0], gateway[1], gateway[2], gateway[3]);
    w5500_write_register(&mut spi, &mut cs_pin, REG_GAR0, &gateway)
        .await
        .expect("Failed to write gateway address");

    // Subnet mask (255.255.255.0)
    let subnet = [255, 255, 255, 0];
    info!("Setting Subnet: {}.{}.{}.{}", subnet[0], subnet[1], subnet[2], subnet[3]);
    w5500_write_register(&mut spi, &mut cs_pin, REG_SUBR0, &subnet)
        .await
        .expect("Failed to write subnet mask");

    // MAC address
    info!("Setting MAC: {:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
          mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    w5500_write_register(&mut spi, &mut cs_pin, REG_SHAR0, &mac_addr)
        .await
        .expect("Failed to write MAC address");

    // IP address
    info!("Setting IP: {}.{}.{}.{}", ip_addr[0], ip_addr[1], ip_addr[2], ip_addr[3]);
    w5500_write_register(&mut spi, &mut cs_pin, REG_SIPR0, &ip_addr)
        .await
        .expect("Failed to write IP address");

    info!("W5500 network configuration complete!");

    // ========================================================================
    // Open TCP Socket on Port 502 (Modbus TCP)
    // ========================================================================
    info!("Opening TCP socket on port 502...");

    // Step 0: Close socket first to ensure clean state
    info!("Ensuring Socket 0 is closed");
    w5500_write_socket_register(&mut spi, &mut cs_pin, REG_S0_CR, SOCK_CMD_CLOSE)
        .await
        .expect("Failed to send CLOSE command");
    Timer::after_millis(10).await;

    // Step 1: Set socket mode to TCP
    info!("Setting Socket 0 mode to TCP");
    w5500_write_socket_register(&mut spi, &mut cs_pin, REG_S0_MR, SOCK_MODE_TCP)
        .await
        .expect("Failed to set socket mode");

    // Step 2: Set source port (502 in big-endian)
    let port_bytes = [0x01, 0xF6]; // 502 = 0x01F6
    info!("Setting Socket 0 port to 502");
    w5500_write_socket_register_multi(&mut spi, &mut cs_pin, REG_S0_PORT0, &port_bytes)
        .await
        .expect("Failed to set socket port");

    // Step 3: Send OPEN command
    info!("Sending OPEN command to Socket 0");
    w5500_write_socket_register(&mut spi, &mut cs_pin, REG_S0_CR, SOCK_CMD_OPEN)
        .await
        .expect("Failed to send OPEN command");

    // Step 4: Wait for command to be processed
    Timer::after_millis(10).await;

    // Poll command register until it clears (max 100ms)
    let mut cmd_cleared = false;
    for _ in 0..10 {
        let cmd = w5500_read_socket_register(&mut spi, &mut cs_pin, REG_S0_CR)
            .await
            .expect("Failed to read command register");
        if cmd == 0x00 {
            cmd_cleared = true;
            break;
        }
        Timer::after_millis(10).await;
    }

    if !cmd_cleared {
        warn!("OPEN command did not clear after 100ms");
    }

    let status = w5500_read_socket_register(&mut spi, &mut cs_pin, REG_S0_SR)
        .await
        .expect("Failed to read socket status after OPEN");

    if status == SOCK_STATUS_INIT {
        info!("Socket 0 opened successfully (status: 0x{:02X})", status);
    } else {
        panic!("Socket 0 unexpected status after OPEN: 0x{:02X} (expected 0x{:02X})",
               status, SOCK_STATUS_INIT);
    }

    // Step 5: Send LISTEN command (TCP server mode)
    info!("Sending LISTEN command to Socket 0");
    w5500_write_socket_register(&mut spi, &mut cs_pin, REG_S0_CR, SOCK_CMD_LISTEN)
        .await
        .expect("Failed to send LISTEN command");

    // Step 6: Wait for command to be processed and status to change
    Timer::after_millis(10).await;

    // Poll command register until it clears (max 100ms)
    let mut cmd_cleared = false;
    for _ in 0..10 {
        let cmd = w5500_read_socket_register(&mut spi, &mut cs_pin, REG_S0_CR)
            .await
            .expect("Failed to read command register");
        if cmd == 0x00 {
            cmd_cleared = true;
            break;
        }
        Timer::after_millis(10).await;
    }

    if !cmd_cleared {
        warn!("LISTEN command did not clear after 100ms");
    }

    // Additional wait for status register to update
    Timer::after_millis(50).await;

    // Poll status register until it changes to LISTEN (max 200ms)
    let mut status = 0x00;
    let mut listen_achieved = false;
    for i in 0..20 {
        status = w5500_read_socket_register(&mut spi, &mut cs_pin, REG_S0_SR)
            .await
            .expect("Failed to read socket status");

        info!("Poll {} - Socket status: 0x{:02X}", i, status);

        if status == SOCK_STATUS_LISTEN {
            listen_achieved = true;
            break;
        }
        Timer::after_millis(10).await;
    }

    if listen_achieved {
        info!("Socket 0 LISTENING on port 502 (status: 0x{:02X}) ✓", status);
    } else {
        warn!("Socket 0 did not reach LISTEN state after 200ms");
        warn!("Final status: 0x{:02X} (expected 0x{:02X})", status, SOCK_STATUS_LISTEN);
        // Don't panic - continue anyway and see if it works
    }

    info!("TCP server ready on port 502!");
    info!("Hardware initialization complete for {}", board_id);

    // Return SPI and CS pin for socket monitoring
    (spi, cs_pin)
}

/// Check socket status and return current state
pub async fn check_socket_status(
    spi: &mut Spi<'_, peripherals::SPI1, peripherals::DMA2_CH3, peripherals::DMA2_CH2>,
    cs: &mut Output<'_, peripherals::PB6>,
) -> Result<u8, ()> {
    w5500_read_socket_register(spi, cs, REG_S0_SR).await
}

/// Close socket (send CLOSE command)
pub async fn close_socket(
    spi: &mut Spi<'_, peripherals::SPI1, peripherals::DMA2_CH3, peripherals::DMA2_CH2>,
    cs: &mut Output<'_, peripherals::PB6>,
) -> Result<(), ()> {
    w5500_write_socket_register(spi, cs, REG_S0_CR, SOCK_CMD_CLOSE).await?;
    Timer::after_millis(10).await;
    Ok(())
}

/// Send LISTEN command to socket (assumes socket is in INIT state)
pub async fn listen_socket(
    spi: &mut Spi<'_, peripherals::SPI1, peripherals::DMA2_CH3, peripherals::DMA2_CH2>,
    cs: &mut Output<'_, peripherals::PB6>,
) -> Result<(), ()> {
    w5500_write_socket_register(spi, cs, REG_S0_CR, SOCK_CMD_LISTEN).await?;
    Timer::after_millis(10).await;
    Ok(())
}

/// Reopen socket (CLOSE -> OPEN -> LISTEN sequence)
pub async fn reopen_socket(
    spi: &mut Spi<'_, peripherals::SPI1, peripherals::DMA2_CH3, peripherals::DMA2_CH2>,
    cs: &mut Output<'_, peripherals::PB6>,
) -> Result<(), ()> {
    // Step 1: Ensure socket is closed and wait for CLOSED status
    w5500_write_socket_register(spi, cs, REG_S0_CR, SOCK_CMD_CLOSE).await?;
    Timer::after_millis(10).await;

    // Wait for socket to reach CLOSED state
    let mut retries = 20;
    loop {
        let status = w5500_read_socket_register(spi, cs, REG_S0_SR).await?;
        if status == 0x00 {  // CLOSED
            break;
        }
        if retries == 0 {
            warn!("Socket did not close properly");
            return Err(());
        }
        retries -= 1;
        Timer::after_millis(10).await;
    }

    // Step 2: Set socket mode to TCP
    w5500_write_socket_register(spi, cs, REG_S0_MR, SOCK_MODE_TCP).await?;

    // Step 3: Set source port (502 in big-endian)
    let port_bytes = [0x01, 0xF6]; // 502 = 0x01F6
    w5500_write_socket_register(spi, cs, REG_S0_PORT0, port_bytes[0]).await?;
    w5500_write_socket_register(spi, cs, REG_S0_PORT0 + 1, port_bytes[1]).await?;

    // Step 4: Send OPEN command
    w5500_write_socket_register(spi, cs, REG_S0_CR, SOCK_CMD_OPEN).await?;
    Timer::after_millis(10).await;

    // Step 5: Wait for INIT status
    let mut retries = 20;
    loop {
        let status = w5500_read_socket_register(spi, cs, REG_S0_SR).await?;
        if status == SOCK_STATUS_INIT {
            break;
        }
        if retries == 0 {
            warn!("Socket did not reach INIT state");
            return Err(());
        }
        retries -= 1;
        Timer::after_millis(10).await;
    }

    // Step 6: Send LISTEN command
    w5500_write_socket_register(spi, cs, REG_S0_CR, SOCK_CMD_LISTEN).await?;
    Timer::after_millis(10).await;

    // Step 7: Wait for LISTEN status
    let mut retries = 20;
    loop {
        let status = w5500_read_socket_register(spi, cs, REG_S0_SR).await?;
        if status == SOCK_STATUS_LISTEN {
            info!("Socket reopened and listening on port 502");
            return Ok(());
        }
        if retries == 0 {
            warn!("Socket did not reach LISTEN state");
            return Err(());
        }
        retries -= 1;
        Timer::after_millis(10).await;
    }
}

/// Check how many bytes are available in RX buffer
pub async fn check_rx_size(
    spi: &mut Spi<'_, peripherals::SPI1, peripherals::DMA2_CH3, peripherals::DMA2_CH2>,
    cs: &mut Output<'_, peripherals::PB6>,
) -> Result<u16, ()> {
    // REG_S0_RX_RSR0 is a 2-byte register
    let high = w5500_read_socket_register(spi, cs, REG_S0_RX_RSR0).await?;
    let low = w5500_read_socket_register(spi, cs, REG_S0_RX_RSR0 + 1).await?;
    Ok(u16::from_be_bytes([high, low]))
}

/// Read data from RX buffer
///
/// Returns the number of bytes actually read (up to buffer.len())
pub async fn read_rx_data(
    spi: &mut Spi<'_, peripherals::SPI1, peripherals::DMA2_CH3, peripherals::DMA2_CH2>,
    cs: &mut Output<'_, peripherals::PB6>,
    buffer: &mut [u8],
) -> Result<u16, ()> {
    // Step 1: Check how many bytes are available
    let rx_size = check_rx_size(spi, cs).await?;
    if rx_size == 0 {
        return Ok(0);
    }

    // Step 2: Read current RX read pointer (2 bytes, big-endian)
    let ptr_high = w5500_read_socket_register(spi, cs, REG_S0_RX_RD0).await?;
    let ptr_low = w5500_read_socket_register(spi, cs, REG_S0_RX_RD0 + 1).await?;
    let rx_ptr = u16::from_be_bytes([ptr_high, ptr_low]);

    // Step 3: Read data from RX buffer (limited by buffer size)
    let bytes_to_read = rx_size.min(buffer.len() as u16);
    w5500_read_rx_buffer(spi, cs, rx_ptr, &mut buffer[..bytes_to_read as usize]).await?;

    // Step 4: Update RX read pointer
    let new_ptr = rx_ptr.wrapping_add(bytes_to_read);
    let new_ptr_bytes = new_ptr.to_be_bytes();
    w5500_write_socket_register(spi, cs, REG_S0_RX_RD0, new_ptr_bytes[0]).await?;
    w5500_write_socket_register(spi, cs, REG_S0_RX_RD0 + 1, new_ptr_bytes[1]).await?;

    // Step 5: Send RECV command to finalize read operation
    w5500_write_socket_register(spi, cs, REG_S0_CR, SOCK_CMD_RECV).await?;

    Ok(bytes_to_read)
}

/// Write data to TX buffer and send
///
/// Returns number of bytes written
pub async fn write_tx_data(
    spi: &mut Spi<'_, peripherals::SPI1, peripherals::DMA2_CH3, peripherals::DMA2_CH2>,
    cs: &mut Output<'_, peripherals::PB6>,
    data: &[u8],
) -> Result<u16, ()> {
    if data.is_empty() {
        return Ok(0);
    }

    // Step 1: Read current TX write pointer (2 bytes, big-endian)
    let ptr_high = w5500_read_socket_register(spi, cs, REG_S0_TX_WR0).await?;
    let ptr_low = w5500_read_socket_register(spi, cs, REG_S0_TX_WR0 + 1).await?;
    let tx_ptr = u16::from_be_bytes([ptr_high, ptr_low]);

    // Step 2: Write data to TX buffer
    let bytes_to_write = data.len() as u16;
    w5500_write_tx_buffer(spi, cs, tx_ptr, data).await?;

    // Step 3: Update TX write pointer
    let new_ptr = tx_ptr.wrapping_add(bytes_to_write);
    let new_ptr_bytes = new_ptr.to_be_bytes();
    w5500_write_socket_register(spi, cs, REG_S0_TX_WR0, new_ptr_bytes[0]).await?;
    w5500_write_socket_register(spi, cs, REG_S0_TX_WR0 + 1, new_ptr_bytes[1]).await?;

    // Step 4: Send SEND command to transmit the data
    w5500_write_socket_register(spi, cs, REG_S0_CR, SOCK_CMD_SEND).await?;

    // Step 5: Wait for SEND command to complete
    Timer::after_millis(10).await;

    Ok(bytes_to_write)
}

// ============================================================================
// W5500 Ethernet Functions
// ============================================================================

/// W5500 Control Phase bits
const CONTROL_PHASE_READ: u8 = 0x00;
const CONTROL_PHASE_WRITE: u8 = 0x04;

/// W5500 Block Select Bits (BSB) - Common Register block
const BSB_COMMON_REG: u8 = 0x00;
const BSB_SOCKET0_REG: u8 = 0x01;   // Socket 0 register block
const BSB_SOCKET0_TX: u8 = 0x02;    // Socket 0 TX buffer
const BSB_SOCKET0_RX: u8 = 0x03;    // Socket 0 RX buffer

/// W5500 Common Registers
const REG_VERSIONR: u16 = 0x0039;  // Chip Version Register (should be 0x04)
const REG_SHAR0: u16 = 0x0009;     // Source Hardware Address (MAC) - 6 bytes
const REG_SIPR0: u16 = 0x000F;     // Source IP Address - 4 bytes
const REG_SUBR0: u16 = 0x0005;     // Subnet Mask - 4 bytes
const REG_GAR0: u16 = 0x0001;      // Gateway Address - 4 bytes

/// W5500 Socket 0 Registers
const REG_S0_MR: u16 = 0x0000;      // Socket 0 Mode Register
const REG_S0_CR: u16 = 0x0001;      // Socket 0 Command Register
const REG_S0_SR: u16 = 0x0003;      // Socket 0 Status Register
const REG_S0_PORT0: u16 = 0x0004;   // Socket 0 Source Port (2 bytes)
const REG_S0_TX_FSR0: u16 = 0x0020; // Socket 0 TX Free Size (2 bytes)
const REG_S0_TX_WR0: u16 = 0x0024;  // Socket 0 TX Write Pointer (2 bytes)
const REG_S0_RX_RSR0: u16 = 0x0026; // Socket 0 RX Received Size (2 bytes)
const REG_S0_RX_RD0: u16 = 0x0028;  // Socket 0 RX Read Pointer (2 bytes)

/// Socket Mode Register values
const SOCK_MODE_TCP: u8 = 0x01;     // TCP mode

/// Socket Command Register values
const SOCK_CMD_OPEN: u8 = 0x01;     // Open socket
const SOCK_CMD_LISTEN: u8 = 0x02;   // Listen (TCP server)
const SOCK_CMD_SEND: u8 = 0x20;     // Send data (complete TX operation)
const SOCK_CMD_RECV: u8 = 0x40;     // Receive data (complete RX operation)
const SOCK_CMD_CLOSE: u8 = 0x10;    // Close socket

/// Socket Status Register values
const SOCK_STATUS_CLOSED: u8 = 0x00;
const SOCK_STATUS_INIT: u8 = 0x13;
const SOCK_STATUS_LISTEN: u8 = 0x14;
const SOCK_STATUS_ESTABLISHED: u8 = 0x17;

/// Read a single byte from W5500 common register
///
/// W5500 SPI Frame: [Address High] [Address Low] [Control] [Data...]
async fn w5500_read_register(
    spi: &mut Spi<'_, peripherals::SPI1, peripherals::DMA2_CH3, peripherals::DMA2_CH2>,
    cs: &mut Output<'_, peripherals::PB6>,
    address: u16,
) -> Result<u8, ()> {
    // W5500 control byte: [BSB(5 bits)][RWB(1 bit)][OM(2 bits)]
    // BSB = Block Select, RWB = Read/Write, OM = Operation Mode (Variable Data Length)
    let control = (BSB_COMMON_REG << 3) | CONTROL_PHASE_READ | 0x00; // VDM mode

    let addr_high = (address >> 8) as u8;
    let addr_low = (address & 0xFF) as u8;

    let mut tx_buf = [addr_high, addr_low, control, 0x00]; // Last byte is dummy for read
    let mut rx_buf = [0u8; 4];

    info!("SPI TX: [{:02X} {:02X} {:02X} {:02X}]", tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3]);

    // Select W5500 (CS low)
    cs.set_low();
    Timer::after_micros(1).await; // Small delay for CS setup

    // Perform SPI transaction
    let result = spi.transfer(&mut rx_buf, &tx_buf).await;

    // Deselect W5500 (CS high)
    Timer::after_micros(1).await;
    cs.set_high();

    info!("SPI RX: [{:02X} {:02X} {:02X} {:02X}]", rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3]);

    match result {
        Ok(_) => Ok(rx_buf[3]), // Data is in the 4th byte
        Err(_) => Err(()),
    }
}

/// Write multiple bytes to W5500 common register
///
/// W5500 SPI Frame: [Address High] [Address Low] [Control] [Data...]
async fn w5500_write_register(
    spi: &mut Spi<'_, peripherals::SPI1, peripherals::DMA2_CH3, peripherals::DMA2_CH2>,
    cs: &mut Output<'_, peripherals::PB6>,
    address: u16,
    data: &[u8],
) -> Result<(), ()> {
    // W5500 control byte: [BSB(5 bits)][RWB(1 bit)][OM(2 bits)]
    let control = (BSB_COMMON_REG << 3) | CONTROL_PHASE_WRITE | 0x00; // VDM mode

    let addr_high = (address >> 8) as u8;
    let addr_low = (address & 0xFF) as u8;

    // Build TX buffer: [AddrH, AddrL, Control, ...data]
    let mut tx_buf = [0u8; 32]; // Max 32 bytes for MAC + IP + subnet + gateway
    let len = 3 + data.len();

    tx_buf[0] = addr_high;
    tx_buf[1] = addr_low;
    tx_buf[2] = control;
    tx_buf[3..len].copy_from_slice(data);

    info!("W5500 WRITE to 0x{:04X}: {} bytes", address, data.len());

    // Select W5500 (CS low)
    cs.set_low();
    Timer::after_micros(1).await;

    // Perform SPI write
    let result = spi.write(&tx_buf[..len]).await;

    // Deselect W5500 (CS high)
    Timer::after_micros(1).await;
    cs.set_high();

    match result {
        Ok(_) => Ok(()),
        Err(_) => Err(()),
    }
}

/// Read a single byte from W5500 socket register
async fn w5500_read_socket_register(
    spi: &mut Spi<'_, peripherals::SPI1, peripherals::DMA2_CH3, peripherals::DMA2_CH2>,
    cs: &mut Output<'_, peripherals::PB6>,
    address: u16,
) -> Result<u8, ()> {
    let control = (BSB_SOCKET0_REG << 3) | CONTROL_PHASE_READ | 0x00; // VDM mode

    let addr_high = (address >> 8) as u8;
    let addr_low = (address & 0xFF) as u8;

    let tx_buf = [addr_high, addr_low, control, 0x00];
    let mut rx_buf = [0u8; 4];

    cs.set_low();
    Timer::after_micros(1).await;
    let result = spi.transfer(&mut rx_buf, &tx_buf).await;
    Timer::after_micros(1).await;
    cs.set_high();

    match result {
        Ok(_) => Ok(rx_buf[3]),
        Err(_) => Err(()),
    }
}

/// Write single byte to W5500 socket register
async fn w5500_write_socket_register(
    spi: &mut Spi<'_, peripherals::SPI1, peripherals::DMA2_CH3, peripherals::DMA2_CH2>,
    cs: &mut Output<'_, peripherals::PB6>,
    address: u16,
    data: u8,
) -> Result<(), ()> {
    let control = (BSB_SOCKET0_REG << 3) | CONTROL_PHASE_WRITE | 0x00; // VDM mode

    let addr_high = (address >> 8) as u8;
    let addr_low = (address & 0xFF) as u8;

    let tx_buf = [addr_high, addr_low, control, data];

    cs.set_low();
    Timer::after_micros(1).await;
    let result = spi.write(&tx_buf).await;
    Timer::after_micros(1).await;
    cs.set_high();

    match result {
        Ok(_) => Ok(()),
        Err(_) => Err(()),
    }
}

/// Write multiple bytes to W5500 socket register
async fn w5500_write_socket_register_multi(
    spi: &mut Spi<'_, peripherals::SPI1, peripherals::DMA2_CH3, peripherals::DMA2_CH2>,
    cs: &mut Output<'_, peripherals::PB6>,
    address: u16,
    data: &[u8],
) -> Result<(), ()> {
    let control = (BSB_SOCKET0_REG << 3) | CONTROL_PHASE_WRITE | 0x00; // VDM mode

    let addr_high = (address >> 8) as u8;
    let addr_low = (address & 0xFF) as u8;

    let mut tx_buf = [0u8; 16];
    let len = 3 + data.len();

    tx_buf[0] = addr_high;
    tx_buf[1] = addr_low;
    tx_buf[2] = control;
    tx_buf[3..len].copy_from_slice(data);

    cs.set_low();
    Timer::after_micros(1).await;
    let result = spi.write(&tx_buf[..len]).await;
    Timer::after_micros(1).await;
    cs.set_high();

    match result {
        Ok(_) => Ok(()),
        Err(_) => Err(()),
    }
}

/// Read data from W5500 RX buffer block
///
/// This reads from the Socket 0 RX buffer (BSB=0x03)
async fn w5500_read_rx_buffer(
    spi: &mut Spi<'_, peripherals::SPI1, peripherals::DMA2_CH3, peripherals::DMA2_CH2>,
    cs: &mut Output<'_, peripherals::PB6>,
    address: u16,
    buffer: &mut [u8],
) -> Result<(), ()> {
    let control = (BSB_SOCKET0_RX << 3) | CONTROL_PHASE_READ | 0x00; // VDM mode

    let addr_high = (address >> 8) as u8;
    let addr_low = (address & 0xFF) as u8;

    // Build TX buffer: [AddrH, AddrL, Control, ...dummy bytes for reading]
    let mut tx_buf = [0u8; 256]; // Max Modbus frame is 260 bytes
    let len = 3 + buffer.len();

    tx_buf[0] = addr_high;
    tx_buf[1] = addr_low;
    tx_buf[2] = control;
    // Remaining bytes are dummy for read operation

    let mut rx_buf = [0u8; 256];

    cs.set_low();
    Timer::after_micros(1).await;
    let result = spi.transfer(&mut rx_buf[..len], &tx_buf[..len]).await;
    Timer::after_micros(1).await;
    cs.set_high();

    match result {
        Ok(_) => {
            // Data starts at byte 3 (after address and control)
            buffer.copy_from_slice(&rx_buf[3..len]);
            Ok(())
        }
        Err(_) => Err(()),
    }
}

/// Write data to W5500 TX buffer block
///
/// This writes to the Socket 0 TX buffer (BSB=0x02)
async fn w5500_write_tx_buffer(
    spi: &mut Spi<'_, peripherals::SPI1, peripherals::DMA2_CH3, peripherals::DMA2_CH2>,
    cs: &mut Output<'_, peripherals::PB6>,
    address: u16,
    data: &[u8],
) -> Result<(), ()> {
    let control = (BSB_SOCKET0_TX << 3) | CONTROL_PHASE_WRITE | 0x00; // VDM mode

    let addr_high = (address >> 8) as u8;
    let addr_low = (address & 0xFF) as u8;

    // Build TX buffer: [AddrH, AddrL, Control, ...data]
    let mut tx_buf = [0u8; 256]; // Max Modbus frame is 260 bytes
    let len = 3 + data.len();

    tx_buf[0] = addr_high;
    tx_buf[1] = addr_low;
    tx_buf[2] = control;
    tx_buf[3..len].copy_from_slice(data);

    cs.set_low();
    Timer::after_micros(1).await;
    let result = spi.write(&tx_buf[..len]).await;
    Timer::after_micros(1).await;
    cs.set_high();

    match result {
        Ok(_) => Ok(()),
        Err(_) => Err(()),
    }
}

// ============================================================================
// Modbus TCP Server
// ============================================================================

/// Modbus TCP MBAP Header (7 bytes)
#[derive(Debug)]
pub struct MbapHeader {
    pub transaction_id: u16,  // 2 bytes - copied from request
    pub protocol_id: u16,     // 2 bytes - always 0x0000 for Modbus
    pub length: u16,          // 2 bytes - number of following bytes (unit_id + PDU)
    pub unit_id: u8,          // 1 byte - usually 0x00 for TCP
}

impl MbapHeader {
    /// Parse MBAP header from 7-byte buffer
    pub fn from_bytes(data: &[u8]) -> Result<Self, ()> {
        if data.len() < 7 {
            return Err(());
        }

        Ok(MbapHeader {
            transaction_id: u16::from_be_bytes([data[0], data[1]]),
            protocol_id: u16::from_be_bytes([data[2], data[3]]),
            length: u16::from_be_bytes([data[4], data[5]]),
            unit_id: data[6],
        })
    }

    /// Write MBAP header to buffer
    pub fn to_bytes(&self, buffer: &mut [u8]) -> Result<(), ()> {
        if buffer.len() < 7 {
            return Err(());
        }

        buffer[0..2].copy_from_slice(&self.transaction_id.to_be_bytes());
        buffer[2..4].copy_from_slice(&self.protocol_id.to_be_bytes());
        buffer[4..6].copy_from_slice(&self.length.to_be_bytes());
        buffer[6] = self.unit_id;

        Ok(())
    }
}

/// Modbus function codes
pub mod function_codes {
    pub const READ_HOLDING_REGISTERS: u8 = 0x03;
    pub const READ_INPUT_REGISTERS: u8 = 0x04;
}

/// Modbus exception codes
pub mod exception_codes {
    pub const ILLEGAL_FUNCTION: u8 = 0x01;
    pub const ILLEGAL_DATA_ADDRESS: u8 = 0x02;
    pub const ILLEGAL_DATA_VALUE: u8 = 0x03;
}

/// Parse Modbus TCP request and return (function_code, start_addr, count)
pub fn parse_modbus_request(data: &[u8]) -> Result<(u8, u16, u16), u8> {
    if data.len() < 5 {
        return Err(exception_codes::ILLEGAL_DATA_VALUE);
    }

    let function_code = data[0];
    let start_addr = u16::from_be_bytes([data[1], data[2]]);
    let count = u16::from_be_bytes([data[3], data[4]]);

    // Validate function code
    if function_code != function_codes::READ_HOLDING_REGISTERS
        && function_code != function_codes::READ_INPUT_REGISTERS {
        return Err(exception_codes::ILLEGAL_FUNCTION);
    }

    // Validate count (max 125 registers for Modbus TCP)
    if count == 0 || count > 125 {
        return Err(exception_codes::ILLEGAL_DATA_VALUE);
    }

    Ok((function_code, start_addr, count))
}

/// Sensor data structure for Modbus register mapping
#[derive(Clone, Copy)]
pub struct SensorData {
    pub temperature: f32,    // Celsius
    pub humidity: f32,       // Percentage (0-100)
    pub status: u16,         // Status code (0=OK, 1=Error, etc.)
    pub uptime: u32,         // Uptime in seconds
}

impl Default for SensorData {
    fn default() -> Self {
        SensorData {
            temperature: 25.5,   // Mock temperature
            humidity: 60.0,      // Mock humidity
            status: status::OK,
            uptime: 0,
        }
    }
}

/// Handle Modbus FC03 (Read Holding Registers)
///
/// Maps Modbus register addresses to sensor data:
/// - 40001-40002: Temperature (f32, IEEE 754)
/// - 40003-40004: Humidity (f32, IEEE 754)
/// - 40005: Status (u16)
/// - 40006-40007: Uptime (u32)
///
/// Note: Modbus uses 1-based addressing, but we convert to 0-based internally
pub fn handle_read_registers(
    start_addr: u16,
    count: u16,
    sensor_data: &SensorData,
    response_buffer: &mut [u8],
) -> Result<usize, u8> {
    // Modbus register addresses are 1-based (40001, 40002, etc.)
    // We need to map them to our 0-based array

    let mut pos = 0;

    for i in 0..count {
        let reg_addr = start_addr + i;

        // Map register address to data
        let reg_value = match reg_addr {
            // Temperature: registers 0-1 (Modbus 40001-40002)
            0 => {
                let temp_regs = f32_to_registers(sensor_data.temperature);
                temp_regs[0]
            }
            1 => {
                let temp_regs = f32_to_registers(sensor_data.temperature);
                temp_regs[1]
            }
            // Humidity: registers 2-3 (Modbus 40003-40004)
            2 => {
                let hum_regs = f32_to_registers(sensor_data.humidity);
                hum_regs[0]
            }
            3 => {
                let hum_regs = f32_to_registers(sensor_data.humidity);
                hum_regs[1]
            }
            // Status: register 4 (Modbus 40005)
            4 => sensor_data.status,
            // Uptime: registers 5-6 (Modbus 40006-40007)
            5 => {
                let uptime_regs = u32_to_registers(sensor_data.uptime);
                uptime_regs[0]
            }
            6 => {
                let uptime_regs = u32_to_registers(sensor_data.uptime);
                uptime_regs[1]
            }
            // Reserved: registers 7-9 (Modbus 40008-40010)
            7..=9 => 0x0000,
            // Out of range
            _ => return Err(exception_codes::ILLEGAL_DATA_ADDRESS),
        };

        // Write register value to response buffer (big-endian)
        if pos + 2 > response_buffer.len() {
            return Err(exception_codes::ILLEGAL_DATA_VALUE);
        }
        response_buffer[pos..pos + 2].copy_from_slice(&reg_value.to_be_bytes());
        pos += 2;
    }

    Ok(pos)
}

/// Handle Modbus FC04 (Read Input Registers)
// TODO: Implement FC04 handler (similar to FC03)

// ============================================================================
// Sensor Tasks
// ============================================================================

/// Initialize SHT31-D sensor on I2C1
///
/// Pins: PB8 (SCL), PB9 (SDA) - must be configured as open-drain
///
/// Note: SHT31-D uses I2C address 0x44 (default)
pub async fn init_sht3x() -> I2c<'static, peripherals::I2C1, peripherals::DMA1_CH6, peripherals::DMA1_CH0> {
    info!("Initializing SHT31-D sensor on I2C1");

    // Get peripherals
    let p = unsafe { embassy_stm32::Peripherals::steal() };

    // Configure I2C1 at 100 kHz (standard mode)
    // Important: Disable internal pull-ups - rely on external 4.7k pull-ups
    let mut i2c_config = I2cConfig::default();
    i2c_config.sda_pullup = false;  // Disable internal pull-ups (use external)
    i2c_config.scl_pullup = false;  // Disable internal pull-ups (use external)

    info!("Configuring I2C1: SCL=PB8, SDA=PB9 (open-drain mode)");
    let mut i2c = I2c::new(
        p.I2C1,
        p.PB8,  // SCL (D15 on NUCLEO)
        p.PB9,  // SDA (D14 on NUCLEO)
        I2c1Irqs,
        p.DMA1_CH6, // TX DMA (I2C1 uses DMA1_CH6 or DMA1_CH7 for TX)
        p.DMA1_CH0, // RX DMA (I2C1 uses DMA1_CH0 or DMA1_CH5 for RX)
        Hertz(100_000), // 100 kHz
        i2c_config,
    );

    // Wait for sensor power-on (sensor needs time to stabilize)
    Timer::after_millis(100).await;

    // Test communication with soft reset command
    info!("Sending soft reset to SHT31-D...");
    let reset_cmd = [0x30, 0xA2];
    match i2c.write(0x44, &reset_cmd).await {
        Ok(_) => {
            info!("Soft reset sent successfully");
            // Wait for reset to complete
            Timer::after_millis(20).await;
        }
        Err(_) => {
            warn!("Failed to send soft reset - I2C communication error");
            warn!("Check wiring: SCL=PB8, SDA=PB9, VCC=3.3V, GND=GND");
            warn!("Ensure 4.7kΩ pull-up resistors are present on SCL and SDA");
        }
    }

    info!("SHT31-D sensor initialized");
    i2c
}

/// Read temperature and humidity from SHT31-D sensor
///
/// Returns (temperature_celsius, humidity_percent) or error
///
/// Uses high repeatability measurement (most accurate)
pub async fn read_sht3x(
    i2c: &mut I2c<'_, peripherals::I2C1, peripherals::DMA1_CH6, peripherals::DMA1_CH0>
) -> Result<(f32, f32), ()> {
    // SHT31-D command: 0x2400 (High repeatability measurement, clock stretching disabled)
    let cmd = [0x24, 0x00];

    // Step 1: Send measurement command
    if let Err(_) = i2c.write(0x44, &cmd).await {
        warn!("Failed to send measurement command to SHT31-D");
        return Err(());
    }

    // Step 2: Wait for measurement to complete (CRITICAL - must wait 20ms minimum)
    Timer::after_millis(20).await;

    // Step 3: Read 6 bytes (Temp MSB/LSB + CRC, Humidity MSB/LSB + CRC)
    let mut data = [0u8; 6];
    match i2c.read(0x44, &mut data).await {
        Ok(_) => {
            // Extract temperature (first 2 bytes, ignore CRC at data[2])
            let temp_raw = u16::from_be_bytes([data[0], data[1]]);
            // Extract humidity (bytes 3-4, ignore CRC at data[5])
            let hum_raw = u16::from_be_bytes([data[3], data[4]]);

            // Convert to physical units (SHT31-D datasheet formulas)
            let temp_c = -45.0 + 175.0 * (temp_raw as f32 / 65535.0);
            let hum_pct = 100.0 * (hum_raw as f32 / 65535.0);

            Ok((temp_c, hum_pct))
        }
        Err(_) => {
            warn!("Failed to read SHT31-D sensor data");
            Err(())
        }
    }
}

// ============================================================================
// OLED Display Tasks
// ============================================================================

use embassy_stm32::dma::NoDma;

/// OLED Display type (using blocking I2C without DMA on I2C1, shared with SHT31-D)
pub type OledDisplay = Ssd1306<
    I2CInterface<I2c<'static, peripherals::I2C1, NoDma, NoDma>>,
    DisplaySize128x64,
    BufferedGraphicsMode<DisplaySize128x64>
>;

/// Initialize SSD1306 OLED display on I2C1 (shared with SHT31-D)
///
/// Pins: PB8 (SCL), PB9 (SDA) - same physical bus as SHT31-D sensor
/// Address: 0x3C (default for most SSD1306 displays)
///
/// NOTE: This steals the I2C1 peripheral a second time without DMA.
/// Works because we don't access sensor and display simultaneously.
pub async fn init_oled() -> OledDisplay {
    info!("Initializing SSD1306 OLED display on I2C1 (shared bus)");

    // Steal peripherals again for OLED
    let p = unsafe { embassy_stm32::Peripherals::steal() };

    // Configure I2C1 at 100 kHz (standard mode, same as SHT31-D)
    // Note: Using blocking I2C (no DMA) for ssd1306 compatibility
    let mut i2c_config = I2cConfig::default();
    i2c_config.sda_pullup = false;  // Use external pull-ups
    i2c_config.scl_pullup = false;  // Use external pull-ups

    info!("Configuring I2C1 for OLED: SCL=PB8, SDA=PB9 (blocking mode, no DMA)");
    let i2c = I2c::new(
        p.I2C1,
        p.PB8,  // SCL (D15 on NUCLEO) - shared with SHT31-D
        p.PB9,  // SDA (D14 on NUCLEO) - shared with SHT31-D
        I2c1Irqs,
        NoDma,  // No TX DMA for blocking I2C
        NoDma,  // No RX DMA for blocking I2C
        Hertz(100_000), // 100 kHz
        i2c_config,
    );

    // Create display interface
    let interface = I2CDisplayInterface::new(i2c);

    // Create display driver (128x64)
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();

    // Initialize display
    match display.init() {
        Ok(_) => {
            display.clear(BinaryColor::Off).ok();
            display.flush().ok();
            info!("OLED display initialized successfully");
        }
        Err(_) => {
            warn!("Failed to initialize OLED display");
            warn!("Check wiring: SCL=PB8 (D15), SDA=PB9 (D14), VCC=3.3V, GND=GND");
            warn!("Continuing without OLED...");
        }
    }

    display
}

/// Display startup banner on OLED
pub fn display_startup(
    display: &mut OledDisplay,
    board_id: &str,
    ip: [u8; 4],
) {
    let text_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);

    if display.clear(BinaryColor::Off).is_err() {
        return; // Display not working, skip silently
    }

    // Line 1: Board ID
    let mut text: String<32> = String::new();
    let _ = write!(text, "{}", board_id);
    let _ = Text::new(&text, Point::new(0, 10), text_style).draw(display);

    // Line 2: IP Address
    text.clear();
    let _ = write!(text, "IP: {}.{}.{}.{}", ip[0], ip[1], ip[2], ip[3]);
    let _ = Text::new(&text, Point::new(0, 22), text_style).draw(display);

    // Line 3: Status
    let _ = Text::new("Initializing...", Point::new(0, 34), text_style).draw(display);

    let _ = display.flush();
}

/// Update OLED display with current sensor data
pub fn update_display(
    display: &mut OledDisplay,
    sensor_data: &SensorData,
    board_id: &str,
    ip: [u8; 4],
    connected: bool,
) {
    let text_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);

    if display.clear(BinaryColor::Off).is_err() {
        return; // Display not working, skip silently
    }

    // Line 1: Board ID
    let mut text: String<32> = String::new();
    let _ = write!(text, "{}", board_id);
    let _ = Text::new(&text, Point::new(0, 10), text_style).draw(display);

    // Line 2: IP Address
    text.clear();
    let _ = write!(text, "{}.{}.{}.{}:502", ip[0], ip[1], ip[2], ip[3]);
    let _ = Text::new(&text, Point::new(0, 22), text_style).draw(display);

    // Line 3: Temperature
    text.clear();
    let _ = write!(text, "T: {:.1}C", sensor_data.temperature);
    let _ = Text::new(&text, Point::new(0, 34), text_style).draw(display);

    // Line 4: Humidity
    text.clear();
    let _ = write!(text, "H: {:.1}%", sensor_data.humidity);
    let _ = Text::new(&text, Point::new(0, 46), text_style).draw(display);

    // Line 5: Connection status
    text.clear();
    if connected {
        let _ = write!(text, "CONNECTED");
    } else {
        let _ = write!(text, "LISTENING");
    }
    let _ = Text::new(&text, Point::new(0, 58), text_style).draw(display);

    let _ = display.flush();
}

// ============================================================================
// Shared State
// ============================================================================

// TODO: Define shared state for sensor data
// use embassy_sync::mutex::Mutex;
// use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
//
// pub struct SensorData {
//     pub temperature: f32,
//     pub humidity: f32,
//     pub status: u16,
//     pub uptime: u32,
// }
//
// static SENSOR_DATA: Mutex<CriticalSectionRawMutex, SensorData> = Mutex::new(SensorData {
//     temperature: 0.0,
//     humidity: 0.0,
//     status: status::OK,
//     uptime: 0,
// });

// ============================================================================
// Helper Functions
// ============================================================================

/// Convert f32 to two u16 registers (IEEE 754)
pub fn f32_to_registers(value: f32) -> [u16; 2] {
    let bytes = value.to_be_bytes();
    [
        u16::from_be_bytes([bytes[0], bytes[1]]),
        u16::from_be_bytes([bytes[2], bytes[3]]),
    ]
}

/// Convert u32 to two u16 registers
pub fn u32_to_registers(value: u32) -> [u16; 2] {
    [(value >> 16) as u16, (value & 0xFFFF) as u16]
}
