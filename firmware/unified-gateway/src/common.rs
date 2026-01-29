//! Common code for Unified IIoT Gateway (Modbus TCP + BACnet/IP)
//!
//! This module contains:
//! - Hardware initialization (W5500, SHT3x, OLED)
//! - W5500 TCP socket functions (Socket 0) for Modbus TCP
//! - W5500 UDP socket functions (Socket 1) for BACnet/IP
//! - Sensor reading tasks
//! - OLED display tasks
//!
//! # W5500 Socket Allocation
//!
//! | Socket | Protocol | Port  | Purpose          |
//! |--------|----------|-------|------------------|
//! | 0      | TCP      | 502   | Modbus TCP       |
//! | 1      | UDP      | 47808 | BACnet/IP        |
//!
//! Derived from wk11-unified-monitoring with dual-protocol support.

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
// Device Status Codes
// ============================================================================

pub mod status {
    pub const OK: u16 = 0;
    pub const SENSOR_ERROR: u16 = 1;
    pub const NETWORK_ERROR: u16 = 2;
}

// ============================================================================
// Hardware Initialization
// ============================================================================

/// Initialize hardware peripherals for BACnet/IP node
///
/// Returns (SPI, CS pin) so main loop can handle BACnet packets
///
/// # Arguments
/// * `board_id` - Identifier string for logging
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

    // Reset W5500
    info!("Resetting W5500...");
    rst_pin.set_low();
    Timer::after_millis(100).await;
    rst_pin.set_high();
    info!("W5500 reset complete");

    // Wait for W5500 PLL to stabilize
    info!("Waiting 200ms for W5500 PLL...");
    Timer::after_millis(200).await;

    // ========================================================================
    // Test W5500 Communication
    // ========================================================================
    info!("Reading W5500 version register (expecting 0x04)...");
    match w5500_read_register(&mut spi, &mut cs_pin, REG_VERSIONR).await {
        Ok(version) => {
            if version == 0x04 {
                info!("W5500 version: 0x{:02X} - SPI working!", version);
            } else {
                warn!("W5500 version: 0x{:02X} - UNEXPECTED", version);
                panic!("W5500 initialization failed");
            }
        }
        Err(_) => {
            warn!("Failed to read W5500 version register");
            panic!("W5500 SPI communication failed");
        }
    }

    // ========================================================================
    // Configure W5500 Network Settings
    // ========================================================================
    info!("Configuring W5500 network...");

    // Gateway address (10.10.10.1)
    let gateway = [10, 10, 10, 1];
    w5500_write_register(&mut spi, &mut cs_pin, REG_GAR0, &gateway)
        .await
        .expect("Failed to write gateway");

    // Subnet mask (255.255.255.0)
    let subnet = [255, 255, 255, 0];
    w5500_write_register(&mut spi, &mut cs_pin, REG_SUBR0, &subnet)
        .await
        .expect("Failed to write subnet");

    // MAC address
    w5500_write_register(&mut spi, &mut cs_pin, REG_SHAR0, &mac_addr)
        .await
        .expect("Failed to write MAC");

    // IP address
    w5500_write_register(&mut spi, &mut cs_pin, REG_SIPR0, &ip_addr)
        .await
        .expect("Failed to write IP");

    info!("W5500 network configuration complete!");
    info!("Hardware initialization complete for {}", board_id);

    (spi, cs_pin)
}

// ============================================================================
// W5500 TCP Socket Functions (Socket 0 - Modbus TCP)
// ============================================================================

/// Socket 0 is reserved for TCP (Modbus)
const TCP_SOCKET: u8 = 0;

/// Open TCP server socket on specified port
///
/// Uses Socket 0 for Modbus TCP (port 502)
pub async fn open_tcp_server(
    spi: &mut Spi<'_, peripherals::SPI1, peripherals::DMA2_CH3, peripherals::DMA2_CH2>,
    cs: &mut Output<'_, peripherals::PB6>,
    port: u16,
) -> Result<(), ()> {
    info!("Opening TCP server on port {}...", port);

    // Close socket first to ensure clean state
    w5500_write_socket_register(spi, cs, TCP_SOCKET, REG_SN_CR, SOCK_CMD_CLOSE).await?;
    Timer::after_millis(10).await;

    // Wait for socket to close
    for _ in 0..20 {
        let status = w5500_read_socket_register(spi, cs, TCP_SOCKET, REG_SN_SR).await?;
        if status == SOCK_STATUS_CLOSED {
            break;
        }
        Timer::after_millis(10).await;
    }

    // Set socket mode to TCP
    w5500_write_socket_register(spi, cs, TCP_SOCKET, REG_SN_MR, SOCK_MODE_TCP).await?;

    // Set source port
    let port_bytes = port.to_be_bytes();
    w5500_write_socket_register(spi, cs, TCP_SOCKET, REG_SN_PORT0, port_bytes[0]).await?;
    w5500_write_socket_register(spi, cs, TCP_SOCKET, REG_SN_PORT0 + 1, port_bytes[1]).await?;

    // Send OPEN command
    w5500_write_socket_register(spi, cs, TCP_SOCKET, REG_SN_CR, SOCK_CMD_OPEN).await?;
    Timer::after_millis(10).await;

    // Verify socket opened in INIT state
    let status = w5500_read_socket_register(spi, cs, TCP_SOCKET, REG_SN_SR).await?;
    if status == SOCK_STATUS_INIT {
        info!("TCP socket in INIT state");

        // Send LISTEN command to start accepting connections
        w5500_write_socket_register(spi, cs, TCP_SOCKET, REG_SN_CR, SOCK_CMD_LISTEN).await?;
        Timer::after_millis(10).await;

        let status = w5500_read_socket_register(spi, cs, TCP_SOCKET, REG_SN_SR).await?;
        if status == SOCK_STATUS_LISTEN {
            info!("TCP server listening on port {}", port);
            Ok(())
        } else {
            warn!("TCP listen failed, status: 0x{:02X}", status);
            Err(())
        }
    } else {
        warn!("TCP socket failed to open, status: 0x{:02X}", status);
        Err(())
    }
}

/// Get TCP socket status
pub async fn tcp_get_status(
    spi: &mut Spi<'_, peripherals::SPI1, peripherals::DMA2_CH3, peripherals::DMA2_CH2>,
    cs: &mut Output<'_, peripherals::PB6>,
) -> Result<u8, ()> {
    w5500_read_socket_register(spi, cs, TCP_SOCKET, REG_SN_SR).await
}

/// Check if TCP data is available
pub async fn tcp_rx_available(
    spi: &mut Spi<'_, peripherals::SPI1, peripherals::DMA2_CH3, peripherals::DMA2_CH2>,
    cs: &mut Output<'_, peripherals::PB6>,
) -> Result<u16, ()> {
    let high = w5500_read_socket_register(spi, cs, TCP_SOCKET, REG_SN_RX_RSR0).await?;
    let low = w5500_read_socket_register(spi, cs, TCP_SOCKET, REG_SN_RX_RSR0 + 1).await?;
    Ok(u16::from_be_bytes([high, low]))
}

/// Receive TCP data
///
/// Returns number of bytes read
pub async fn tcp_recv(
    spi: &mut Spi<'_, peripherals::SPI1, peripherals::DMA2_CH3, peripherals::DMA2_CH2>,
    cs: &mut Output<'_, peripherals::PB6>,
    buffer: &mut [u8],
) -> Result<u16, ()> {
    let rx_size = tcp_rx_available(spi, cs).await?;
    if rx_size == 0 {
        return Ok(0);
    }

    // Read RX read pointer
    let ptr_high = w5500_read_socket_register(spi, cs, TCP_SOCKET, REG_SN_RX_RD0).await?;
    let ptr_low = w5500_read_socket_register(spi, cs, TCP_SOCKET, REG_SN_RX_RD0 + 1).await?;
    let rx_ptr = u16::from_be_bytes([ptr_high, ptr_low]);

    // Read data (no header for TCP, unlike UDP)
    let bytes_to_read = rx_size.min(buffer.len() as u16);
    w5500_read_socket_rx_buffer(spi, cs, TCP_SOCKET, rx_ptr, &mut buffer[..bytes_to_read as usize]).await?;

    // Update RX read pointer
    let new_ptr = rx_ptr.wrapping_add(bytes_to_read);
    let new_ptr_bytes = new_ptr.to_be_bytes();
    w5500_write_socket_register(spi, cs, TCP_SOCKET, REG_SN_RX_RD0, new_ptr_bytes[0]).await?;
    w5500_write_socket_register(spi, cs, TCP_SOCKET, REG_SN_RX_RD0 + 1, new_ptr_bytes[1]).await?;

    // Send RECV command
    w5500_write_socket_register(spi, cs, TCP_SOCKET, REG_SN_CR, SOCK_CMD_RECV).await?;

    Ok(bytes_to_read)
}

/// Send TCP data
///
/// Returns number of bytes sent
pub async fn tcp_send(
    spi: &mut Spi<'_, peripherals::SPI1, peripherals::DMA2_CH3, peripherals::DMA2_CH2>,
    cs: &mut Output<'_, peripherals::PB6>,
    data: &[u8],
) -> Result<u16, ()> {
    if data.is_empty() {
        return Ok(0);
    }

    // Read TX write pointer
    let ptr_high = w5500_read_socket_register(spi, cs, TCP_SOCKET, REG_SN_TX_WR0).await?;
    let ptr_low = w5500_read_socket_register(spi, cs, TCP_SOCKET, REG_SN_TX_WR0 + 1).await?;
    let tx_ptr = u16::from_be_bytes([ptr_high, ptr_low]);

    // Write data to TX buffer
    w5500_write_socket_tx_buffer(spi, cs, TCP_SOCKET, tx_ptr, data).await?;

    // Update TX write pointer
    let new_ptr = tx_ptr.wrapping_add(data.len() as u16);
    let new_ptr_bytes = new_ptr.to_be_bytes();
    w5500_write_socket_register(spi, cs, TCP_SOCKET, REG_SN_TX_WR0, new_ptr_bytes[0]).await?;
    w5500_write_socket_register(spi, cs, TCP_SOCKET, REG_SN_TX_WR0 + 1, new_ptr_bytes[1]).await?;

    // Send SEND command
    w5500_write_socket_register(spi, cs, TCP_SOCKET, REG_SN_CR, SOCK_CMD_SEND).await?;

    // Wait for send to complete
    for _ in 0..100 {
        let ir = w5500_read_socket_register(spi, cs, TCP_SOCKET, REG_SN_IR).await?;
        if ir & SOCK_IR_SEND_OK != 0 {
            // Clear the interrupt flag
            w5500_write_socket_register(spi, cs, TCP_SOCKET, REG_SN_IR, SOCK_IR_SEND_OK).await?;
            return Ok(data.len() as u16);
        }
        Timer::after_millis(1).await;
    }

    warn!("TCP send timeout");
    Ok(data.len() as u16) // Return anyway, might have sent
}

/// Close TCP connection and return to listening state
pub async fn tcp_disconnect(
    spi: &mut Spi<'_, peripherals::SPI1, peripherals::DMA2_CH3, peripherals::DMA2_CH2>,
    cs: &mut Output<'_, peripherals::PB6>,
    port: u16,
) -> Result<(), ()> {
    info!("TCP client disconnected, returning to listen state");

    // Send DISCON command (graceful close)
    w5500_write_socket_register(spi, cs, TCP_SOCKET, REG_SN_CR, SOCK_CMD_DISCON).await?;
    Timer::after_millis(10).await;

    // Re-open the socket in listen mode
    open_tcp_server(spi, cs, port).await
}

// ============================================================================
// W5500 UDP Socket Functions (Socket 1 - BACnet/IP)
// ============================================================================

/// Socket 1 is reserved for UDP (BACnet)
const UDP_SOCKET: u8 = 1;

/// Open UDP socket on specified port
///
/// Uses Socket 1 for BACnet/IP (UDP port 47808 = 0xBAC0)
pub async fn open_udp_socket(
    spi: &mut Spi<'_, peripherals::SPI1, peripherals::DMA2_CH3, peripherals::DMA2_CH2>,
    cs: &mut Output<'_, peripherals::PB6>,
    port: u16,
) -> Result<(), ()> {
    info!("Opening UDP socket on port {}...", port);

    // Close socket first to ensure clean state
    w5500_write_socket_register(spi, cs, UDP_SOCKET, REG_SN_CR, SOCK_CMD_CLOSE).await?;
    Timer::after_millis(10).await;

    // Wait for socket to close
    for _ in 0..20 {
        let status = w5500_read_socket_register(spi, cs, UDP_SOCKET, REG_SN_SR).await?;
        if status == SOCK_STATUS_CLOSED {
            break;
        }
        Timer::after_millis(10).await;
    }

    // Set socket mode to UDP
    w5500_write_socket_register(spi, cs, UDP_SOCKET, REG_SN_MR, SOCK_MODE_UDP).await?;

    // Set source port
    let port_bytes = port.to_be_bytes();
    w5500_write_socket_register(spi, cs, UDP_SOCKET, REG_SN_PORT0, port_bytes[0]).await?;
    w5500_write_socket_register(spi, cs, UDP_SOCKET, REG_SN_PORT0 + 1, port_bytes[1]).await?;

    // Send OPEN command
    w5500_write_socket_register(spi, cs, UDP_SOCKET, REG_SN_CR, SOCK_CMD_OPEN).await?;
    Timer::after_millis(10).await;

    // Verify socket opened
    let status = w5500_read_socket_register(spi, cs, UDP_SOCKET, REG_SN_SR).await?;
    if status == SOCK_STATUS_UDP {
        info!("UDP socket opened on port {}", port);
        Ok(())
    } else {
        warn!("UDP socket failed to open, status: 0x{:02X}", status);
        Err(())
    }
}

/// Get UDP socket status (for debugging)
pub async fn udp_get_status(
    spi: &mut Spi<'_, peripherals::SPI1, peripherals::DMA2_CH3, peripherals::DMA2_CH2>,
    cs: &mut Output<'_, peripherals::PB6>,
) -> Result<u8, ()> {
    w5500_read_socket_register(spi, cs, UDP_SOCKET, REG_SN_SR).await
}

/// Check if UDP data is available
pub async fn udp_rx_available(
    spi: &mut Spi<'_, peripherals::SPI1, peripherals::DMA2_CH3, peripherals::DMA2_CH2>,
    cs: &mut Output<'_, peripherals::PB6>,
) -> Result<u16, ()> {
    let high = w5500_read_socket_register(spi, cs, UDP_SOCKET, REG_SN_RX_RSR0).await?;
    let low = w5500_read_socket_register(spi, cs, UDP_SOCKET, REG_SN_RX_RSR0 + 1).await?;
    Ok(u16::from_be_bytes([high, low]))
}

/// Receive UDP datagram
///
/// Returns (bytes_read, source_ip, source_port)
pub async fn udp_recv(
    spi: &mut Spi<'_, peripherals::SPI1, peripherals::DMA2_CH3, peripherals::DMA2_CH2>,
    cs: &mut Output<'_, peripherals::PB6>,
    buffer: &mut [u8],
) -> Result<(u16, [u8; 4], u16), ()> {
    let rx_size = udp_rx_available(spi, cs).await?;
    if rx_size == 0 {
        return Ok((0, [0; 4], 0));
    }

    // Read RX read pointer
    let ptr_high = w5500_read_socket_register(spi, cs, UDP_SOCKET, REG_SN_RX_RD0).await?;
    let ptr_low = w5500_read_socket_register(spi, cs, UDP_SOCKET, REG_SN_RX_RD0 + 1).await?;
    let rx_ptr = u16::from_be_bytes([ptr_high, ptr_low]);

    // W5500 UDP header: [IP(4)] [Port(2)] [Length(2)] [Data...]
    let mut header = [0u8; 8];
    w5500_read_socket_rx_buffer(spi, cs, UDP_SOCKET, rx_ptr, &mut header).await?;

    let src_ip = [header[0], header[1], header[2], header[3]];
    let src_port = u16::from_be_bytes([header[4], header[5]]);
    let data_len = u16::from_be_bytes([header[6], header[7]]);

    // Read actual data
    let bytes_to_read = data_len.min(buffer.len() as u16);
    w5500_read_socket_rx_buffer(spi, cs, UDP_SOCKET, rx_ptr.wrapping_add(8), &mut buffer[..bytes_to_read as usize]).await?;

    // Update RX read pointer
    let new_ptr = rx_ptr.wrapping_add(8 + data_len);
    let new_ptr_bytes = new_ptr.to_be_bytes();
    w5500_write_socket_register(spi, cs, UDP_SOCKET, REG_SN_RX_RD0, new_ptr_bytes[0]).await?;
    w5500_write_socket_register(spi, cs, UDP_SOCKET, REG_SN_RX_RD0 + 1, new_ptr_bytes[1]).await?;

    // Send RECV command
    w5500_write_socket_register(spi, cs, UDP_SOCKET, REG_SN_CR, SOCK_CMD_RECV).await?;

    Ok((bytes_to_read, src_ip, src_port))
}

/// Send UDP datagram to specified destination
pub async fn udp_send(
    spi: &mut Spi<'_, peripherals::SPI1, peripherals::DMA2_CH3, peripherals::DMA2_CH2>,
    cs: &mut Output<'_, peripherals::PB6>,
    dest_ip: [u8; 4],
    dest_port: u16,
    data: &[u8],
) -> Result<u16, ()> {
    if data.is_empty() {
        return Ok(0);
    }

    // Set destination IP
    w5500_write_socket_register(spi, cs, UDP_SOCKET, REG_SN_DIPR0, dest_ip[0]).await?;
    w5500_write_socket_register(spi, cs, UDP_SOCKET, REG_SN_DIPR0 + 1, dest_ip[1]).await?;
    w5500_write_socket_register(spi, cs, UDP_SOCKET, REG_SN_DIPR0 + 2, dest_ip[2]).await?;
    w5500_write_socket_register(spi, cs, UDP_SOCKET, REG_SN_DIPR0 + 3, dest_ip[3]).await?;

    // Set destination port
    let port_bytes = dest_port.to_be_bytes();
    w5500_write_socket_register(spi, cs, UDP_SOCKET, REG_SN_DPORT0, port_bytes[0]).await?;
    w5500_write_socket_register(spi, cs, UDP_SOCKET, REG_SN_DPORT0 + 1, port_bytes[1]).await?;

    // Read TX write pointer
    let ptr_high = w5500_read_socket_register(spi, cs, UDP_SOCKET, REG_SN_TX_WR0).await?;
    let ptr_low = w5500_read_socket_register(spi, cs, UDP_SOCKET, REG_SN_TX_WR0 + 1).await?;
    let tx_ptr = u16::from_be_bytes([ptr_high, ptr_low]);

    // Write data to TX buffer
    w5500_write_socket_tx_buffer(spi, cs, UDP_SOCKET, tx_ptr, data).await?;

    // Update TX write pointer
    let new_ptr = tx_ptr.wrapping_add(data.len() as u16);
    let new_ptr_bytes = new_ptr.to_be_bytes();
    w5500_write_socket_register(spi, cs, UDP_SOCKET, REG_SN_TX_WR0, new_ptr_bytes[0]).await?;
    w5500_write_socket_register(spi, cs, UDP_SOCKET, REG_SN_TX_WR0 + 1, new_ptr_bytes[1]).await?;

    // Send SEND command
    w5500_write_socket_register(spi, cs, UDP_SOCKET, REG_SN_CR, SOCK_CMD_SEND).await?;
    Timer::after_millis(10).await;

    Ok(data.len() as u16)
}

/// Send UDP broadcast to all devices on subnet
pub async fn udp_send_broadcast(
    spi: &mut Spi<'_, peripherals::SPI1, peripherals::DMA2_CH3, peripherals::DMA2_CH2>,
    cs: &mut Output<'_, peripherals::PB6>,
    dest_port: u16,
    data: &[u8],
) -> Result<u16, ()> {
    // Broadcast address: 255.255.255.255
    udp_send(spi, cs, [255, 255, 255, 255], dest_port, data).await
}

// ============================================================================
// W5500 Register Definitions
// ============================================================================

/// W5500 Control Phase bits
const CONTROL_PHASE_READ: u8 = 0x00;
const CONTROL_PHASE_WRITE: u8 = 0x04;

/// W5500 Block Select Bits (BSB)
const BSB_COMMON_REG: u8 = 0x00;

/// W5500 Common Registers
const REG_VERSIONR: u16 = 0x0039;
const REG_SHAR0: u16 = 0x0009;     // MAC address (6 bytes)
const REG_SIPR0: u16 = 0x000F;     // IP address (4 bytes)
const REG_SUBR0: u16 = 0x0005;     // Subnet mask (4 bytes)
const REG_GAR0: u16 = 0x0001;      // Gateway address (4 bytes)

/// W5500 Socket Registers (offset from socket base)
const REG_SN_MR: u16 = 0x0000;      // Mode
const REG_SN_CR: u16 = 0x0001;      // Command
const REG_SN_IR: u16 = 0x0002;      // Interrupt
const REG_SN_SR: u16 = 0x0003;      // Status
const REG_SN_PORT0: u16 = 0x0004;   // Source port (2 bytes)
const REG_SN_DIPR0: u16 = 0x000C;   // Destination IP (4 bytes)
const REG_SN_DPORT0: u16 = 0x0010;  // Destination port (2 bytes)
const REG_SN_TX_WR0: u16 = 0x0024;  // TX write pointer (2 bytes)
const REG_SN_RX_RSR0: u16 = 0x0026; // RX received size (2 bytes)
const REG_SN_RX_RD0: u16 = 0x0028;  // RX read pointer (2 bytes)

/// Socket modes
const SOCK_MODE_TCP: u8 = 0x01;
const SOCK_MODE_UDP: u8 = 0x02;

/// Socket commands
const SOCK_CMD_OPEN: u8 = 0x01;
const SOCK_CMD_LISTEN: u8 = 0x02;
const SOCK_CMD_DISCON: u8 = 0x08;
const SOCK_CMD_CLOSE: u8 = 0x10;
const SOCK_CMD_SEND: u8 = 0x20;
const SOCK_CMD_RECV: u8 = 0x40;

/// Socket interrupt flags
const SOCK_IR_SEND_OK: u8 = 0x10;

/// Socket status values
const SOCK_STATUS_CLOSED: u8 = 0x00;
const SOCK_STATUS_INIT: u8 = 0x13;
const SOCK_STATUS_LISTEN: u8 = 0x14;
/// TCP connection established
pub const SOCK_STATUS_ESTABLISHED: u8 = 0x17;
/// TCP close wait (peer closed)
pub const SOCK_STATUS_CLOSE_WAIT: u8 = 0x1C;
const SOCK_STATUS_UDP: u8 = 0x22;

// ============================================================================
// W5500 Low-Level SPI Functions
// ============================================================================

/// Read single byte from W5500 common register
async fn w5500_read_register(
    spi: &mut Spi<'_, peripherals::SPI1, peripherals::DMA2_CH3, peripherals::DMA2_CH2>,
    cs: &mut Output<'_, peripherals::PB6>,
    address: u16,
) -> Result<u8, ()> {
    let control = (BSB_COMMON_REG << 3) | CONTROL_PHASE_READ;
    let tx_buf = [(address >> 8) as u8, (address & 0xFF) as u8, control, 0x00];
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

/// Write multiple bytes to W5500 common register
async fn w5500_write_register(
    spi: &mut Spi<'_, peripherals::SPI1, peripherals::DMA2_CH3, peripherals::DMA2_CH2>,
    cs: &mut Output<'_, peripherals::PB6>,
    address: u16,
    data: &[u8],
) -> Result<(), ()> {
    let control = (BSB_COMMON_REG << 3) | CONTROL_PHASE_WRITE;
    let mut tx_buf = [0u8; 32];
    let len = 3 + data.len();

    tx_buf[0] = (address >> 8) as u8;
    tx_buf[1] = (address & 0xFF) as u8;
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

/// Read single byte from W5500 socket register
async fn w5500_read_socket_register(
    spi: &mut Spi<'_, peripherals::SPI1, peripherals::DMA2_CH3, peripherals::DMA2_CH2>,
    cs: &mut Output<'_, peripherals::PB6>,
    socket: u8,
    address: u16,
) -> Result<u8, ()> {
    let bsb = ((socket * 4) + 1) << 3; // Socket n register block
    let control = bsb | CONTROL_PHASE_READ;
    let tx_buf = [(address >> 8) as u8, (address & 0xFF) as u8, control, 0x00];
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
    socket: u8,
    address: u16,
    data: u8,
) -> Result<(), ()> {
    let bsb = ((socket * 4) + 1) << 3;
    let control = bsb | CONTROL_PHASE_WRITE;
    let tx_buf = [(address >> 8) as u8, (address & 0xFF) as u8, control, data];

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

/// Read from W5500 socket RX buffer
async fn w5500_read_socket_rx_buffer(
    spi: &mut Spi<'_, peripherals::SPI1, peripherals::DMA2_CH3, peripherals::DMA2_CH2>,
    cs: &mut Output<'_, peripherals::PB6>,
    socket: u8,
    address: u16,
    buffer: &mut [u8],
) -> Result<(), ()> {
    let bsb = ((socket * 4) + 3) << 3; // Socket n RX buffer
    let control = bsb | CONTROL_PHASE_READ;

    let mut tx_buf = [0u8; 512];
    let len = 3 + buffer.len();
    tx_buf[0] = (address >> 8) as u8;
    tx_buf[1] = (address & 0xFF) as u8;
    tx_buf[2] = control;

    let mut rx_buf = [0u8; 512];

    cs.set_low();
    Timer::after_micros(1).await;
    let result = spi.transfer(&mut rx_buf[..len], &tx_buf[..len]).await;
    Timer::after_micros(1).await;
    cs.set_high();

    match result {
        Ok(_) => {
            buffer.copy_from_slice(&rx_buf[3..len]);
            Ok(())
        }
        Err(_) => Err(()),
    }
}

/// Write to W5500 socket TX buffer
async fn w5500_write_socket_tx_buffer(
    spi: &mut Spi<'_, peripherals::SPI1, peripherals::DMA2_CH3, peripherals::DMA2_CH2>,
    cs: &mut Output<'_, peripherals::PB6>,
    socket: u8,
    address: u16,
    data: &[u8],
) -> Result<(), ()> {
    let bsb = ((socket * 4) + 2) << 3; // Socket n TX buffer
    let control = bsb | CONTROL_PHASE_WRITE;

    let mut tx_buf = [0u8; 512];
    let len = 3 + data.len();
    tx_buf[0] = (address >> 8) as u8;
    tx_buf[1] = (address & 0xFF) as u8;
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
// Sensor Data
// ============================================================================

/// Sensor data structure
#[derive(Clone, Copy)]
pub struct SensorData {
    pub temperature: f32,
    pub humidity: f32,
    pub status: u16,
    pub uptime: u32,
}

impl Default for SensorData {
    fn default() -> Self {
        SensorData {
            temperature: 25.5,
            humidity: 60.0,
            status: status::OK,
            uptime: 0,
        }
    }
}

// ============================================================================
// SHT3x Sensor Functions
// ============================================================================

/// Initialize SHT31-D sensor on I2C1
pub async fn init_sht3x() -> I2c<'static, peripherals::I2C1, peripherals::DMA1_CH6, peripherals::DMA1_CH0> {
    info!("Initializing SHT31-D sensor on I2C1");

    let p = unsafe { embassy_stm32::Peripherals::steal() };

    let mut i2c_config = I2cConfig::default();
    i2c_config.sda_pullup = false;
    i2c_config.scl_pullup = false;

    info!("Configuring I2C1: SCL=PB8, SDA=PB9");
    let mut i2c = I2c::new(
        p.I2C1,
        p.PB8,
        p.PB9,
        I2c1Irqs,
        p.DMA1_CH6,
        p.DMA1_CH0,
        Hertz(100_000),
        i2c_config,
    );

    Timer::after_millis(100).await;

    // Soft reset
    let reset_cmd = [0x30, 0xA2];
    match i2c.write(0x44, &reset_cmd).await {
        Ok(_) => {
            info!("SHT31-D soft reset sent");
            Timer::after_millis(20).await;
        }
        Err(_) => {
            warn!("SHT31-D communication error");
        }
    }

    info!("SHT31-D initialized");
    i2c
}

/// Read temperature and humidity from SHT31-D
pub async fn read_sht3x(
    i2c: &mut I2c<'_, peripherals::I2C1, peripherals::DMA1_CH6, peripherals::DMA1_CH0>
) -> Result<(f32, f32), ()> {
    let cmd = [0x24, 0x00]; // High repeatability

    if i2c.write(0x44, &cmd).await.is_err() {
        return Err(());
    }

    Timer::after_millis(20).await;

    let mut data = [0u8; 6];
    match i2c.read(0x44, &mut data).await {
        Ok(_) => {
            let temp_raw = u16::from_be_bytes([data[0], data[1]]);
            let hum_raw = u16::from_be_bytes([data[3], data[4]]);

            let temp_c = -45.0 + 175.0 * (temp_raw as f32 / 65535.0);
            let hum_pct = 100.0 * (hum_raw as f32 / 65535.0);

            Ok((temp_c, hum_pct))
        }
        Err(_) => Err(()),
    }
}

// ============================================================================
// OLED Display Functions
// ============================================================================

use embassy_stm32::dma::NoDma;

pub type OledDisplay = Ssd1306<
    I2CInterface<I2c<'static, peripherals::I2C1, NoDma, NoDma>>,
    DisplaySize128x64,
    BufferedGraphicsMode<DisplaySize128x64>
>;

/// Initialize SSD1306 OLED display
pub async fn init_oled() -> OledDisplay {
    info!("Initializing OLED display");

    let p = unsafe { embassy_stm32::Peripherals::steal() };

    let mut i2c_config = I2cConfig::default();
    i2c_config.sda_pullup = false;
    i2c_config.scl_pullup = false;

    let i2c = I2c::new(
        p.I2C1,
        p.PB8,
        p.PB9,
        I2c1Irqs,
        NoDma,
        NoDma,
        Hertz(100_000),
        i2c_config,
    );

    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();

    match display.init() {
        Ok(_) => {
            display.clear(BinaryColor::Off).ok();
            display.flush().ok();
            info!("OLED initialized");
        }
        Err(_) => {
            warn!("OLED init failed");
        }
    }

    display
}

/// Display startup banner for unified gateway
pub fn display_startup(display: &mut OledDisplay, board_id: &str, ip: [u8; 4], _port: u16) {
    let text_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);

    if display.clear(BinaryColor::Off).is_err() {
        return;
    }

    let mut text: String<32> = String::new();
    let _ = write!(text, "{}", board_id);
    let _ = Text::new(&text, Point::new(0, 10), text_style).draw(display);

    text.clear();
    let _ = write!(text, "{}.{}.{}.{}", ip[0], ip[1], ip[2], ip[3]);
    let _ = Text::new(&text, Point::new(0, 22), text_style).draw(display);

    // Show both protocols
    let _ = Text::new("Modbus:502 BACnet:47808", Point::new(0, 34), text_style).draw(display);
    let _ = Text::new("Initializing...", Point::new(0, 46), text_style).draw(display);

    let _ = display.flush();
}

/// Update display with sensor data (unified gateway layout)
///
/// Display layout (128x64 pixels, FONT_6X10):
/// ```text
/// Line 1: Board ID
/// Line 2: IP address
/// Line 3: Temperature
/// Line 4: Humidity
/// Line 5: Protocol ports
/// ```
pub fn update_display(
    display: &mut OledDisplay,
    sensor_data: &SensorData,
    board_id: &str,
    ip: [u8; 4],
) {
    let text_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);

    if display.clear(BinaryColor::Off).is_err() {
        return;
    }

    let mut text: String<32> = String::new();
    let _ = write!(text, "{}", board_id);
    let _ = Text::new(&text, Point::new(0, 10), text_style).draw(display);

    text.clear();
    let _ = write!(text, "{}.{}.{}.{}", ip[0], ip[1], ip[2], ip[3]);
    let _ = Text::new(&text, Point::new(0, 22), text_style).draw(display);

    text.clear();
    let _ = write!(text, "T:{:.1}C  H:{:.1}%", sensor_data.temperature, sensor_data.humidity);
    let _ = Text::new(&text, Point::new(0, 34), text_style).draw(display);

    // Show status indicator based on sensor_data.status
    text.clear();
    let status_str = match sensor_data.status {
        status::OK => "OK",
        status::SENSOR_ERROR => "SENSOR ERR",
        status::NETWORK_ERROR => "NET ERR",
        _ => "?",
    };
    let _ = write!(text, "Status: {}", status_str);
    let _ = Text::new(&text, Point::new(0, 46), text_style).draw(display);

    // Show both protocol ports
    let _ = Text::new("TCP:502 UDP:47808", Point::new(0, 58), text_style).draw(display);

    let _ = display.flush();
}
