//! Unified IIoT Gateway - Modbus TCP + BACnet/IP
//!
//! This firmware runs BOTH industrial protocols on a single device:
//! - Modbus TCP on port 502 (Socket 0)
//! - BACnet/IP on port 47808 (Socket 1)
//!
//! The gateway exposes SHT3x sensor data (temperature + humidity) via both
//! protocols, allowing integration with:
//! - SCADA systems (Modbus TCP)
//! - Building Management Systems (BACnet/IP)
//! - Unified monitoring (InfluxDB + Grafana)
//!
//! # Hardware
//! - MCU: NUCLEO-F446RE (STM32F446RE, Cortex-M4F)
//! - Ethernet: W5500 SPI module
//! - Sensor: SHT3x I2C (temperature + humidity)
//! - Display: SSD1306 OLED I2C
//!
//! # Main Loop
//! 1. Check TCP socket for Modbus requests
//! 2. Check UDP socket for BACnet packets
//! 3. Read sensor every 2 seconds
//! 4. Update display every 2 seconds

#![no_std]
#![no_main]

// These handle panics and debug output
use defmt_rtt as _;
use panic_probe as _;

// Our modules
mod bacnet;
mod common;
mod modbus;

use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_time::Timer;

// ============================================================================
// Configuration
// ============================================================================

/// Board identifier (shown on display and in logs)
const BOARD_ID: &str = "UNIFIED_GW";

/// Static IP address for this device
/// This is modbus_1's IP - we're upgrading it to run both protocols
const IP_ADDRESS: [u8; 4] = [10, 10, 10, 100];

/// MAC address (locally administered)
/// First byte has bit 1 set (0x02) = locally administered
const MAC_ADDRESS: [u8; 6] = [0x02, 0x00, 0x00, 0x00, 0x00, 0x01];

/// Modbus TCP port (standard)
const MODBUS_PORT: u16 = 502;

// ============================================================================
// Main Entry Point
// ============================================================================

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // ========================================================================
    // Step 1: Print startup banner
    // ========================================================================
    info!("========================================");
    info!("Unified IIoT Gateway - {}", BOARD_ID);
    info!("IP: {}.{}.{}.{}", IP_ADDRESS[0], IP_ADDRESS[1], IP_ADDRESS[2], IP_ADDRESS[3]);
    info!("Modbus TCP: port {}", MODBUS_PORT);
    info!("BACnet/IP:  port {} (Device {})", bacnet::BACNET_PORT, bacnet::DEVICE_INSTANCE);
    info!("========================================");

    // ========================================================================
    // Step 2: Initialize Hardware
    // ========================================================================

    // Initialize W5500 Ethernet with network configuration
    // This returns the SPI handle and chip-select pin
    let (mut spi, mut cs) = common::init_hardware(BOARD_ID, IP_ADDRESS, MAC_ADDRESS).await;
    info!("W5500 Ethernet initialized");

    // Initialize the SHT3x temperature/humidity sensor
    let mut sht3x = common::init_sht3x().await;
    info!("SHT3x sensor initialized");

    // Initialize the OLED display
    let mut oled = common::init_oled().await;
    info!("OLED display initialized");

    // Show startup message on display
    common::display_startup(&mut oled, BOARD_ID, IP_ADDRESS, MODBUS_PORT);

    // ========================================================================
    // Step 3: Open Sockets for Both Protocols
    // ========================================================================

    // Socket 0: TCP server for Modbus (port 502)
    match common::open_tcp_server(&mut spi, &mut cs, MODBUS_PORT).await {
        Ok(_) => info!("Modbus TCP listening on port {}", MODBUS_PORT),
        Err(_) => {
            warn!("Failed to open Modbus TCP socket!");
            // Continue anyway - maybe it will work later
        }
    }

    // Socket 1: UDP for BACnet/IP (port 47808)
    match common::open_udp_socket(&mut spi, &mut cs, bacnet::BACNET_PORT).await {
        Ok(_) => info!("BACnet/IP UDP socket on port {}", bacnet::BACNET_PORT),
        Err(_) => {
            warn!("Failed to open BACnet UDP socket!");
            // Continue anyway - maybe it will work later
        }
    }

    // ========================================================================
    // Step 4: Take Initial Sensor Reading
    // ========================================================================

    let mut sensor_data = common::SensorData::default();

    info!("Taking initial sensor reading...");
    match common::read_sht3x(&mut sht3x).await {
        Ok((temp, hum)) => {
            sensor_data.temperature = temp;
            sensor_data.humidity = hum;
            // Note: defmt doesn't support floats directly, so we multiply by 10
            info!("Temperature: {} (x0.1 C)", (temp * 10.0) as i32);
            info!("Humidity: {} (x0.1 %)", (hum * 10.0) as i32);
        }
        Err(_) => {
            warn!("Failed to read sensor - using default values");
        }
    }

    info!("=== Unified Gateway Ready! ===");
    info!("  Modbus TCP: {}.{}.{}.{}:{}",
          IP_ADDRESS[0], IP_ADDRESS[1], IP_ADDRESS[2], IP_ADDRESS[3], MODBUS_PORT);
    info!("  BACnet/IP:  {}.{}.{}.{}:{}",
          IP_ADDRESS[0], IP_ADDRESS[1], IP_ADDRESS[2], IP_ADDRESS[3], bacnet::BACNET_PORT);

    // ========================================================================
    // Step 5: Main Loop
    // ========================================================================
    //
    // The main loop handles four tasks:
    // A. Check for Modbus TCP connections and requests
    // B. Check for BACnet/IP packets
    // C. Read sensor periodically (every 2 seconds)
    // D. Update the display periodically

    let mut loop_counter: u32 = 0;
    let mut tcp_rx_buffer = [0u8; 512];   // Buffer for Modbus TCP packets
    let mut tcp_tx_buffer = [0u8; 512];   // Buffer for Modbus TCP responses
    let mut udp_rx_buffer = [0u8; 512];   // Buffer for BACnet UDP packets
    let mut udp_tx_buffer = [0u8; 512];   // Buffer for BACnet UDP responses

    loop {
        // Wait 100ms between iterations
        // This gives us 10 iterations per second
        Timer::after_millis(100).await;
        loop_counter = loop_counter.wrapping_add(1);

        // ====================================================================
        // Task A: Check for Modbus TCP connections and data
        // ====================================================================

        // Check TCP socket status
        match common::tcp_get_status(&mut spi, &mut cs).await {
            Ok(status) => {
                match status {
                    // Client connected - check for data
                    common::SOCK_STATUS_ESTABLISHED => {
                        match common::tcp_rx_available(&mut spi, &mut cs).await {
                            Ok(bytes_available) if bytes_available > 0 => {
                                info!("Modbus: {} bytes received", bytes_available);

                                // Read the request
                                match common::tcp_recv(&mut spi, &mut cs, &mut tcp_rx_buffer).await {
                                    Ok(bytes_read) if bytes_read > 0 => {
                                        // Convert sensor data to Modbus format
                                        let modbus_data = modbus::ModbusSensorData {
                                            temperature: sensor_data.temperature,
                                            humidity: sensor_data.humidity,
                                            status: sensor_data.status,
                                            uptime: sensor_data.uptime,
                                        };

                                        // Handle the Modbus request
                                        let response = modbus::handle_request(
                                            &tcp_rx_buffer[..bytes_read as usize],
                                            &modbus_data,
                                            &mut tcp_tx_buffer,
                                        );

                                        // Send response
                                        match response {
                                            modbus::ModbusResponse::Response(len) => {
                                                info!("Modbus: Sending response ({} bytes)", len);
                                                match common::tcp_send(&mut spi, &mut cs, &tcp_tx_buffer[..len]).await {
                                                    Ok(_) => info!("Modbus: Response sent"),
                                                    Err(_) => warn!("Modbus: Failed to send response"),
                                                }
                                            }
                                            modbus::ModbusResponse::Exception(len) => {
                                                info!("Modbus: Sending exception ({} bytes)", len);
                                                match common::tcp_send(&mut spi, &mut cs, &tcp_tx_buffer[..len]).await {
                                                    Ok(_) => info!("Modbus: Exception sent"),
                                                    Err(_) => warn!("Modbus: Failed to send exception"),
                                                }
                                            }
                                            modbus::ModbusResponse::Invalid => {
                                                warn!("Modbus: Invalid request, ignoring");
                                            }
                                        }
                                    }
                                    Ok(_) => {
                                        // No data yet
                                    }
                                    Err(_) => {
                                        warn!("Modbus: Failed to receive data");
                                    }
                                }
                            }
                            Ok(_) => {
                                // No data available, client still connected
                            }
                            Err(_) => {
                                warn!("Modbus: Failed to check RX buffer");
                            }
                        }
                    }

                    // Client disconnected - reopen socket
                    common::SOCK_STATUS_CLOSE_WAIT => {
                        info!("Modbus: Client disconnected");
                        match common::tcp_disconnect(&mut spi, &mut cs, MODBUS_PORT).await {
                            Ok(_) => info!("Modbus: Socket reset, listening again"),
                            Err(_) => warn!("Modbus: Failed to reset socket"),
                        }
                    }

                    // Socket is listening, waiting for connection
                    _ => {
                        // Normal - socket is listening for new connections
                    }
                }
            }
            Err(_) => {
                warn!("Modbus: Failed to get socket status");
            }
        }

        // ====================================================================
        // Task B: Check for BACnet/IP packets (UDP)
        // ====================================================================

        // Check if any UDP data arrived
        match common::udp_rx_available(&mut spi, &mut cs).await {
            Ok(bytes_available) if bytes_available > 0 => {
                info!("BACnet: {} bytes received", bytes_available);

                // Read the packet
                match common::udp_recv(&mut spi, &mut cs, &mut udp_rx_buffer).await {
                    Ok((bytes_read, src_ip, src_port)) => {
                        info!(
                            "BACnet: From {}.{}.{}.{}:{}",
                            src_ip[0], src_ip[1], src_ip[2], src_ip[3], src_port
                        );

                        // Create BACnet sensor data from our readings
                        let bacnet_data = bacnet::BacnetSensorData {
                            temperature: sensor_data.temperature,
                            humidity: sensor_data.humidity,
                            uptime: sensor_data.uptime,
                        };

                        // Handle the BACnet packet
                        let response = bacnet::handle_packet(
                            &udp_rx_buffer[..bytes_read as usize],
                            src_ip,
                            src_port,
                            &bacnet_data,
                            &mut udp_tx_buffer,
                        );

                        // Send response if needed
                        match response {
                            bacnet::BacnetResponse::IAm(len) => {
                                info!("BACnet: Sending I-Am broadcast ({} bytes)", len);
                                // Broadcast I-Am to all devices
                                match common::udp_send_broadcast(
                                    &mut spi,
                                    &mut cs,
                                    bacnet::BACNET_PORT,
                                    &udp_tx_buffer[..len],
                                ).await {
                                    Ok(_) => info!("BACnet: I-Am sent"),
                                    Err(_) => warn!("BACnet: Failed to send I-Am"),
                                }
                            }

                            bacnet::BacnetResponse::ReadPropertyAck { length, dest_ip, dest_port } => {
                                info!("BACnet: Sending ReadProperty response ({} bytes)", length);
                                match common::udp_send(
                                    &mut spi,
                                    &mut cs,
                                    dest_ip,
                                    dest_port,
                                    &udp_tx_buffer[..length],
                                ).await {
                                    Ok(_) => info!("BACnet: Response sent"),
                                    Err(_) => warn!("BACnet: Failed to send response"),
                                }
                            }

                            bacnet::BacnetResponse::None => {
                                // Unknown packet type, ignore
                            }
                        }
                    }
                    Err(_) => {
                        warn!("BACnet: Failed to read UDP packet");
                    }
                }
            }
            Ok(_) => {
                // No data available, that's normal
            }
            Err(_) => {
                warn!("BACnet: Failed to check RX buffer");
            }
        }

        // ====================================================================
        // Task C: Read sensor every 2 seconds (20 iterations)
        // ====================================================================

        if loop_counter % 20 == 0 {
            match common::read_sht3x(&mut sht3x).await {
                Ok((temp, hum)) => {
                    sensor_data.temperature = temp;
                    sensor_data.humidity = hum;
                    sensor_data.uptime = sensor_data.uptime.wrapping_add(2);
                    sensor_data.status = common::status::OK;
                }
                Err(_) => {
                    sensor_data.status = common::status::SENSOR_ERROR;
                }
            }
        }

        // ====================================================================
        // Task D: Update display every 2 seconds (20 iterations)
        // ====================================================================

        if loop_counter % 20 == 0 {
            common::update_display(&mut oled, &sensor_data, BOARD_ID, IP_ADDRESS);
        }
    }
}
