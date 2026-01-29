#![no_std]
#![no_main]

mod common;

use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_stm32 as _;  // Import to register time driver
use heapless;
use {defmt_rtt as _, panic_probe as _};

// Board 2 Configuration
const BOARD_ID: &str = "MODBUS_2";
const IP_ADDRESS: [u8; 4] = [10, 10, 10, 200];
const MAC_ADDRESS: [u8; 6] = [0x02, 0x00, 0x00, 0x00, 0x00, 0x20]; // Locally administered MAC
const MODBUS_PORT: u16 = 502;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("========================================");
    info!("Week 9: Modbus TCP Slave - {}", BOARD_ID);
    info!("========================================");

    // Initialize hardware (W5500 with network config)
    let (mut spi, mut cs) = common::init_hardware(BOARD_ID, IP_ADDRESS, MAC_ADDRESS).await;

    // Initialize SHT3x sensor
    let mut sht3x = common::init_sht3x().await;

    // Initialize OLED display
    let mut oled = common::init_oled().await;

    // Display startup banner
    common::display_startup(&mut oled, BOARD_ID, IP_ADDRESS);

    info!("=== Board ready - Network configured ===");

    // Create sensor data structure (will be updated with real readings)
    let mut sensor_data = common::SensorData::default();

    // Take initial sensor reading
    info!("Taking initial sensor reading...");
    match common::read_sht3x(&mut sht3x).await {
        Ok((temp, hum)) => {
            sensor_data.temperature = temp;
            sensor_data.humidity = hum;
            info!("Initial readings:");
            info!("  Temperature: {} C (x10)", (temp * 10.0) as i32);
            info!("  Humidity: {} % (x10)", (hum * 10.0) as i32);
        }
        Err(_) => {
            warn!("Failed to read sensor - using default values");
        }
    }

    // Monitor socket status and incoming data
    let mut loop_count = 0u32;
    let mut is_connected = false;

    loop {
        embassy_time::Timer::after_millis(500).await;

        // Update uptime every iteration (increment by 0.5 seconds)
        sensor_data.uptime = sensor_data.uptime.wrapping_add(1);
        loop_count = loop_count.wrapping_add(1);

        // Read sensor every 2 seconds (every 4 iterations of 500ms)
        if loop_count % 4 == 0 {
            match common::read_sht3x(&mut sht3x).await {
                Ok((temp, hum)) => {
                    sensor_data.temperature = temp;
                    sensor_data.humidity = hum;
                }
                Err(_) => {
                    // Sensor read failed - keep previous values
                }
            }
        }

        // Update OLED display every 2 seconds (every 4 iterations of 500ms)
        if loop_count % 4 == 0 {
            common::update_display(&mut oled, &sensor_data, BOARD_ID, IP_ADDRESS, is_connected);
        }

        // Check socket status
        let status = match common::check_socket_status(&mut spi, &mut cs).await {
            Ok(s) => s,
            Err(_) => {
                warn!("Failed to read socket status");
                continue;
            }
        };

        // Handle socket state transitions
        match status {
            0x00 => {  // CLOSED - need to reopen and listen
                warn!("Socket CLOSED - reopening...");
                is_connected = false;
                if let Err(_) = common::reopen_socket(&mut spi, &mut cs).await {
                    warn!("Failed to reopen socket");
                }
                continue;
            }
            0x1C => {  // CLOSE_WAIT - client closed, we need to close too
                info!("Socket CLOSE_WAIT - closing connection");
                is_connected = false;
                if let Err(_) = common::close_socket(&mut spi, &mut cs).await {
                    warn!("Failed to close socket");
                }
                continue;
            }
            0x13 => {  // INIT - need to send LISTEN command
                info!("Socket INIT - sending LISTEN");
                is_connected = false;
                if let Err(_) = common::listen_socket(&mut spi, &mut cs).await {
                    warn!("Failed to send LISTEN command");
                }
                continue;
            }
            0x14 => {  // LISTEN - waiting for connection (normal state)
                is_connected = false;
                // Nothing to do, just wait
            }
            0x17 => {  // ESTABLISHED - connection active
                is_connected = true;
                // Handle data below
            }
            _ => {
                // Unknown or transitional state
            }
        }

        // Check for incoming data when connected
        if status == 0x17 {  // ESTABLISHED
            match common::check_rx_size(&mut spi, &mut cs).await {
                Ok(rx_bytes) if rx_bytes > 0 => {
                    info!("Connection ESTABLISHED - {} bytes available!", rx_bytes);

                    // Read the data into a buffer
                    let mut buffer = [0u8; 260]; // Max Modbus TCP frame
                    match common::read_rx_data(&mut spi, &mut cs, &mut buffer).await {
                        Ok(bytes_read) => {
                            info!("Read {} bytes from RX buffer", bytes_read);

                            let data = &buffer[..bytes_read as usize];
                            info!("Received data (hex): {:02X}", data);

                            // Try to parse as Modbus TCP
                            if bytes_read >= 7 {
                                match common::MbapHeader::from_bytes(data) {
                                    Ok(mbap) => {
                                        info!("MBAP Header parsed:");
                                        info!("  Transaction ID: 0x{:04X}", mbap.transaction_id);
                                        info!("  Protocol ID: 0x{:04X}", mbap.protocol_id);
                                        info!("  Length: {}", mbap.length);
                                        info!("  Unit ID: 0x{:02X}", mbap.unit_id);

                                        // Parse PDU (after MBAP header)
                                        if bytes_read >= 12 {  // MBAP (7) + FC (1) + Addr (2) + Count (2)
                                            let pdu = &data[7..];
                                            match common::parse_modbus_request(pdu) {
                                                Ok((fc, addr, count)) => {
                                                    info!("Modbus Request:");
                                                    info!("  Function Code: 0x{:02X}", fc);
                                                    info!("  Start Address: {}", addr);
                                                    info!("  Register Count: {}", count);

                                                    // Build Modbus response
                                                    let mut response = [0u8; 260];
                                                    let mut pos = 0;

                                                    // Write MBAP header (copy from request)
                                                    if let Ok(_) = mbap.to_bytes(&mut response[pos..pos+7]) {
                                                        pos += 7;

                                                        // Update length field for response
                                                        // Length = unit_id (1) + fc (1) + byte_count (1) + data (count * 2)
                                                        let response_length = 1 + 1 + 1 + (count * 2);
                                                        response[4..6].copy_from_slice(&response_length.to_be_bytes());

                                                        // Write PDU header
                                                        response[pos] = fc;  // Function code
                                                        pos += 1;
                                                        response[pos] = (count * 2) as u8;  // Byte count
                                                        pos += 1;

                                                        // Use register handler to fill data from sensor readings
                                                        match common::handle_read_registers(
                                                            addr,
                                                            count,
                                                            &sensor_data,
                                                            &mut response[pos..]
                                                        ) {
                                                            Ok(data_len) => {
                                                                pos += data_len;
                                                                info!("Sending {} byte response", pos);

                                                                // Send response
                                                                match common::write_tx_data(&mut spi, &mut cs, &response[..pos]).await {
                                                                    Ok(bytes_sent) => {
                                                                        info!("Response sent: {} bytes", bytes_sent);
                                                                    }
                                                                    Err(_) => {
                                                                        info!("Failed to send response");
                                                                    }
                                                                }
                                                            }
                                                            Err(exception_code) => {
                                                                info!("Register read error - exception: 0x{:02X}", exception_code);
                                                                // TODO: Send Modbus exception response
                                                            }
                                                        }
                                                    }
                                                }
                                                Err(exception) => {
                                                    info!("Modbus parse error - exception: 0x{:02X}", exception);
                                                }
                                            }
                                        }
                                    }
                                    Err(_) => {
                                        info!("Failed to parse MBAP header");
                                    }
                                }
                            } else {
                                info!("Frame too small for Modbus TCP (< 7 bytes)");
                            }
                        }
                        Err(_) => {
                            info!("Failed to read RX data");
                        }
                    }
                }
                Ok(_) => {
                    // Connected but no data yet
                }
                Err(_) => {
                    info!("Failed to read RX size");
                }
            }
        }
    }
}
