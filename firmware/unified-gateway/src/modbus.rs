//! Modbus TCP Protocol Implementation
//!
//! This module implements Modbus TCP server functionality.
//! It handles ReadHoldingRegisters (FC03) and ReadInputRegisters (FC04).
//!
//! # Register Map
//!
//! | Address | Description | Type |
//! |---------|-------------|------|
//! | 0-1     | Temperature | f32 (IEEE 754) |
//! | 2-3     | Humidity    | f32 (IEEE 754) |
//! | 4       | Status      | u16 |
//! | 5-6     | Uptime      | u32 |

use defmt::info;

// ============================================================================
// Modbus Function Codes
// ============================================================================

pub mod function_code {
    pub const READ_HOLDING_REGISTERS: u8 = 0x03;
    pub const READ_INPUT_REGISTERS: u8 = 0x04;
}

pub mod exception_code {
    pub const ILLEGAL_FUNCTION: u8 = 0x01;
    pub const ILLEGAL_DATA_ADDRESS: u8 = 0x02;
    pub const ILLEGAL_DATA_VALUE: u8 = 0x03;
}

// ============================================================================
// MBAP Header (Modbus Application Protocol Header)
// ============================================================================

/// MBAP Header for Modbus TCP (7 bytes)
///
/// ```text
/// ┌─────────────────┬─────────────────┬────────┬─────────┐
/// │ Transaction ID  │ Protocol ID     │ Length │ Unit ID │
/// │ (2 bytes)       │ (2 bytes, =0)   │(2 bytes)│(1 byte) │
/// └─────────────────┴─────────────────┴────────┴─────────┘
/// ```
#[derive(Debug, Clone, Copy)]
pub struct MbapHeader {
    pub transaction_id: u16,
    pub protocol_id: u16,    // Always 0 for Modbus
    pub length: u16,         // Bytes following (unit_id + PDU)
    pub unit_id: u8,
}

impl MbapHeader {
    /// Parse MBAP header from 7 bytes
    pub fn from_bytes(data: &[u8]) -> Option<Self> {
        if data.len() < 7 {
            return None;
        }

        Some(MbapHeader {
            transaction_id: u16::from_be_bytes([data[0], data[1]]),
            protocol_id: u16::from_be_bytes([data[2], data[3]]),
            length: u16::from_be_bytes([data[4], data[5]]),
            unit_id: data[6],
        })
    }

    /// Write MBAP header to buffer (7 bytes)
    pub fn to_bytes(&self, buffer: &mut [u8]) -> usize {
        if buffer.len() < 7 {
            return 0;
        }

        buffer[0..2].copy_from_slice(&self.transaction_id.to_be_bytes());
        buffer[2..4].copy_from_slice(&self.protocol_id.to_be_bytes());
        buffer[4..6].copy_from_slice(&self.length.to_be_bytes());
        buffer[6] = self.unit_id;

        7
    }
}

// ============================================================================
// Modbus Request Parsing
// ============================================================================

/// Parsed Modbus request
#[derive(Debug)]
pub struct ModbusRequest {
    pub header: MbapHeader,
    pub function_code: u8,
    pub start_address: u16,
    pub quantity: u16,
}

/// Parse a Modbus TCP request
///
/// Returns None if the packet is invalid or too short
pub fn parse_request(data: &[u8]) -> Option<ModbusRequest> {
    // Minimum: MBAP (7) + FC (1) + Addr (2) + Qty (2) = 12 bytes
    if data.len() < 12 {
        return None;
    }

    let header = MbapHeader::from_bytes(data)?;

    // Check protocol ID (must be 0 for Modbus)
    if header.protocol_id != 0 {
        return None;
    }

    let function_code = data[7];
    let start_address = u16::from_be_bytes([data[8], data[9]]);
    let quantity = u16::from_be_bytes([data[10], data[11]]);

    Some(ModbusRequest {
        header,
        function_code,
        start_address,
        quantity,
    })
}

// ============================================================================
// Modbus Response Building
// ============================================================================

/// Sensor data for Modbus register mapping
#[derive(Clone, Copy, Default)]
pub struct ModbusSensorData {
    pub temperature: f32,
    pub humidity: f32,
    pub status: u16,
    pub uptime: u32,
}

/// Build a Modbus response for a ReadHoldingRegisters request
///
/// Returns the number of bytes written to the buffer
pub fn build_read_registers_response(
    buffer: &mut [u8],
    request: &ModbusRequest,
    sensor_data: &ModbusSensorData,
) -> Result<usize, u8> {
    // Validate function code
    if request.function_code != function_code::READ_HOLDING_REGISTERS
        && request.function_code != function_code::READ_INPUT_REGISTERS
    {
        return Err(exception_code::ILLEGAL_FUNCTION);
    }

    // Validate quantity (1-125 registers)
    if request.quantity == 0 || request.quantity > 125 {
        return Err(exception_code::ILLEGAL_DATA_VALUE);
    }

    // Build register data
    let mut register_data = [0u8; 256];
    let mut reg_pos = 0;

    for i in 0..request.quantity {
        let reg_addr = request.start_address + i;

        let reg_value = match reg_addr {
            // Temperature: registers 0-1 (f32)
            0 => f32_to_registers(sensor_data.temperature)[0],
            1 => f32_to_registers(sensor_data.temperature)[1],
            // Humidity: registers 2-3 (f32)
            2 => f32_to_registers(sensor_data.humidity)[0],
            3 => f32_to_registers(sensor_data.humidity)[1],
            // Status: register 4
            4 => sensor_data.status,
            // Uptime: registers 5-6 (u32)
            5 => (sensor_data.uptime >> 16) as u16,
            6 => (sensor_data.uptime & 0xFFFF) as u16,
            // Out of range
            _ => return Err(exception_code::ILLEGAL_DATA_ADDRESS),
        };

        register_data[reg_pos..reg_pos + 2].copy_from_slice(&reg_value.to_be_bytes());
        reg_pos += 2;
    }

    let byte_count = (request.quantity * 2) as u8;

    // Build response
    let mut pos = 0;

    // MBAP Header (copy transaction_id, protocol_id, update length)
    let response_header = MbapHeader {
        transaction_id: request.header.transaction_id,
        protocol_id: 0,
        length: (3 + byte_count) as u16,  // unit_id + fc + byte_count + data
        unit_id: request.header.unit_id,
    };
    pos += response_header.to_bytes(&mut buffer[pos..]);

    // Function code
    buffer[pos] = request.function_code;
    pos += 1;

    // Byte count
    buffer[pos] = byte_count;
    pos += 1;

    // Register data
    buffer[pos..pos + reg_pos].copy_from_slice(&register_data[..reg_pos]);
    pos += reg_pos;

    Ok(pos)
}

/// Build a Modbus exception response
pub fn build_exception_response(
    buffer: &mut [u8],
    request: &ModbusRequest,
    exception_code: u8,
) -> usize {
    let mut pos = 0;

    // MBAP Header
    let response_header = MbapHeader {
        transaction_id: request.header.transaction_id,
        protocol_id: 0,
        length: 3,  // unit_id + fc + exception_code
        unit_id: request.header.unit_id,
    };
    pos += response_header.to_bytes(&mut buffer[pos..]);

    // Function code with error bit set (0x80)
    buffer[pos] = request.function_code | 0x80;
    pos += 1;

    // Exception code
    buffer[pos] = exception_code;
    pos += 1;

    pos
}

// ============================================================================
// Helper Functions
// ============================================================================

/// Convert f32 to two Modbus registers (big-endian IEEE 754)
fn f32_to_registers(value: f32) -> [u16; 2] {
    let bytes = value.to_be_bytes();
    [
        u16::from_be_bytes([bytes[0], bytes[1]]),
        u16::from_be_bytes([bytes[2], bytes[3]]),
    ]
}

// ============================================================================
// Main Handler
// ============================================================================

/// Result of handling a Modbus request
pub enum ModbusResponse {
    /// Valid response to send
    Response(usize),
    /// Exception response to send
    Exception(usize),
    /// Invalid packet, ignore
    Invalid,
}

/// Handle an incoming Modbus TCP packet
///
/// Returns the response to send back
pub fn handle_request(
    data: &[u8],
    sensor_data: &ModbusSensorData,
    response_buffer: &mut [u8],
) -> ModbusResponse {
    // Parse request
    let request = match parse_request(data) {
        Some(r) => r,
        None => return ModbusResponse::Invalid,
    };

    info!("Modbus: FC={} Addr={} Qty={}",
          request.function_code,
          request.start_address,
          request.quantity);

    // Build response
    match build_read_registers_response(response_buffer, &request, sensor_data) {
        Ok(len) => ModbusResponse::Response(len),
        Err(exc) => {
            let len = build_exception_response(response_buffer, &request, exc);
            ModbusResponse::Exception(len)
        }
    }
}
