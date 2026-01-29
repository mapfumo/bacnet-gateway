//! BACnet/IP Protocol Implementation
//!
//! This module implements a minimal BACnet/IP stack for embedded systems.
//!
//! # BACnet Packet Structure
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────┐
//! │ BVLC (4 bytes) │ NPDU (2+ bytes) │ APDU (variable)      │
//! └─────────────────────────────────────────────────────────┘
//! ```
//!
//! - BVLC: BACnet Virtual Link Control (IP transport wrapper)
//! - NPDU: Network Protocol Data Unit (routing info)
//! - APDU: Application Protocol Data Unit (the actual command/response)
//!
//! # Supported Services
//!
//! - Who-Is (receive) → I-Am (send): Device discovery
//! - ReadProperty (receive) → Response (send): Read sensor values

// ============================================================================
// PART 1: Constants and Configuration
// ============================================================================

/// BACnet/IP UDP port (0xBAC0 = 47808)
pub const BACNET_PORT: u16 = 47808;

/// Our device instance number (unique identifier on the network)
/// BMS systems use this to address our device
pub const DEVICE_INSTANCE: u32 = 1234;

/// Device name (shown in BMS software)
pub const DEVICE_NAME: &str = "BACnet-Gateway-01";

/// Vendor ID (0 = unknown/experimental)
pub const VENDOR_ID: u16 = 0;

// ============================================================================
// PART 2: BVLC (BACnet Virtual Link Control)
// ============================================================================
//
// BVLC is the "envelope" that wraps BACnet messages for IP transport.
// It's always 4 bytes at the start of every BACnet/IP packet.
//
// Format:
//   Byte 0: Type (always 0x81 for BACnet/IP)
//   Byte 1: Function code
//   Byte 2-3: Total length (big-endian, includes BVLC header)

/// BVLC type for BACnet/IP
pub const BVLC_TYPE_BACNET_IP: u8 = 0x81;

/// BVLC function codes
pub mod bvlc_function {
    /// Unicast message (point-to-point)
    pub const ORIGINAL_UNICAST: u8 = 0x0A;
    /// Broadcast message (to all devices)
    pub const ORIGINAL_BROADCAST: u8 = 0x0B;
}

/// BVLC Header (4 bytes)
///
/// This is always at the start of a BACnet/IP packet.
#[derive(Debug, Clone, Copy)]
pub struct BvlcHeader {
    /// Always 0x81 for BACnet/IP
    pub bvlc_type: u8,
    /// Function code (unicast, broadcast, etc.)
    pub function: u8,
    /// Total packet length including this header
    pub length: u16,
}

impl BvlcHeader {
    /// Parse BVLC header from bytes
    ///
    /// # Example
    /// ```
    /// let data = [0x81, 0x0B, 0x00, 0x12]; // Broadcast, 18 bytes total
    /// let header = BvlcHeader::from_bytes(&data).unwrap();
    /// ```
    pub fn from_bytes(data: &[u8]) -> Option<Self> {
        if data.len() < 4 {
            return None;
        }

        // Check if it's a valid BACnet/IP packet
        if data[0] != BVLC_TYPE_BACNET_IP {
            return None;
        }

        Some(BvlcHeader {
            bvlc_type: data[0],
            function: data[1],
            length: u16::from_be_bytes([data[2], data[3]]),
        })
    }

    /// Write BVLC header to buffer
    ///
    /// Returns the number of bytes written (always 4)
    pub fn to_bytes(&self, buffer: &mut [u8]) -> usize {
        if buffer.len() < 4 {
            return 0;
        }

        buffer[0] = self.bvlc_type;
        buffer[1] = self.function;
        buffer[2] = (self.length >> 8) as u8;  // High byte
        buffer[3] = (self.length & 0xFF) as u8; // Low byte

        4  // We wrote 4 bytes
    }
}

// ============================================================================
// PART 3: NPDU (Network Protocol Data Unit)
// ============================================================================
//
// NPDU handles routing between BACnet networks.
// For simple single-network setups, it's just 2 bytes.
//
// Format (simple case):
//   Byte 0: Version (always 0x01)
//   Byte 1: Control flags (usually 0x00 for no routing)

/// NPDU version (always 1)
pub const NPDU_VERSION: u8 = 0x01;

/// Simple NPDU header (no routing)
#[derive(Debug, Clone, Copy)]
pub struct NpduHeader {
    /// Protocol version (always 1)
    pub version: u8,
    /// Control byte - bit flags for routing options
    /// For simple networks: 0x00 (no special routing)
    pub control: u8,
}

impl NpduHeader {
    /// Create a simple NPDU (no routing, no special flags)
    pub fn simple() -> Self {
        NpduHeader {
            version: NPDU_VERSION,
            control: 0x00,
        }
    }

    /// Parse NPDU header from bytes
    pub fn from_bytes(data: &[u8]) -> Option<Self> {
        if data.len() < 2 {
            return None;
        }

        Some(NpduHeader {
            version: data[0],
            control: data[1],
        })
    }

    /// Write NPDU header to buffer
    pub fn to_bytes(&self, buffer: &mut [u8]) -> usize {
        if buffer.len() < 2 {
            return 0;
        }

        buffer[0] = self.version;
        buffer[1] = self.control;

        2  // We wrote 2 bytes
    }
}

// ============================================================================
// PART 4: APDU Types and Service Codes
// ============================================================================
//
// The APDU is where the actual BACnet commands live.
// The first byte tells us what type of message it is.
//
// PDU Type (upper 4 bits of first byte):
//   0x0 = Confirmed Request (expects a response)
//   0x1 = Unconfirmed Request (no response expected)
//   0x2 = Simple ACK
//   0x3 = Complex ACK (contains data)
//   0x5 = Error

/// APDU PDU Types (upper 4 bits)
pub mod pdu_type {
    pub const CONFIRMED_REQUEST: u8 = 0x00;
    pub const UNCONFIRMED_REQUEST: u8 = 0x10;
    pub const SIMPLE_ACK: u8 = 0x20;
    pub const COMPLEX_ACK: u8 = 0x30;
    pub const ERROR: u8 = 0x50;
}

/// Unconfirmed service codes
pub mod unconfirmed_service {
    /// Who-Is: "Is anyone there? Tell me about yourself!"
    pub const WHO_IS: u8 = 0x08;
    /// I-Am: "I'm device X with these properties"
    pub const I_AM: u8 = 0x00;
}

/// Confirmed service codes
pub mod confirmed_service {
    /// ReadProperty: "What's the value of object X, property Y?"
    pub const READ_PROPERTY: u8 = 0x0C;
}

// ============================================================================
// PART 5: BACnet Object Types
// ============================================================================
//
// BACnet organizes everything into "objects". Each object has:
//   - Type (Device, Analog Input, Binary Output, etc.)
//   - Instance number (to distinguish multiple objects of same type)
//   - Properties (Name, Value, Units, etc.)
//
// Our device exposes:
//   - Device object (instance 1234) - represents the whole device
//   - Analog Input 0 - Temperature sensor
//   - Analog Input 1 - Humidity sensor

/// BACnet Object Types
pub mod object_type {
    pub const ANALOG_INPUT: u16 = 0;
    pub const ANALOG_OUTPUT: u16 = 1;
    pub const ANALOG_VALUE: u16 = 2;
    pub const BINARY_INPUT: u16 = 3;
    pub const BINARY_OUTPUT: u16 = 4;
    pub const DEVICE: u16 = 8;
}

/// BACnet Property Identifiers
///
/// Each object has multiple properties. These are the common ones.
pub mod property_id {
    /// Object identifier (type + instance)
    pub const OBJECT_IDENTIFIER: u32 = 75;
    /// Human-readable name
    pub const OBJECT_NAME: u32 = 77;
    /// Object type
    pub const OBJECT_TYPE: u32 = 79;
    /// Current value (for inputs/outputs)
    pub const PRESENT_VALUE: u32 = 85;
    /// Description text
    pub const DESCRIPTION: u32 = 28;
    /// Engineering units
    pub const UNITS: u32 = 117;
    /// Device's object list
    pub const OBJECT_LIST: u32 = 76;
    /// Vendor identifier
    pub const VENDOR_IDENTIFIER: u32 = 120;
    /// Vendor name
    pub const VENDOR_NAME: u32 = 121;
    /// Model name
    pub const MODEL_NAME: u32 = 70;
    /// Firmware revision
    pub const FIRMWARE_REVISION: u32 = 44;
    /// Application software version
    pub const APPLICATION_SOFTWARE_VERSION: u32 = 12;
    /// System status
    pub const SYSTEM_STATUS: u32 = 112;
}

/// Engineering Units (subset)
pub mod units {
    pub const DEGREES_CELSIUS: u16 = 62;
    pub const PERCENT_RELATIVE_HUMIDITY: u16 = 29;
    pub const NO_UNITS: u16 = 95;
}

// ============================================================================
// PART 6: Sensor Data (shared with main application)
// ============================================================================

/// Current sensor readings
///
/// This struct is updated by the sensor task and read by the BACnet handler.
#[derive(Clone, Copy, Default)]
pub struct BacnetSensorData {
    pub temperature: f32,  // Degrees Celsius
    pub humidity: f32,     // Percent RH
    pub uptime: u32,       // Seconds since boot
}

// ============================================================================
// PART 7: Packet Parsing - Who-Is Detection
// ============================================================================

/// Check if an incoming packet is a Who-Is request
///
/// Who-Is packets look like:
/// ```text
/// BVLC: 81 0B 00 0C        (broadcast, 12 bytes)
/// NPDU: 01 00              (version 1, no routing)
/// APDU: 10 08              (unconfirmed request, Who-Is service)
///       [optional range]   (device instance range filter)
/// ```
pub fn is_who_is(data: &[u8]) -> bool {
    // Minimum Who-Is: BVLC(4) + NPDU(2) + APDU(2) = 8 bytes
    if data.len() < 8 {
        return false;
    }

    // Check BVLC type
    if data[0] != BVLC_TYPE_BACNET_IP {
        return false;
    }

    // Skip BVLC (4 bytes) and NPDU (2 bytes), look at APDU
    let apdu_start = 6;

    // Check PDU type (upper 4 bits) = Unconfirmed Request (0x10)
    // Check service = Who-Is (0x08)
    let pdu_type = data[apdu_start] & 0xF0;
    let service = data[apdu_start + 1];

    pdu_type == pdu_type::UNCONFIRMED_REQUEST && service == unconfirmed_service::WHO_IS
}

// ============================================================================
// PART 8: Building I-Am Response
// ============================================================================

/// Build an I-Am response packet
///
/// I-Am is sent in response to Who-Is. It tells other devices:
/// - Our device object identifier (type + instance)
/// - Max APDU length we support
/// - Segmentation support (none for us)
/// - Vendor ID
///
/// Returns the number of bytes written to the buffer.
pub fn build_i_am(buffer: &mut [u8]) -> usize {
    let mut pos = 0;

    // --- BVLC Header (4 bytes) ---
    // We'll fill in length at the end
    buffer[0] = BVLC_TYPE_BACNET_IP;  // Type
    buffer[1] = bvlc_function::ORIGINAL_BROADCAST;  // Broadcast
    pos = 4;  // Skip length for now

    // --- NPDU (2 bytes) ---
    buffer[pos] = NPDU_VERSION;  // Version 1
    buffer[pos + 1] = 0x00;      // No routing
    pos += 2;

    // --- APDU: I-Am ---
    // PDU Type + Service
    buffer[pos] = pdu_type::UNCONFIRMED_REQUEST;
    buffer[pos + 1] = unconfirmed_service::I_AM;
    pos += 2;

    // I-Am contains 4 pieces of information:

    // 1. Device Object Identifier (4 bytes)
    //    Format: context tag 0xC4, then type(10 bits) + instance(22 bits)
    buffer[pos] = 0xC4;  // Context tag for object ID
    let object_id = ((object_type::DEVICE as u32) << 22) | DEVICE_INSTANCE;
    buffer[pos + 1] = ((object_id >> 24) & 0xFF) as u8;
    buffer[pos + 2] = ((object_id >> 16) & 0xFF) as u8;
    buffer[pos + 3] = ((object_id >> 8) & 0xFF) as u8;
    buffer[pos + 4] = (object_id & 0xFF) as u8;
    pos += 5;

    // 2. Max APDU Length Accepted (2 bytes)
    //    We support up to 480 bytes (encoded as 0x05)
    buffer[pos] = 0x22;  // Context tag + length
    buffer[pos + 1] = 0x05;  // 480 bytes
    pos += 2;

    // 3. Segmentation Supported (1 byte)
    //    0x03 = no segmentation
    buffer[pos] = 0x91;  // Context tag
    buffer[pos + 1] = 0x03;  // No segmentation
    pos += 2;

    // 4. Vendor ID (2 bytes)
    buffer[pos] = 0x21;  // Context tag
    buffer[pos + 1] = VENDOR_ID as u8;
    pos += 2;

    // --- Fill in BVLC length ---
    let total_len = pos as u16;
    buffer[2] = (total_len >> 8) as u8;
    buffer[3] = (total_len & 0xFF) as u8;

    pos
}

// ============================================================================
// PART 9: Parsing ReadProperty Request
// ============================================================================

/// Parsed ReadProperty request
#[derive(Debug)]
pub struct ReadPropertyRequest {
    /// Invoke ID - must be echoed in response
    pub invoke_id: u8,
    /// Object type (Device, Analog Input, etc.)
    pub object_type: u16,
    /// Object instance number
    pub object_instance: u32,
    /// Property being requested
    pub property_id: u32,
}

/// Try to parse a ReadProperty request from the packet
///
/// ReadProperty packets look like:
/// ```text
/// BVLC: 81 0A 00 11        (unicast, 17 bytes)
/// NPDU: 01 00              (version 1)
/// APDU: 00                 (confirmed request, segmented=no)
///       05                 (max-segs=0, max-apdu=5)
///       01                 (invoke-id)
///       0C                 (service = ReadProperty)
///       0C ...             (object identifier)
///       19 ...             (property identifier)
/// ```
pub fn parse_read_property(data: &[u8]) -> Option<ReadPropertyRequest> {
    // Minimum length check
    if data.len() < 15 {
        return None;
    }

    // Check BVLC
    if data[0] != BVLC_TYPE_BACNET_IP {
        return None;
    }

    // Skip BVLC (4) + NPDU (2) = 6 bytes
    let apdu = &data[6..];

    // Check PDU type (must be confirmed request)
    if apdu[0] & 0xF0 != pdu_type::CONFIRMED_REQUEST {
        return None;
    }

    // Byte 1: max segments/max APDU (skip)
    // Byte 2: invoke ID (we need this!)
    let invoke_id = apdu[2];

    // Byte 3: service choice
    if apdu[3] != confirmed_service::READ_PROPERTY {
        return None;
    }

    // Now parse the tagged parameters
    let params = &apdu[4..];
    let mut pos = 0;

    // Context tag 0: Object Identifier (4 bytes of data)
    // Tag byte 0x0C = context tag 0, 4 bytes length
    if params.len() < pos + 5 {
        return None;
    }
    if params[pos] != 0x0C {  // Context tag 0, 4 bytes
        return None;
    }
    pos += 1;

    // Decode object identifier
    let obj_bytes = &params[pos..pos + 4];
    let object_id = u32::from_be_bytes([obj_bytes[0], obj_bytes[1], obj_bytes[2], obj_bytes[3]]);
    let object_type = ((object_id >> 22) & 0x3FF) as u16;
    let object_instance = object_id & 0x3FFFFF;
    pos += 4;

    // Context tag 1: Property Identifier
    if params.len() < pos + 2 {
        return None;
    }

    // Property ID can be 1, 2, or 3 bytes depending on value
    let tag = params[pos];
    pos += 1;

    let property_id = if (tag & 0x07) == 1 {
        // 1 byte value
        params[pos] as u32
    } else if (tag & 0x07) == 2 {
        // 2 byte value
        u16::from_be_bytes([params[pos], params[pos + 1]]) as u32
    } else {
        // For now, just read 1 byte
        params[pos] as u32
    };

    Some(ReadPropertyRequest {
        invoke_id,
        object_type,
        object_instance,
        property_id,
    })
}

// ============================================================================
// PART 10: Building ReadProperty Response
// ============================================================================

/// Build a ReadProperty response for Present Value
///
/// This is called when someone asks for a sensor reading.
pub fn build_read_property_ack(
    buffer: &mut [u8],
    request: &ReadPropertyRequest,
    sensor_data: &BacnetSensorData,
) -> usize {
    let mut pos = 0;

    // --- BVLC Header ---
    buffer[0] = BVLC_TYPE_BACNET_IP;
    buffer[1] = bvlc_function::ORIGINAL_UNICAST;
    pos = 4;  // Fill length later

    // --- NPDU ---
    buffer[pos] = NPDU_VERSION;
    buffer[pos + 1] = 0x00;
    pos += 2;

    // --- APDU: Complex ACK ---
    buffer[pos] = pdu_type::COMPLEX_ACK;
    buffer[pos + 1] = request.invoke_id;  // Echo the invoke ID!
    buffer[pos + 2] = confirmed_service::READ_PROPERTY;
    pos += 3;

    // --- ReadProperty ACK parameters ---

    // Context tag 0: Object Identifier (echo back)
    buffer[pos] = 0x0C;  // Context 0, length 4
    let object_id = ((request.object_type as u32) << 22) | request.object_instance;
    buffer[pos + 1] = ((object_id >> 24) & 0xFF) as u8;
    buffer[pos + 2] = ((object_id >> 16) & 0xFF) as u8;
    buffer[pos + 3] = ((object_id >> 8) & 0xFF) as u8;
    buffer[pos + 4] = (object_id & 0xFF) as u8;
    pos += 5;

    // Context tag 1: Property Identifier (echo back)
    buffer[pos] = 0x19;  // Context 1, length 1
    buffer[pos + 1] = request.property_id as u8;
    pos += 2;

    // Context tag 3: Property Value (opening tag)
    buffer[pos] = 0x3E;  // Opening tag for context 3
    pos += 1;

    // Encode the value based on what was requested
    pos += encode_property_value(
        &mut buffer[pos..],
        request.object_type,
        request.object_instance,
        request.property_id,
        sensor_data,
    );

    // Context tag 3: closing tag
    buffer[pos] = 0x3F;
    pos += 1;

    // Fill in BVLC length
    let total_len = pos as u16;
    buffer[2] = (total_len >> 8) as u8;
    buffer[3] = (total_len & 0xFF) as u8;

    pos
}

/// Encode a property value into the buffer
///
/// Returns number of bytes written.
fn encode_property_value(
    buffer: &mut [u8],
    object_type: u16,
    object_instance: u32,
    property_id: u32,
    sensor_data: &BacnetSensorData,
) -> usize {
    match (object_type, property_id) {
        // Device object - Object Identifier
        (8, 75) => {
            // Object ID: Device, instance DEVICE_INSTANCE
            buffer[0] = 0xC4;  // Object ID tag
            let obj_id = ((object_type::DEVICE as u32) << 22) | DEVICE_INSTANCE;
            buffer[1] = ((obj_id >> 24) & 0xFF) as u8;
            buffer[2] = ((obj_id >> 16) & 0xFF) as u8;
            buffer[3] = ((obj_id >> 8) & 0xFF) as u8;
            buffer[4] = (obj_id & 0xFF) as u8;
            5
        }

        // Analog Input - Present Value
        (0, 85) => {
            // Get the appropriate sensor value
            let value = match object_instance {
                0 => sensor_data.temperature,
                1 => sensor_data.humidity,
                _ => 0.0,
            };

            // Encode as REAL (float32)
            buffer[0] = 0x44;  // Application tag: REAL (4 bytes)
            let bytes = value.to_be_bytes();
            buffer[1] = bytes[0];
            buffer[2] = bytes[1];
            buffer[3] = bytes[2];
            buffer[4] = bytes[3];
            5
        }

        // Analog Input - Object Name
        (0, 77) => {
            let name = match object_instance {
                0 => "Temperature",
                1 => "Humidity",
                _ => "Unknown",
            };
            encode_character_string(buffer, name)
        }

        // Analog Input - Units
        (0, 117) => {
            let unit = match object_instance {
                0 => units::DEGREES_CELSIUS,
                1 => units::PERCENT_RELATIVE_HUMIDITY,
                _ => units::NO_UNITS,
            };
            // Encode as enumerated
            buffer[0] = 0x91;  // Enumerated, 1 byte
            buffer[1] = unit as u8;
            2
        }

        // Device - Object Name
        (8, 77) => {
            encode_character_string(buffer, DEVICE_NAME)
        }

        // Device - Vendor ID
        (8, 120) => {
            buffer[0] = 0x21;  // Unsigned, 1 byte
            buffer[1] = VENDOR_ID as u8;
            2
        }

        // Analog Value (object type 2) - Used for computed/derived values like uptime
        // Unlike Analog Input (physical sensors), Analog Value represents software-generated data
        (2, 85) => {  // Property 85 = Present Value
            if object_instance == 0 {  // AV0 = Uptime
                let value = sensor_data.uptime as f32;
                buffer[0] = 0x44;  // BACnet REAL tag (IEEE 754 float32)
                let bytes = value.to_be_bytes();
                buffer[1..5].copy_from_slice(&bytes);
                5
            } else {
                0
            }
        }

        (2, 77) => {  // Property 77 = Object Name
            if object_instance == 0 {
                encode_character_string(buffer, "Uptime")
            } else {
                0
            }
        }

        (2, 117) => {  // Property 117 = Units
            if object_instance == 0 {
                buffer[0] = 0x91;  // BACnet Enumerated tag
                buffer[1] = 73;    // Unit 73 = seconds (ASHRAE 135-2020)
                2
            } else {
                0
            }
        }

        // Default: return empty
        _ => 0,
    }
}

/// Encode a character string in BACnet format
fn encode_character_string(buffer: &mut [u8], text: &str) -> usize {
    let bytes = text.as_bytes();
    let len = bytes.len();

    // Character string: tag + encoding + string bytes
    // Tag: 0x75 for strings up to 253 bytes
    if len < 253 {
        buffer[0] = 0x75;  // Character string tag, extended length follows
        buffer[1] = (len + 1) as u8;  // Length including encoding byte
        buffer[2] = 0x00;  // UTF-8 encoding
        buffer[3..3 + len].copy_from_slice(bytes);
        3 + len
    } else {
        0  // Too long
    }
}

// ============================================================================
// PART 11: Main Packet Handler
// ============================================================================

/// Result of handling an incoming BACnet packet
pub enum BacnetResponse {
    /// No response needed
    None,
    /// Send I-Am broadcast
    IAm(usize),  // Length of response in buffer
    /// Send ReadProperty response to specific IP/port
    ReadPropertyAck {
        length: usize,
        dest_ip: [u8; 4],
        dest_port: u16,
    },
}

/// Handle an incoming BACnet/IP packet
///
/// This is the main entry point. Call this when you receive a UDP packet
/// on port 47808.
///
/// # Arguments
/// * `data` - The received UDP packet data
/// * `src_ip` - Source IP address
/// * `src_port` - Source UDP port
/// * `sensor_data` - Current sensor readings
/// * `response_buffer` - Buffer to write response into
///
/// # Returns
/// * `BacnetResponse` indicating what to send back (if anything)
pub fn handle_packet(
    data: &[u8],
    src_ip: [u8; 4],
    src_port: u16,
    sensor_data: &BacnetSensorData,
    response_buffer: &mut [u8],
) -> BacnetResponse {
    // Check for Who-Is
    if is_who_is(data) {
        let len = build_i_am(response_buffer);
        return BacnetResponse::IAm(len);
    }

    // Check for ReadProperty
    if let Some(request) = parse_read_property(data) {
        let len = build_read_property_ack(response_buffer, &request, sensor_data);
        return BacnetResponse::ReadPropertyAck {
            length: len,
            dest_ip: src_ip,
            dest_port: src_port,
        };
    }

    // Unknown packet type - ignore
    BacnetResponse::None
}
