# Troubleshooting Guide

Common issues encountered during development and their solutions.

## Embassy Task Arena Full

**Error:**
```
[ERROR] panicked at 'embassy-executor: task arena is full. You must increase the arena size'
```

**Cause:**
The main async task has large stack allocations (4x 512-byte buffers = 2KB+), which exceeds the default Embassy task arena size of 4KB.

**Solution:**
Add `task-arena-size-32768` feature to `embassy-executor` in Cargo.toml:

```toml
embassy-executor = { version = "0.6", features = [
    "arch-cortex-m",
    "executor-thread",
    "defmt",
    "integrated-timers",
    "task-arena-size-32768"  # 32KB arena for large async tasks
] }
```

**Available arena sizes:** 4096 (default), 8192, 16384, 32768, 65536

## OLED Display Not Working

**Symptoms:**
- Display shows nothing (black screen)
- `OLED init failed` warning in logs

**Possible causes:**

1. **I2C wiring issue**
   - Check connections: SCL=PB8 (D15), SDA=PB9 (D14)
   - Verify VCC=3.3V, GND=GND
   - Ensure pull-up resistors are present (4.7K on SDA and SCL)

2. **Wrong I2C address**
   - Default address is 0x3C
   - Some displays use 0x3D (check datasheet)

3. **I2C bus conflict**
   - SHT3x sensor and OLED share I2C1
   - Both use `Peripherals::steal()` which can cause issues
   - Ensure proper initialization order: SHT3x before OLED

## W5500 SPI Communication Failed

**Error:**
```
W5500 version: 0xXX - UNEXPECTED
```

**Solution:**
- Check SPI wiring: SCK=PA5, MISO=PA6, MOSI=PA7, CS=PB6, RST=PC7
- Verify W5500 module has 3.3V power
- Ensure CS pin starts HIGH (deselected)
- Check that RST pin toggles LOW then HIGH during init

## TCP Socket Won't Open

**Symptoms:**
- `TCP socket failed to open, status: 0xXX`
- `TCP listen failed, status: 0xXX`

**Common status codes:**
- 0x00 = CLOSED (socket never opened)
- 0x13 = INIT (opened but not listening)
- 0x14 = LISTEN (correct for TCP server)
- 0x17 = ESTABLISHED (client connected)

**Solution:**
- Ensure socket is closed before reopening
- Wait for status transitions (use delays)
- Check that port number is set correctly

## UDP Socket Won't Open

**Symptoms:**
- `UDP socket failed to open, status: 0xXX`

**Expected status:** 0x22 = UDP mode

**Solution:**
- Same checks as TCP socket
- Verify socket mode is set to 0x02 (UDP)

## Modbus Client Can't Connect

**Symptoms:**
- TCP connection refused
- No response from device

**Checklist:**
1. Verify IP address matches (10.10.10.100)
2. Check port 502 is correct
3. Ensure W5500 link LED is on (Ethernet connected)
4. Test with `ping 10.10.10.100`
5. Check firewall settings on client PC

## BACnet Device Not Discovered

**Symptoms:**
- Who-Is broadcast gets no I-Am response
- Yabe doesn't show device

**Checklist:**
1. Verify UDP port 47808 is open
2. Check device is on same subnet as BACnet client
3. Ensure broadcast address (255.255.255.255) is reachable
4. Verify device instance (1234) matches expected

## Sensor Reading Errors

**Error:**
```
SENSOR ERR displayed on OLED
```

**Cause:**
SHT3x I2C communication failed

**Solution:**
- Check I2C wiring (same as OLED)
- Verify SHT3x address is 0x44
- Ensure sensor has stable 3.3V power
- Check for I2C bus conflicts

## Build Errors

### "memory.x not found"

**Solution:**
Ensure you're building from the `firmware/` directory:
```bash
cd firmware
cargo build --release
```

### Linker errors about FLASH/RAM

**Solution:**
Check `memory.x` has correct values for STM32F446RE:
```
FLASH : ORIGIN = 0x08000000, LENGTH = 512K
RAM   : ORIGIN = 0x20000000, LENGTH = 128K
```

## Flashing Issues

### "No probe found"

**Solution:**
```bash
# List available probes
probe-rs list

# Use specific probe ID
probe-rs run --probe 0483:374b:XXXXXX --chip STM32F446RETx target/...
```

### "Target not responding"

**Solution:**
- Press RESET button on NUCLEO board
- Check USB cable connection
- Try `probe-rs reset --chip STM32F446RETx`

## W5500 Returns Version 0x00

**Error:**
```
W5500 version: 0x00 - UNEXPECTED
W5500 initialization failed
```

**Cause:**
SPI communication is not working. The W5500 should return 0x04 from the version register.

**Solutions:**
1. **Power cycle the board** - Sometimes the W5500 gets into a bad state that only a full power reset can fix (flashing the MCU doesn't reset the W5500's power)
2. **Check MISO connection** - Getting 0x00 often means no data is coming back from the W5500
3. **Verify wiring:**
   - PA5 → SCLK
   - PA6 → MISO (most common issue)
   - PA7 → MOSI
   - PB6 → SCS (chip select)
   - PC7 → RST
   - 3.3V → VCC
   - GND → GND

**Note:** The firmware worked, then suddenly returned 0x00 without touching wires. A power cycle fixed it. The W5500 module can get into a stuck state.

## BACnet ReadProperty Not Parsing

**Symptoms:**
- Device receives BACnet packets (logs show bytes received)
- No response is sent back
- Python collector reports "Failed to read from device"

**Cause:**
Bug in `parse_read_property()` - the context tag check was using wrong bitmask:

```rust
// WRONG:
if (params[pos] & 0xF0) != 0x0C {  // This always fails!

// CORRECT:
if params[pos] != 0x0C {  // Exact match for context tag 0, 4 bytes
```

**Explanation:**
The BACnet context tag 0x0C means:
- Bit 3 = 1 (context tag)
- Bits 4-7 = 0 (tag number 0)
- Bits 0-2 = 4 (4 bytes of data)

Using `& 0xF0` gives 0x00 (upper nibble), which doesn't equal 0x0C, so parsing always failed.

**File:** `firmware/src/bacnet.rs` line ~460

## UDP Packets Not Received (But TCP Works)

**Symptoms:**
- Modbus TCP (port 502) works fine
- BACnet UDP (port 47808) shows no received packets
- Ping to device works

**Debugging steps:**
1. Add debug logging to print UDP socket status:
   ```rust
   if let Ok(status) = common::udp_get_status(&mut spi, &mut cs).await {
       info!("UDP socket status: 0x{:02X}", status);
   }
   ```
2. Status should be 0x22 (UDP mode)
3. If 0x00, socket didn't open correctly

**Common causes:**
- Socket not opened after TCP socket
- Need to power cycle if W5500 got into bad state
- UDP packets being sent but not received (check network routing)

## I-Am Response Not Received by Client

**Symptoms:**
- Device sends I-Am (logs show "BACnet: I-Am sent")
- Python collector says "No response to Who-Is"

**Cause:**
I-Am is broadcast to 255.255.255.255:47808, but the client socket is listening on an ephemeral port (e.g., 55959).

**Workaround:**
The collector falls back to direct IP addressing and uses ReadProperty, which works because responses are unicast to the source IP:port.

**Proper fix (optional):**
Have the collector bind to port 47808 to receive broadcasts, or modify firmware to send I-Am as unicast to the requester.

## Multiple Probes Connected

**Error:**
```
Selection: Error: Failed to parse probe index
```

**Cause:**
Multiple ST-Link probes connected, probe-rs can't auto-select.

**Solution:**
List probes and specify which one:
```bash
probe-rs list
# Output shows probe IDs like: 0483:374b:0671FF3833554B3043164817

probe-rs run --probe 0483:374b:0671FF3833554B3043164817 --chip STM32F446RETx target/...
```

## Device or Resource Busy (os error 16)

**Error:**
```
Error: Device or resource busy (os error 16)
```

**Cause:**
Another process (or previous probe-rs instance) is still connected to the probe.

**Solution:**
- Close any other debug sessions
- Kill any hanging probe-rs processes: `pkill -9 probe-rs`
- Unplug and replug USB cable
