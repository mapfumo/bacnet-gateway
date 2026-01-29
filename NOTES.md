# BACnet Gateway - Technical Notes

## Framework Decision: Embassy over RTIC

**Decision Date:** January 2026

### Context

This project implements a BACnet/IP gateway on STM32F446RE with W5500 Ethernet. The initial skeleton used RTIC 2.x, but we chose to switch to Embassy.

### Options Considered

1. **RTIC 2.x** - Hardware interrupt-driven concurrency with static priority analysis
2. **Embassy** - Async/await framework with cooperative multitasking

### Decision: Embassy

We chose Embassy for the following reasons:

#### 1. Code Reuse
The `wk11-unified-monitoring` project already has battle-tested Embassy code for:
- W5500 SPI driver with DMA
- SHT3x I2C sensor reading
- SSD1306 OLED display
- Network socket handling

Copying and extending this code is faster than porting to RTIC.

#### 2. BACnet/IP Timing Requirements
BACnet/IP runs over UDP on Ethernet. The timing characteristics are:
- Network latency: 1-10ms typical
- No hard real-time requirements
- Who-Is/I-Am discovery: broadcast/response pattern
- ReadProperty: request/response pattern

The network stack dominates timing, not the MCU scheduler. Both frameworks would perform identically for this use case.

#### 3. When RTIC Would Matter
RTIC's deterministic interrupt handling would be valuable for:
- **BACnet MS/TP** (RS-485 token passing) - microsecond timing requirements
- Mixing BACnet with hard real-time tasks (motor control, safety systems)
- Formal WCET (Worst Case Execution Time) analysis for certification

Since we're using BACnet/IP over Ethernet, these considerations don't apply.

### Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    BACnet/IP Gateway                        │
├─────────────────────────────────────────────────────────────┤
│  Embassy Executor                                           │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐         │
│  │ Sensor Task │  │ BACnet Task │  │ Display Task│         │
│  │ (2s poll)   │  │ (UDP:47808) │  │ (2s update) │         │
│  └─────────────┘  └─────────────┘  └─────────────┘         │
├─────────────────────────────────────────────────────────────┤
│  Hardware Abstraction                                       │
│  ┌─────────┐  ┌─────────┐  ┌─────────┐                     │
│  │ SHT3x   │  │ W5500   │  │ SSD1306 │                     │
│  │ (I2C1)  │  │ (SPI1)  │  │ (I2C1)  │                     │
│  └─────────┘  └─────────┘  └─────────┘                     │
└─────────────────────────────────────────────────────────────┘
```

### References

- [Embassy Book](https://embassy.dev/book/)
- [RTIC Book](https://rtic.rs/)
- [BACnet/IP Standard](http://www.bacnet.org/)
- `wk11-unified-monitoring` - Reference implementation with Modbus TCP

### Future Considerations

If BACnet MS/TP support is added later (RS-485 token passing), consider:
1. Using RTIC for the MS/TP timing-critical token management
2. Or using Embassy with careful interrupt handling for token timing
3. The UDP/IP portions can remain async regardless
