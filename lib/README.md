# lib/ - Official Vendor Libraries

**Purpose**: Contains unmodified official library code from the microcontroller vendor/course materials.

**Status**: **READ-ONLY** - Do not modify these files

---

## Contents

| File | Version | Author | Purpose |
|------|---------|--------|---------|
| **i2c-v2.c/h** | v2 | norberto | Hardware I2C master driver (MSSP module) |
| **spi-master-v1.c/h** | v1 | norberto | Bit-bang SPI master implementation |

---

## I2C Library (i2c-v2)

### Overview

Reference implementation for I2C (Inter-Integrated Circuit) master mode using the PIC16F886 MSSP module.

### API Reference

```c
void i2c_start(void);
```
**Description**: Generates I2C START condition  
**Usage**: Call before beginning any I2C transaction  
**Hardware**: Sets SEN bit in SSPCON2, waits for completion

---

```c
void i2c_stop(void);
```
**Description**: Generates I2C STOP condition  
**Usage**: Call after completing I2C transaction  
**Hardware**: Sets PEN bit in SSPCON2, waits for completion

---

```c
void i2c_rstart(void);
```
**Description**: Generates I2C RESTART condition  
**Usage**: Used for read operations after write (address phase → data phase)  
**Hardware**: Sets RSEN bit in SSPCON2, waits for completion

---

```c
unsigned char i2c_write(unsigned char I2C_data);
```
**Description**: Writes one byte to I2C bus  
**Parameters**:
- `I2C_data`: Byte to transmit (address or data)

**Returns**: ACK status from slave (0 = ACK, 1 = NACK)  
**Hardware**: Writes to SSPBUF, waits for BF=0

**Example**:
```c
i2c_start();
i2c_write(0x10 << 1);        // Device address 0x10, write mode
i2c_write(0x04);             // Register address
i2c_stop();
```

---

```c
unsigned char i2c_read(char master_ack);
```
**Description**: Reads one byte from I2C bus  
**Parameters**:
- `master_ack`: 1 = send ACK (more data to read), 0 = send NACK (last byte)

**Returns**: Byte received from slave  
**Hardware**: Enables RCEN, waits for BF=1, reads SSPBUF

**Example**:
```c
i2c_start();
i2c_write((0x10 << 1) | 1);  // Device address 0x10, read mode
uint8_t data_msb = i2c_read(1);  // Read with ACK
uint8_t data_lsb = i2c_read(0);  // Read with NACK (last byte)
i2c_stop();
```

---

### Hardware Requirements

- **MSSP Module**: Configured in I2C master mode (SSPCON = 0x28)
- **Clock**: SSPADD register sets SCL frequency
- **Pins**: RC3 (SCL), RC4 (SDA) - require external 4.7kΩ pull-up resistors
- **Interrupts**: Not used (polling mode)

### Notes

- This library uses **blocking/polling** - waits for hardware flags
- Master must control START/STOP conditions
- Slave address must be shifted left 1 bit (LSB = R/W̅ bit)
- No error recovery - relies on HAL wrapper for timeout handling

---

## SPI Library (spi-master-v1)

### Overview

Software-based SPI (Serial Peripheral Interface) master implementation using bit-banging on GPIO pins.

### API Reference

```c
char spi_write_read(char one_byte);
```
**Description**: Transmits one byte via SPI and simultaneously receives one byte  
**Parameters**:
- `one_byte`: Byte to transmit (MOSI)

**Returns**: Byte received (MISO) - always 0x00 in this project (no MISO connected)

**Timing**: ~10µs per bit @ 20MHz (software delay loops)

**Example**:
```c
spi_write_read(0xE0 | brightness);  // Send LED brightness byte
spi_write_read(blue);               // Send blue component
spi_write_read(green);              // Send green component
spi_write_read(red);                // Send red component
```

---

### Pin Definitions

```c
#define spi_clk     PORTCbits.RC0   // Clock output
#define spi_dat_out PORTCbits.RC1   // Data output (MOSI)
```

**Configuration**:
- RC0: SPI Clock (toggles for each bit)
- RC1: Master Out Slave In (data transmission)
- No MISO: SK9822 LEDs don't send data back

---

### Hardware Requirements

- **No MSSP module required**: Pure software implementation
- **Pins**: RC0 (clock), RC1 (data) - must be configured as outputs
- **Timing**: Not timing-critical for SK9822 LEDs (can run slow)
- **Mode**: Compatible with SPI Mode 0 (CPOL=0, CPHA=0)

### Algorithm

1. For each of 8 bits (MSB first):
   - Set data line (RC1) = current bit value
   - Set clock low (RC0 = 0)
   - Delay
   - Set clock high (RC0 = 1)
   - Delay
   - (Optional: Read MISO if connected)
2. Return received byte

### Notes

- ⚠️ Software delays may vary with compiler optimization
- Not suitable for high-speed SPI (use hardware MSSP for >1 MHz)
- Clock frequency: ~100 kHz (adequate for SK9822 LEDs)
- No chip select (CS) control - handled by protocol (start/stop frames)

---

## Integration with HAL

These libraries are **wrapped by the HAL layer**:

```
Application Code
      ↓
  hal_i2c.h  ←—— wraps —→ lib/i2c-v2.h
      ↓
  MSSP Hardware

Application Code
      ↓
  hal_spi.h  ←—— wraps —→ lib/spi-master-v1.h
      ↓
  GPIO Pins (RC0/RC1)
```

### Why Use HAL Wrappers?

1. **Initialization**: HAL handles register configuration (SSPCON, SSPADD, TRIS, etc.)
2. **Error Handling**: HAL adds timeout protection and error checking
3. **Portability**: Easier to replace libraries without changing application code
4. **Consistency**: Unified API style across all peripherals
5. **Safety**: HAL validates parameters and prevents misuse

**Example**:

```c
// DON'T: Call directly from application
i2c_start();
i2c_write(0x20);
i2c_stop();

// DO: Use HAL wrapper instead
hal_i2c_start();
hal_i2c_write(0x20);
hal_i2c_stop();
```

---

## References

### I2C Protocol
- **I²C-bus specification v6.0** (NXP UM10204)
- **PIC16F886 Datasheet** - Section 11: Master Synchronous Serial Port (MSSP)
- Clock frequency: 100 kHz standard mode, 400 kHz fast mode

### SPI Protocol
- **SPI Block Guide** (Motorola)
- **SK9822 Datasheet** - Compatible with APA102 protocol
- Clock polarity: CPOL=0, Clock phase: CPHA=0

### PIC16F886 Hardware
- **Datasheet**: DS41291E (Microchip)
- **MSSP Module**: Pages 89-104 (I2C master mode)
- **GPIO**: Pages 33-39 (digital I/O configuration)

---

## Important Notes

### Do NOT Modify

These files are official reference implementations:
- Maintain compatibility with course materials
- Used by multiple projects/students
- Modifications may break existing code
- Version control history may be lost

### If You Need Changes

Instead of modifying `lib/`, create wrappers or extensions in `hal/`:

```c
// hal/hal_i2c.c - extends lib/i2c-v2.h
#include "../lib/i2c-v2.h"

void hal_i2c_init(void) {
    // Configure MSSP registers
    SSPCON = 0x28;
    SSPADD = 49;  // 100 kHz @ 20MHz
    // ...
}

uint8_t hal_i2c_write_timeout(uint8_t data) {
    uint16_t timeout = 0xFFFF;
    // Add timeout protection to i2c_write()
    // ...
}
```

---

## Troubleshooting

### I2C Not Working

| Symptom | Cause | Solution |
|---------|-------|----------|
| Bus always high | Missing pull-ups | Add 4.7kΩ resistors on SCL/SDA |
| No ACK from slave | Wrong address | Check device address (7-bit vs 8-bit) |
| Clock too fast | SSPADD too low | Recalculate: `SSPADD = (Fosc/4/Fscl) - 1` |
| Hangs on i2c_write() | Hardware not initialized | Call HAL init before using lib functions |

### SPI Not Working

| Symptom | Cause | Solution |
|---------|-------|----------|
| LEDs don't light | Pins not outputs | Set TRISC0=0, TRISC1=0 |
| Random LED colors | Timing too fast | Increase delay in spi_write_read() |
| First LED wrong | Protocol mismatch | SK9822: send 4×0x00 header, 4×0xFF tail |

---

## Version Information

| Library | Version | Date | Source |
|---------|---------|------|--------|
| i2c-v2 | 2.0 | Jun 29, 2022 | Course materials (norberto) |
| spi-master-v1 | 1.0 | Jul 12, 2022 | Course materials (norberto) |

**Last Updated**: December 2025  
**Maintained By**: Universidad Politécnica de Madrid - Sistemas Empotrados

---

[← Back to Main README](../README.md)
