# hal/ - Hardware Abstraction Layer

**Purpose**: Provides a clean, portable interface to PIC16F886 peripherals, isolating hardware-specific details from application code.

**Design Philosophy**: All hardware register access must go through HAL - upper layers (sensor/actuator/protocol) never touch registers directly.

---

## Contents

| Module | Peripheral | Purpose |
|--------|------------|---------|
| **hal_adc** | ADC (10-bit) | Analog-to-Digital conversion for sensors |
| **hal_uart** | EUSART | Serial communication with SMA-COMP host |
| **hal_i2c** | MSSP (I2C) | I2C bus master for digital sensors |
| **hal_spi** | GPIO (bit-bang) | SPI communication for LED strip |
| **hal_pwm** | Timer2 + CCP1 | PWM generation for fan control |
| **hal_eeprom** | EEPROM | Non-volatile configuration storage |

---

## Architecture Principles

### 1. Single Initialization
Peripherals are initialized **once** in `main()` during startup:
```c
void main(void) {
    adc_init();
    uart_init(9600);
    hal_i2c_init();
    spi_init();
    pwm_init();
    // Never call *_init() again
}
```

### 2. Stateless Operations
HAL functions don't maintain state - they operate directly on hardware:
```c
uint16_t temp = adc_read(0);  // Read AN0, no state tracking
```

### 3. Error Handling
HAL provides timeout and error detection:
```c
if (!uart_read_byte_timeout(&byte)) {
    // Handle timeout/error
}
```

### 4. Interrupt Safety
Critical sections protected when necessary:
```c
INTCONbits.GIE = 0;  // Disable interrupts
// Critical ADC operation
INTCONbits.GIE = 1;  // Re-enable interrupts
```

---

## Module: hal_adc (Analog-to-Digital Converter)

### Overview
10-bit ADC with 3 enabled channels (AN0, AN1, AN2) for analog sensors.

### API Reference

```c
void adc_init(void);
```
**Description**: Configures ADC module for 10-bit right-justified conversions  
**Called**: Once in main() startup  
**Configuration**:
- ANSEL: 0b00000111 (AN0, AN1, AN2 enabled)
- ADCON1: Right-justified (ADFM=1), VDD/VSS reference (VCFG=00)
- ADCON0: FOSC/32 clock (TAD=1.6µs), ADC enabled (ADON=1)

---

```c
uint16_t adc_read(uint8_t channel);
```
**Description**: Performs single ADC conversion on specified channel  
**Parameters**:
- `channel`: ADC channel (0-15, but only 0-2 are enabled)

**Returns**: 10-bit result (0-1023), or 0 if invalid channel

**Timing**:
- Acquisition: 10µs (fixed delay)
- Conversion: ~12 TAD = 19.2µs
- Total: ~30µs per read

**Example**:
```c
uint16_t raw = adc_read(0);  // Read AN0 (temperature sensor)
// Convert: 5V / 1024 steps = 4.88 mV/step
```

**⚠️ Thread Safety**: Disables interrupts during conversion to prevent ISR conflicts

---

### Hardware Configuration

| Register | Value | Description |
|----------|-------|-------------|
| ANSEL | 0b00000111 | AN0-AN2 analog, rest digital |
| ANSELH | 0x00 | All Port B digital |
| ADCON0 | 0b10000001 | ADCS=10 (FOSC/32), CHS=0000 (AN0), ADON=1 |
| ADCON1 | 0b10000000 | ADFM=1 (right-justify), VCFG=00 (VDD/VSS) |

### Timing Analysis

**Clock Selection**: FOSC/32
- FOSC = 20 MHz → FOSC/4 = 5 MHz
- TAD = 32 × 0.2µs = 1.6µs (within 0.8-6.0µs spec)

**Conversion Time**:
- Acquisition: 10µs (software delay)
- Conversion: 12 TAD = 19.2µs
- Total per channel: ~30µs

### Pin Assignment

| Pin | Channel | Sensor | Range |
|-----|---------|--------|-------|
| RA0 | AN0 | LM35 Temperature | 0-5V (0-500°C) |
| RA1 | AN1 | HIH4000 Humidity | 0-5V (0-100% RH) |
| RA2 | AN2 | Microphone | 0-5V (analog audio) |

---

## Module: hal_uart (Universal Asynchronous Receiver/Transmitter)

### Overview
Asynchronous serial communication at 9600 baud (8N1) for host communication.

### API Reference

```c
void uart_init(uint32_t baud);
```
**Description**: Initializes EUSART module for async communication  
**Parameters**:
- `baud`: Baudrate in bps (typically 9600)

**Configuration**:
- Format: 8 data bits, no parity, 1 stop bit (8N1)
- Mode: Asynchronous, high-speed (BRGH=1)
- SPBRG calculation: `SPBRG = (FOSC / (16 × baud)) - 1`

**Example**:
```c
uart_init(9600);  // 9600 baud → SPBRG=129 (0.16% error)
```

---

```c
void uart_send_byte(uint8_t b);
```
**Description**: Transmits one byte via UART  
**Parameters**:
- `b`: Byte to transmit

**Blocking**: Waits for TXIF flag (transmit buffer empty)

**Example**:
```c
uart_send_byte(0xAA);  // Send protocol header
```

---

```c
bool uart_read_byte_timeout(uint8_t *byte);
```
**Description**: Reads one byte with timeout protection  
**Parameters**:
- `byte`: Pointer to store received byte

**Returns**: 
- `true` if byte received successfully
- `false` if timeout or error (overrun, framing)

**Timeout**: ~100ms @ 20MHz (0xFFFF iterations)

**Error Handling**:
- **Overrun (OERR)**: Clears by toggling CREN
- **Framing (FERR)**: Discards corrupted byte

**Example**:
```c
uint8_t rx;
if (uart_read_byte_timeout(&rx)) {
    // Process received byte
} else {
    // Timeout or error
}
```

---

```c
bool uart_data_available(void);
```
**Description**: Non-blocking check for received data  
**Returns**: `true` if data waiting in RCREG

**Example**:
```c
if (uart_data_available()) {
    uint8_t rx = RCREG;  // Read immediately
}
```

---

### Hardware Configuration

| Register | Value | Description |
|----------|-------|-------------|
| TXSTA | 0b00100100 | BRGH=1 (high-speed), TXEN=1 (TX enable) |
| RCSTA | 0b10010000 | SPEN=1 (serial port), CREN=1 (RX enable) |
| BAUDCTL | 0x00 | BRG16=0 (8-bit generator) |
| SPBRG | 129 | 9600 baud @ 20MHz (0.16% error) |

### Baudrate Calculation

**Formula**: `SPBRG = (FOSC / (16 × baud)) - 1`

**Example (9600 baud)**:
```
SPBRG = (20,000,000 / (16 × 9600)) - 1
      = (20,000,000 / 153,600) - 1
      = 130.208 - 1
      = 129.208 → 129

Actual baud = 20,000,000 / (16 × (129 + 1))
            = 9,615.38 baud
Error = (9615.38 - 9600) / 9600 = 0.16%
```

**Acceptable Error**: ±2% for reliable communication

### Pin Assignment

| Pin | Function | Direction | Connection |
|-----|----------|-----------|------------|
| RC6 | TX | Output | → SMA-COMP RX |
| RC7 | RX | Input | ← SMA-COMP TX |
| GND | Ground | - | Common ground |

---

## Module: hal_i2c (Inter-Integrated Circuit)

### Overview
I2C master mode at 100 kHz for digital sensor communication.

### API Reference

```c
void hal_i2c_init(void);
```
**Description**: Configures MSSP module for I2C master mode  
**Configuration**:
- Mode: I2C Master (SSPCON=0x28)
- Clock: 100 kHz (SSPADD=49)
- Pins: RC3 (SCL), RC4 (SDA)
- Slew rate: Disabled (standard mode)

**Hardware Requirements**: 4.7kΩ pull-up resistors on SCL and SDA

---

```c
void hal_i2c_start(void);
void hal_i2c_stop(void);
void hal_i2c_rstart(void);
```
**Description**: I2C bus control conditions  
**Wraps**: `lib/i2c-v2.h` functions

**Usage Pattern**:
```c
hal_i2c_start();
hal_i2c_write(device_addr);
hal_i2c_write(register_addr);
hal_i2c_rstart();
hal_i2c_write(device_addr | 0x01);  // Read mode
uint8_t data = hal_i2c_read(0);     // NACK (last byte)
hal_i2c_stop();
```

---

```c
uint8_t hal_i2c_write(uint8_t data);
uint8_t hal_i2c_read(uint8_t ack);
```
**Description**: I2C data transfer  
**Wraps**: `lib/i2c-v2.h` functions

**Parameters**:
- `data`: Byte to write (address or data)
- `ack`: 1 = ACK (more bytes), 0 = NACK (last byte)

**Returns**: 
- `hal_i2c_write()`: ACK status (0=ACK, 1=NACK)
- `hal_i2c_read()`: Received byte

---

### Hardware Configuration

| Register | Value | Description |
|----------|-------|-------------|
| SSPCON | 0x28 | SSPEN=1, I2C Master mode |
| SSPADD | 49 | 100 kHz clock @ 20MHz |
| SSPSTAT | 0x80 | Slew rate control disabled |

### Clock Calculation

**Formula**: `Fscl = FOSC / (4 × (SSPADD + 1))`

**Example (100 kHz)**:
```
100,000 = 20,000,000 / (4 × (SSPADD + 1))
SSPADD = (20,000,000 / (4 × 100,000)) - 1
       = (20,000,000 / 400,000) - 1
       = 50 - 1
       = 49
```

### Connected Devices

| Device | Address | Register | Data Size | Purpose |
|--------|---------|----------|-----------|---------|
| VEML7700 | 0x10 | 0x04 (ALS) | 2 bytes | Light sensor |
| iAQ-Core | 0x5A | - (direct read) | 9 bytes | CO₂ sensor |

---

## Module: hal_spi (Serial Peripheral Interface)

### Overview
Software bit-bang SPI for SK9822 RGB LED strip control.

### API Reference

```c
void spi_init(void);
```
**Description**: Configures GPIO pins for bit-bang SPI  
**Configuration**:
- RC0: Clock output (TRISC0=0)
- RC1: Data output/MOSI (TRISC1=0)
- No MISO (SK9822 doesn't send data back)

---

```c
void spi_send_byte(uint8_t b);
```
**Description**: Sends one byte via bit-bang SPI  
**Parameters**:
- `b`: Byte to transmit

**Wraps**: `lib/spi-master-v1.h` → `spi_write_read()`

**Timing**: ~80µs per byte @ 20MHz (software delays)

**Example**:
```c
spi_send_byte(0xE0 | brightness);  // LED brightness header
spi_send_byte(blue);
spi_send_byte(green);
spi_send_byte(red);
```

---

### Pin Assignment

| Pin | Function | Direction | Purpose |
|-----|----------|-----------|---------|
| RC0 | SPI_CLK | Output | Clock signal to LEDs |
| RC1 | SPI_MOSI | Output | Data signal to LEDs |

### SK9822 Protocol

**Frame Structure**:
```
[Start Frame: 4 × 0x00]
[LED Frame: 0xE0|brightness, B, G, R] × N_LEDS
[End Frame: 4 × 0xFF]
```

**No Chip Select**: Protocol uses start/stop frames instead

---

## Module: hal_pwm (Pulse Width Modulation)

### Overview
Hardware PWM at 20 kHz for fan speed control using Timer2 and CCP1.

### API Reference

```c
void pwm_init(void);
```
**Description**: Configures Timer2 and CCP1 for PWM output  
**Configuration**:
- Frequency: 20 kHz (PR2=249)
- Resolution: 10 bits (1000 steps)
- Pin: RC2 (CCP1)
- Mode: PWM mode (CCP1CON=0x0C)

---

```c
void pwm_set_percent(uint8_t percent);
```
**Description**: Sets PWM duty cycle  
**Parameters**:
- `percent`: Duty cycle 0-100%

**Calculation**:
```c
duty = (percent / 100) × 4 × (PR2 + 1)
     = (percent / 100) × 4 × 250
     = percent × 10

CCPR1L = duty >> 2         // 8 MSB
CCP1CON bits 5:4 = duty & 0x03  // 2 LSB
```

**Example**:
```c
pwm_set_percent(50);   // 50% duty cycle
pwm_set_percent(100);  // 100% duty cycle (always on)
pwm_set_percent(0);    // 0% duty cycle (always off)
```

---

### Hardware Configuration

| Register | Value | Description |
|----------|-------|-------------|
| T2CON | 0x04 | TMR2ON=1, Prescaler=1:1, Postscaler=1:1 |
| PR2 | 249 | Period register (20 kHz) |
| CCP1CON | 0x0C | PWM mode, DC1B bits cleared |
| CCPR1L | 0-249 | Duty cycle (8 MSB) |

### Frequency Calculation

**Formula**: `Fpwm = FOSC / (4 × (PR2 + 1) × TMR2_prescaler)`

**Example (20 kHz)**:
```
20,000 = 20,000,000 / (4 × (PR2 + 1) × 1)
PR2 = (20,000,000 / (4 × 20,000)) - 1
    = (20,000,000 / 80,000) - 1
    = 250 - 1
    = 249
```

**Resolution**: 10 bits = 1000 steps (0.1% duty cycle resolution)

---

## Module: hal_eeprom (Electrically Erasable Programmable ROM)

### Overview
Non-volatile storage for configuration persistence (256 bytes internal).

### API Reference

```c
void eeprom_write(uint8_t addr, uint8_t val);
```
**Description**: Writes one byte to EEPROM  
**Parameters**:
- `addr`: Address 0-255
- `val`: Byte value to write

**Timing**: ~4ms per write (blocking)

**⚠️ Critical Sequence**:
1. Disable interrupts (GIE=0)
2. Write 0x55 → EECON2
3. Write 0xAA → EECON2
4. Set WR bit
5. Re-enable interrupts (GIE=1)
6. Wait while WR=1

**Example**:
```c
eeprom_write(0x00, 128);  // Save LED red value at address 0
```

---

```c
uint8_t eeprom_read(uint8_t addr);
```
**Description**: Reads one byte from EEPROM  
**Parameters**:
- `addr`: Address 0-255

**Returns**: Byte value (0xFF if never written)

**Timing**: ~1µs (non-blocking)

**Example**:
```c
uint8_t saved_brightness = eeprom_read(0x04);
if (saved_brightness == 0xFF) {
    // First boot - use default
    saved_brightness = 16;
}
```

---

### Memory Map (SMA-LAMP)

| Address | Content | Default | Range |
|---------|---------|---------|-------|
| 0x00 | LED Brightness | 16 | 0-31 |
| 0x01 | LED Red | 128 | 0-255 |
| 0x02 | LED Green | 128 | 0-255 |
| 0x03 | LED Blue | 128 | 0-255 |
| 0x04 | Fan Speed | 0 | 0-100 |

### Endurance

**Write Cycles**: ~1,000,000 per location (typical)

**Project Usage**: 5 writes every 5 seconds = 1 write/second
- Lifetime: 1,000,000 / (5 writes/5s) = 1,000,000 seconds = ~278 hours continuous

**Wear Leveling**: Not implemented (unnecessary for 5s save interval)

---

## Thread Safety

### Interrupt Considerations

| Module | ISR-Safe? | Notes |
|--------|-----------|-------|
| **hal_adc** | Protected | Disables GIE during conversion |
| **hal_uart** | Yes | Uses hardware flags |
| **hal_i2c** | No | Blocking operations |
| **hal_spi** | No | Blocking bit-bang |
| **hal_pwm** | Yes | Hardware generates signal |
| **hal_eeprom** | No | Must disable GIE during write |

**Rule**: Never call I2C, SPI, or EEPROM write from ISR.

---

## Performance Metrics

| Operation | Timing | Blocking | Notes |
|-----------|--------|----------|-------|
| ADC read (single) | ~30µs | Yes | Acquisition + conversion |
| UART send byte | ~1ms | Yes | @ 9600 baud |
| UART read timeout | 0-100ms | Yes | Depends on data arrival |
| I2C transaction | ~200µs | Yes | 2-byte read @ 100kHz |
| SPI send byte | ~80µs | Yes | Bit-bang software |
| PWM update | ~1µs | No | Register write only |
| EEPROM write | ~4ms | Yes | Self-timed write cycle |
| EEPROM read | ~1µs | No | Direct register access |

---

## Troubleshooting

### Common Issues

| Symptom | Module | Cause | Solution |
|---------|--------|-------|----------|
| ADC always reads 0 | hal_adc | ANSEL not set | Verify ANSEL=0x07 |
| UART garbage output | hal_uart | Wrong SPBRG | Recalculate for FOSC |
| I2C no ACK | hal_i2c | Missing pull-ups | Add 4.7kΩ resistors |
| SPI not working | hal_spi | TRIS not output | Set TRISC0=0, TRISC1=0 |
| PWM stuck | hal_pwm | CCP1CON wrong | Verify CCP1CON=0x0C |
| EEPROM corrupt | hal_eeprom | Interrupted write | Don't call from ISR |

---

## References

- **PIC16F886 Datasheet** - DS41291E (Microchip)
- **MPLAB XC8 Compiler Guide** - DS50002053 (Microchip)
- **I²C-bus specification** - UM10204 (NXP)
- **SPI Block Guide** - S12SPIV3/D (Motorola)

---

[← Back to Main README](../README.md)
