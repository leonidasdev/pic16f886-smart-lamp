# protocol/ - Communication Protocol and Timing

**Purpose**: Implements UART communication protocol and time-based event scheduling for coordinated system operation.

**Design Philosophy**: Provides reliable data exchange with error detection (CRC-8) and deterministic timing for sensor/actuator operations.

---

## Contents

| Module | Purpose | Key Features |
|--------|---------|--------------|
| **protocol_uart** | UART communication | Binary protocol, CRC-8 error detection, state machine parser |
| **scheduler** | Time-based events | 10ms/100ms tick system, event flags for main loop |

---

## Module: protocol_uart (UART Communication Protocol)

### Overview

Binary UART protocol for bidirectional communication between PIC16F886 (SMA-LAMP) and host computer (SMA-COMP).

### Protocol Specifications

| Parameter | Value | Notes |
|-----------|-------|-------|
| **Baudrate** | 9600 bps | 8N1 format |
| **Frame Format** | Binary | Fixed header, variable length |
| **Error Detection** | CRC-8-CCITT | Polynomial 0x07 |
| **Max Frame Size** | 33 bytes | Header + Length + Cmd + 28 data + 2 CRC |
| **Direction** | Bidirectional | Async, full-duplex |
| **Timeout** | 100ms | Configurable in HAL |

---

### Frame Structure

**General Format**:
```
┌────────┬────────┬─────────┬──────────────┬────────┬────────┐
│ HEADER │ LENGTH │ COMMAND │ DATA (0-28)  │ CRC_LO │ CRC_HI │
├────────┼────────┼─────────┼──────────────┼────────┼────────┤
│  0xAA  │ 3-31   │ 0x00-06 │ Variable     │ CRC-8  │  0x00  │
└────────┴────────┴─────────┴──────────────┴────────┴────────┘
  1 byte   1 byte   1 byte     0-28 bytes    1 byte   1 byte
```

**Field Descriptions**:

1. **HEADER (0xAA)**: 
   - Fixed synchronization byte
   - Identifies frame start
   - Value: 0xAA (chosen for uniqueness)

2. **LENGTH**: 
   - Total bytes after LENGTH field
   - Formula: `LENGTH = 1(CMD) + N(DATA) + 2(CRC)`
   - Range: 3 to 31 bytes
   - Example: CMD only → LENGTH = 3

3. **COMMAND**: 
   - Command type identifier
   - Range: 0x00-0x06 (7 commands defined)
   - See command table below

4. **DATA**: 
   - Payload specific to command
   - Length: 0 to 28 bytes
   - Big-endian for multi-byte values

5. **CRC_LO**: 
   - CRC-8-CCITT checksum
   - Calculated over: [LENGTH][COMMAND][DATA...]
   - Polynomial: 0x07, Init: 0x00

6. **CRC_HI**: 
   - Always 0x00
   - Reserved for future 16-bit CRC

---

### Command Reference

| ID | Name | Direction | Data Bytes | Description |
|----|------|-----------|------------|-------------|
| **0x00** | CMD_NOISE | PIC → Host | 1 | Noise level category (0=LOW, 1=MED, 2=HIGH) |
| **0x01** | CMD_LUX | PIC → Host | 2 | Luminosity in lux (0-65535, big-endian) |
| **0x02** | CMD_CO2 | PIC → Host | 2 | CO₂ concentration in ppm (0-65535, 0xFFFF=warmup) |
| **0x03** | CMD_HUM | PIC → Host | 2 | Relative humidity in % (0-100, big-endian) |
| **0x04** | CMD_FAN | Host → PIC | 1 | Fan speed 0-100% |
| **0x05** | CMD_LEDS | Host → PIC | 4 | RGB + Brightness: [R][G][B][Bright(0-31)] |
| **0x06** | CMD_TEMP | PIC → Host | 1 | Temperature in °C (0-255) |

**Big-Endian Format**:
```c
// Sending 16-bit value (e.g., lux = 1234):
data[0] = (lux >> 8) & 0xFF;  // MSB first
data[1] = lux & 0xFF;          // LSB second

// Receiving:
uint16_t value = ((uint16_t)data[0] << 8) | data[1];
```

---

### CRC-8 Implementation

**Algorithm**: CRC-8-CCITT

**Parameters**:
- Polynomial: 0x07 (x^8 + x^2 + x + 1)
- Initial value: 0x00
- Input reflection: No
- Output reflection: No
- Final XOR: 0x00

**Calculation**:
```c
uint8_t protocol_crc8(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}
```

**What is Checksummed**:
```
Input: [LENGTH][COMMAND][DATA...]
Output: CRC_LO byte
```

**Example**:
```
Frame: AA 04 06 19 XX XX
Data for CRC: [04][06][19]
CRC-8: Calculate → 0xXX (placed in CRC_LO position)
```

---

### API Reference

```c
void protocol_init(uint32_t baud);
```
**Description**: Initializes UART and protocol state machine

**Parameters**:
- `baud`: Baudrate in bps (typically 9600)

**Call once** in main() before using protocol functions.

---

```c
void protocol_send(uint8_t cmd, const uint8_t *data, uint8_t len);
```
**Description**: Sends a complete frame with automatic CRC calculation

**Parameters**:
- `cmd`: Command ID (CMD_NOISE, CMD_LUX, etc.)
- `data`: Pointer to data buffer (can be NULL if len=0)
- `len`: Number of data bytes (0-28)

**Example**:
```c
// Send temperature (25°C)
uint8_t temp = 25;
protocol_send(CMD_TEMP, &temp, 1);

// Send humidity (60%)
uint16_t hum = 60;
uint8_t hum_bytes[2] = {(hum >> 8) & 0xFF, hum & 0xFF};
protocol_send(CMD_HUM, hum_bytes, 2);

// Send LED command
uint8_t led_data[4] = {255, 0, 0, 31};  // Red, full brightness
protocol_send(CMD_LEDS, led_data, 4);
```

**Timing**: ~10ms @ 9600 baud for 10-byte frame

---

```c
bool protocol_feed_byte(uint8_t byte);
```
**Description**: Feeds one byte to state machine parser (use in ISR)

**Parameters**:
- `byte`: Received byte from UART

**Returns**: 
- `true` if complete valid frame received
- `false` if frame incomplete or invalid

**Usage Pattern** (in ISR):
```c
void __interrupt() isr(void) {
    if (PIR1bits.RCIF) {
        uint8_t byte = RCREG;
        if (protocol_feed_byte(byte)) {
            // Complete frame received
            new_command_received = true;
        }
    }
}
```

**State Machine**: Automatically handles frame parsing, validation, and CRC checking.

---

```c
bool protocol_receive(uint8_t *cmd, uint8_t *data, uint8_t *len);
```
**Description**: Non-blocking receive with timeout (for polling in main loop)

**Parameters**:
- `cmd`: Pointer to store command ID
- `data`: Buffer to store received data (min 28 bytes)
- `len`: Pointer to store data length

**Returns**: 
- `true` if valid frame received
- `false` if timeout or no data

**Example**:
```c
uint8_t cmd, data[28], len;
if (protocol_receive(&cmd, data, &len)) {
    switch(cmd) {
        case CMD_FAN:
            fan_set_speed(data[0]);
            break;
        case CMD_LEDS:
            led_set_color(data[0], data[1], data[2], data[3]);
            break;
    }
}
```

**Timeout**: ~100ms (configured in hal_uart)

---

```c
uint8_t protocol_crc8(const uint8_t *data, uint8_t len);
```
**Description**: Calculates CRC-8 checksum (utility function)

**Parameters**:
- `data`: Buffer to checksum
- `len`: Number of bytes

**Returns**: CRC-8 value

**Typically called internally** by protocol_send(), but available for testing.

---

### State Machine Details

**Parser States**:

1. **STATE_HEADER**: Waiting for 0xAA
2. **STATE_LENGTH**: Reading length byte
3. **STATE_CMD**: Reading command byte
4. **STATE_DATA**: Reading data bytes (if any)
5. **STATE_CRC0**: Reading CRC_LO
6. **STATE_CRC1**: Reading CRC_HI (always 0x00)

**Transitions**:
```
HEADER ──0xAA──→ LENGTH ──valid──→ CMD ──→ DATA ──→ CRC0 ──→ CRC1
   ↑                 │               │       │        │        │
   │                 │               │       │        │        │
   └─────────────────┴───────────────┴───────┴────────┴────────┘
                   (any error or invalid byte)
```

**Error Recovery**: On any invalid byte, state resets to STATE_HEADER.

---

### Global Variables

```c
extern struct protocol_command_t received_command;
extern volatile bool new_command_received;
```

**protocol_command_t Structure**:
```c
struct protocol_command_t {
    uint8_t type;              // Command type (CMD_FAN, CMD_LEDS)
    union {
        uint8_t fan_speed;     // For CMD_FAN
        struct {               // For CMD_LEDS
            uint8_t red;
            uint8_t green;
            uint8_t blue;
            uint8_t brightness;
        } led;
    };
};
```

**Usage**:
```c
if (new_command_received) {
    new_command_received = false;  // Clear flag
    
    if (received_command.type == CMD_FAN) {
        fan_set_speed(received_command.fan_speed);
    } else if (received_command.type == CMD_LEDS) {
        led_set_color(
            received_command.led.red,
            received_command.led.green,
            received_command.led.blue,
            received_command.led.brightness
        );
    }
}
```

---

### Frame Examples

**Example 1: Send Noise Level (LOW)**
```
Raw bytes: AA 04 00 00 XX 00
           │  │  │  │  │  └─ CRC_HI (always 0x00)
           │  │  │  │  └──── CRC_LO (calculated)
           │  │  │  └─────── Data: 0 (NOISE_LOW)
           │  │  └────────── Command: CMD_NOISE (0x00)
           │  └───────────── Length: 4 (1 cmd + 1 data + 2 crc)
           └──────────────── Header: 0xAA

CRC calculation:
  Input: [04][00][00]
  CRC-8: 0x04
  
Final frame: AA 04 00 00 04 00
```

**Example 2: Send CO₂ (1234 ppm)**
```
1234 decimal = 0x04D2 hex

Raw bytes: AA 05 02 04 D2 XX 00
           │  │  │  │  │  │  └─ CRC_HI
           │  │  │  │  │  └──── CRC_LO
           │  │  │  │  └─────── Data LSB: 0xD2
           │  │  │  └────────── Data MSB: 0x04
           │  │  └───────────── Command: CMD_CO2 (0x02)
           │  └──────────────── Length: 5
           └─────────────────── Header: 0xAA

CRC calculation:
  Input: [05][02][04][D2]
  CRC-8: Calculate → CRC_LO
```

**Example 3: Receive LED Command (Red, 50% brightness)**
```
Raw bytes: AA 07 05 FF 00 00 10 XX 00
           │  │  │  │  │  │  │  │  └─ CRC_HI
           │  │  │  │  │  │  │  └──── CRC_LO
           │  │  │  │  │  │  └─────── Brightness: 16 (50% of 31)
           │  │  │  │  │  └────────── Blue: 0
           │  │  │  │  └───────────── Green: 0
           │  │  │  └──────────────── Red: 255
           │  │  └─────────────────── Command: CMD_LEDS (0x05)
           │  └────────────────────── Length: 7
           └───────────────────────── Header: 0xAA

Action:
  led_set_color(255, 0, 0, 16);
```

---

### Error Handling

**CRC Mismatch**:
- Frame discarded
- State machine resets to STATE_HEADER
- No notification to application

**Length Out of Range**:
- Frame discarded
- State machine resets

**Timeout**:
- Partial frame discarded after 100ms
- State machine resets on next byte

**Buffer Overflow Protection**:
- Data bytes >28 rejected
- State machine resets

---

### Performance

| Metric | Value | Notes |
|--------|-------|-------|
| **Frame Overhead** | 5 bytes | Header + Length + Cmd + 2×CRC |
| **Max Throughput** | ~960 bytes/s | @ 9600 baud, 8N1 |
| **Typical Frame Time** | 3-10ms | Depends on data length |
| **CRC Calculation** | ~30µs | For 10-byte frame @ 20MHz |
| **Parser CPU Usage** | <1% | When idle, ~5% during reception |

---

## Module: scheduler (Time-Based Event Scheduler)

### Overview

Provides deterministic timing for periodic operations using Timer0 (10ms) and Timer1 (~105ms) interrupts.

### Design Concept

**Two-Tier Timing**:
1. **Fast tick (10ms)**: Noise sensor sampling
2. **Slow tick (~105ms)**: Environmental sensor reading

**Event Flags**: Set by ISR, cleared by main loop when event is processed.

---

### API Reference

```c
void scheduler_init(void);
```
**Description**: Initializes scheduler counters

**Call once** in main() before enabling interrupts.

---

```c
void scheduler_tick_10ms(void);
```
**Description**: Increments fast tick counter (call from Timer0 ISR every 10ms)

**Usage**:
```c
void __interrupt() isr(void) {
    if (TMR0IF) {
        TMR0IF = 0;
        TMR0 += 61;  // Compensated reload
        scheduler_tick_10ms();
    }
}
```

---

```c
void scheduler_tick_100ms(void);
```
**Description**: Increments slow tick counter (call from Timer1 ISR every ~105ms)

**Usage**:
```c
void __interrupt() isr(void) {
    if (TMR1IF) {
        TMR1IF = 0;
        scheduler_tick_100ms();
    }
}
```

---

```c
bool scheduler_noise_ready(void);
```
**Description**: Checks if 1 second elapsed (100 × 10ms ticks)

**Returns**: 
- `true` if noise update due
- `false` otherwise

**Auto-clears flag** when returning true.

**Usage**:
```c
if (scheduler_noise_ready()) {
    noise_cat_t noise = noise_read_category();
    protocol_send(CMD_NOISE, &noise, 1);
}
```

---

```c
bool scheduler_env_ready(void);
```
**Description**: Checks if 5 seconds elapsed (50 × ~105ms ticks ≈ 5.25s)

**Returns**: 
- `true` if environmental sensor update due
- `false` otherwise

**Auto-clears flag** when returning true.

**Usage**:
```c
if (scheduler_env_ready()) {
    uint8_t temp = temp_read_degC();
    uint16_t hum = hum_read_percent();
    // ... read other sensors
    protocol_send(CMD_TEMP, &temp, 1);
    protocol_send(CMD_HUM, &hum, 2);
}
```

---

### Timing Accuracy

**Timer0 (10ms tick)**:
```
Fosc/4 = 5 MHz
Prescaler: 1:256
Tick period: 51.2 µs
Ticks per 10ms: 195.3
Preload: TMR0 = 61 (compensated)

Actual period: 10.0032 ms
Error: 0.032% (excellent)
```

**Timer1 (~105ms tick)**:
```
Fosc/4 = 5 MHz
Prescaler: 1:8
Overflow: 65536 counts
Period: 65536 × 1.6µs = 104.8576 ms

Actual period: ~105ms
Error: 0.14% (acceptable for 5s target)
```

**5 Second Period** (environmental sensors):
```
50 ticks × 104.8576ms = 5.243 seconds
Error: +4.86% (acceptable for non-critical timing)
```

---

### Implementation Details

**Internal Counters**:
```c
static uint16_t tick_count_10ms = 0;
static uint16_t tick_count_100ms = 0;
```

**Thresholds**:
- Noise: 100 ticks (1.00 second)
- Environmental: 50 ticks (5.24 seconds)

**Auto-Reset**: Counters reset to 0 when threshold reached.

---

### Integration with Main Loop

**Typical Pattern**:
```c
void main(void) {
    // Initialize
    scheduler_init();
    init_timers();  // Enable Timer0, Timer1 interrupts
    
    while(1) {
        // Event 1: Noise (every 1s)
        if (scheduler_noise_ready()) {
            // Read and transmit noise
        }
        
        // Event 2: Environmental (every 5s)
        if (scheduler_env_ready()) {
            // Read and transmit all env sensors
        }
        
        // Event 3: Commands (asynchronous)
        if (new_command_received) {
            // Process received command
        }
    }
}
```

**Benefits**:
- Main loop is simple and non-blocking
- No delay loops (all timing in ISR)
- Easy to add new events
- Deterministic timing

---

### Adding New Events

**Example**: Add event every 30 seconds

1. **Add counter check in scheduler.c**:
```c
static uint16_t tick_count_30s = 0;

void scheduler_tick_100ms(void) {
    // Existing code...
    
    tick_count_30s++;
    if (tick_count_30s >= 286) {  // 30s / 105ms ≈ 286
        tick_count_30s = 0;
        flag_30s_ready = true;
    }
}
```

2. **Add flag and accessor in scheduler.h**:
```c
bool scheduler_30s_ready(void);
```

3. **Use in main loop**:
```c
if (scheduler_30s_ready()) {
    // Do 30-second task
}
```

---

## Protocol Testing

### Manual Frame Construction

**Python Test Script**:
```python
import serial

def crc8(data):
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc

def send_fan_command(ser, speed):
    """Send fan speed command (0-100%)"""
    frame = [0xAA, 0x04, 0x04, speed]
    crc = crc8([0x04, 0x04, speed])
    frame += [crc, 0x00]
    ser.write(bytes(frame))

def send_led_command(ser, r, g, b, brightness):
    """Send LED color command"""
    frame = [0xAA, 0x07, 0x05, r, g, b, brightness]
    crc = crc8([0x07, 0x05, r, g, b, brightness])
    frame += [crc, 0x00]
    ser.write(bytes(frame))

# Open serial port
ser = serial.Serial('COM3', 9600, timeout=1)

# Test fan
send_fan_command(ser, 50)  # 50% speed

# Test LEDs
send_led_command(ser, 255, 0, 0, 31)  # Red, full brightness
```

---

### Loopback Test

**Test protocol without sensors**:
```c
// In main.c test mode:
void test_protocol_loopback(void) {
    uint8_t test_data[2] = {0x12, 0x34};
    protocol_send(CMD_LUX, test_data, 2);
    
    // Verify frame on terminal:
    // AA 05 01 12 34 XX 00
}
```

---

## Troubleshooting

### Protocol Issues

| Symptom | Cause | Solution |
|---------|-------|----------|
| No frames received | Wrong baudrate | Verify both sides at 9600 |
| Garbled data | Electrical noise | Add ferrite beads, shorter cables |
| CRC errors | Framing error | Check start bit timing |
| Frames timeout | Buffer overflow | Increase read frequency |
| State machine stuck | Missing header | Reset state on timeout |

### Scheduler Issues

| Symptom | Cause | Solution |
|---------|-------|----------|
| Events too fast | Wrong counter threshold | Verify tick counts |
| Events too slow | Timer not running | Check INTCON, PIE bits |
| Missed events | Main loop blocking | Reduce blocking operations |
| Irregular timing | ISR latency | Optimize ISR code |

---

## References

### Standards

- **CRC-8-CCITT**: ITU-T I.432.1 (ATM cell header)
- **UART**: TIA/EIA-232-F (RS-232)

### Application Notes

- AN774: Asynchronous Communications with PIC (Microchip)
- AN510: Implementing Ultra Reliable Communication (Microchip)

### Tools

- **Terminal**: RealTerm, Tera Term (for manual testing)
- **Analyzer**: Saleae Logic Analyzer (protocol debugging)
- **Python**: pyserial library (automated testing)

---

[← Back to Main README](../README.md)
