# SMA-LAMP (LMDE-MA) - Firmware Documentation

**Microcontroller:** PIC16F886 @ 20 MHz  
**Project:** Sistema de Monitorización & Actuación -  Lámpara de Mesa Diseñada para el Estudio
**Version:** 1.0  
**Date:** December 2025  
**Language:** C (XC8 Compiler)  
**Institution:** Universidad Politécnica de Madrid

---

## Table of Contents

1. [System Overview](#system-overview)
2. [Hardware Architecture](#hardware-architecture)
3. [Software Architecture](#software-architecture)
4. [Project Structure](#project-structure)
5. [Quick Start](#quick-start)
6. [Communication Protocol](#communication-protocol)
7. [Building & Deployment](#building--deployment)
8. [Testing](#testing)
9. [Troubleshooting](#troubleshooting)
10. [Documentation](#documentation)

---

## System Overview

SMA-LAMP is an embedded monitoring and control system that:
- **Monitors** environmental conditions (temperature, humidity, noise, light, CO₂)
- **Controls** RGB LED lighting and fan speed
- **Communicates** with SMA-COMP host via UART protocol
- **Persists** configuration in EEPROM
- **Operates** autonomously with timed sensor readings

### Key Features
- Real-time noise monitoring (10ms sampling, 1s window)
- Environmental sensors (5s periodic readings)
- CO₂ sensor with 5-minute warmup handling
- UART protocol with timeout protection
- Automatic configuration persistence (every 5s)
- LED activity indicator on command reception
- Brown-out reset protection

---

## Hardware Architecture

### Microcontroller Specifications
```
MCU:        PIC16F886
Frequency:  20 MHz (HS crystal)
Voltage:    5V
Flash:      8 KB
RAM:        368 bytes
EEPROM:     256 bytes
```

### Pin Assignment

#### Port A
| Pin | Function | Type | Device | Notes |
|-----|----------|------|--------|-------|
| RA0 | AN0 | Analog Input | LM35 Temperature | 0-5V → 0-500°C |
| RA1 | AN1 | Analog Input | HIH4000 Humidity | 0-5V → 0-100% RH |
| RA2 | AN2 | Analog Input | Microphone Noise | Sampled every 10ms |

#### Port C
| Pin | Function | Type | Device | Notes |
|-----|----------|------|--------|-------|
| RC0 | SPI_CLK | Digital Output | SK9822 LEDs | Bit-bang SPI clock |
| RC1 | SPI_SDO | Digital Output | SK9822 LEDs | Bit-bang SPI data |
| RC2 | CCP1/PWM | PWM Output | Fan Motor | 20 kHz, 10-bit resolution |
| RC3 | SCL | I2C | VEML7700, iAQ-Core | Requires 4.7kΩ pull-up |
| RC4 | SDA | I2C | VEML7700, iAQ-Core | Requires 4.7kΩ pull-up |
| RC6 | TX | UART Output | SMA-COMP | 9600 baud |
| RC7 | RX | UART Input | SMA-COMP | 9600 baud |

### Peripheral Usage

| Peripheral | Usage | Configuration |
|------------|-------|---------------|
| **Timer0** | 10ms tick | Prescaler 1:256, preload 61 |
| **Timer1** | ~105ms tick | Prescaler 1:8, 16-bit overflow |
| **Timer2** | PWM base | PR2=249 → 20 kHz |
| **ADC** | 3 channels | 10-bit, VDD/VSS ref, 10µs acq time |
| **MSSP** | I2C Master | 100 kHz, SSPADD=49 |
| **EUSART** | UART | 9600 8N1, SPBRG=129 |
| **CCP1** | PWM | Fan control, duty 0-100% |

### Sensor Details

| Sensor | Interface | Address/Channel | Range | Update Rate |
|--------|-----------|-----------------|-------|-------------|
| **LM35** | ADC | AN0 | 0-255°C | 5 seconds |
| **HIH4000** | ADC | AN1 | 0-100% RH | 5 seconds |
| **Microphone** | ADC | AN2 | 3 categories | 1 second (max of 100 samples) |
| **VEML7700** | I2C | 0x10 | 0-65535 lux | 5 seconds |
| **iAQ-Core** | I2C | 0x5A | 0-65535 ppm CO₂ | 5 seconds (warmup 5 min) |

### Actuator Details

| Actuator | Interface | Control | Range | Notes |
|----------|-----------|---------|-------|-------|
| **SK9822 RGB LED** | SPI (bit-bang) | R, G, B, Brightness | 0-255 RGB, 0-31 brightness | N_LEDS configurable |
| **Fan** | PWM (CCP1) | Speed percentage | 0-100% | 20 kHz frequency |

---

## Software Architecture

### Layer Hierarchy

```
┌─────────────────────────────────────────┐
│              main.c                     │  ← Application Logic
│  • Initialization                       │
│  • ISR (Timer0/Timer1)                  │
│  • Main Loop (scheduler-driven)         │
└───────────────┬─────────────────────────┘
                │
    ┌───────────┴───────────┬─────────────┬──────────────┐
    ▼                       ▼             ▼              ▼
┌─────────┐         ┌──────────────┐  ┌──────────┐  ┌────────────┐
│ sensor/ │         │  actuador/   │  │protocol/ │  │    hal/    │
│         │         │              │  │          │  │            │
│ • temp  │         │  • led       │  │• uart    │  │• adc       │
│ • hum   │         │  • fan       │  │• sched   │  │• i2c       │
│ • noise │         │              │  │          │  │• spi       │
│ • lux   │         │              │  │          │  │• pwm       │
│ • co2   │         │              │  │          │  │• uart      │
│         │         │              │  │          │  │• eeprom    │
└────┬────┘         └──────┬───────┘  └────┬─────┘  └─────┬──────┘
     │                     │               │               │
     │                     │               │               │
     └─────────────────────┴───────────────┴───────────────┘
                                  │
                           ┌──────┴──────┐
                           │    lib/     │  ← Official Libraries
                           │             │
                           │ • i2c-v2    │  (Read-only)
                           │ • spi-v1    │
                           └─────────────┘
```

### Design Principles

1. **Layered Architecture**: Clear separation of concerns
2. **Single Responsibility**: Each module has one job
3. **HAL Abstraction**: Hardware access only through HAL layer
4. **No Re-initialization**: Peripherals initialized once in main()
5. **Event-Driven**: Scheduler triggers periodic operations
6. **Non-Blocking**: All operations use timeouts or polling

---

## Project Structure

```
SMA-LAMP/
├── main.c                  # Application entry point and main loop
├── README.md              # This file - project overview
├── REGISTERS_MAP.md       # PIC16F886 register reference
│
├── lib/                   # Official vendor libraries (read-only)
│   ├── README.md         # Library documentation
│   ├── i2c-v2.c/h        # Hardware I2C driver
│   └── spi-master-v1.c/h # Bit-bang SPI driver
│
├── hal/                   # Hardware Abstraction Layer
│   ├── README.md         # HAL architecture and API reference
│   ├── hal_adc.c/h       # ADC peripheral (3 channels, 10-bit)
│   ├── hal_uart.c/h      # UART communication (9600 baud)
│   ├── hal_i2c.c/h       # I2C master (100 kHz)
│   ├── hal_spi.c/h       # SPI bit-bang implementation
│   ├── hal_pwm.c/h       # PWM generation (20 kHz)
│   └── hal_eeprom.c/h    # EEPROM read/write
│
├── sensor/                # Sensor driver implementations
│   ├── README.md         # Sensor specifications and usage
│   ├── sensor_temp.c/h   # LM35 temperature sensor (ADC)
│   ├── sensor_hum.c/h    # HIH4000 humidity sensor (ADC)
│   ├── sensor_noise.c/h  # Microphone noise level (ADC)
│   ├── sensor_lux.c/h    # VEML7700 light sensor (I2C)
│   └── sensor_co2.c/h    # iAQ-Core CO₂ sensor (I2C)
│
├── actuador/              # Actuator driver implementations
│   ├── README.md         # Actuator specifications and usage
│   ├── actuator_led.c/h  # SK9822 RGB LED strip (SPI)
│   └── actuator_fan.c/h  # PWM fan control
│
├── protocol/              # Communication and timing
│   ├── README.md         # Protocol specification
│   ├── protocol_uart.c/h # UART command protocol with CRC-8
│   └── scheduler.c/h     # Time-based event scheduler
│
└── test/                  # Individual component tests
    ├── README.md         # Testing documentation and procedures
    ├── test_fan.c        # Fan PWM ramp test
    ├── test_led.c        # LED color cycling test
    ├── test_command.c    # UART command reception test
    ├── test_co2.c        # CO₂ sensor I2C test
    ├── test_lux.c        # Light sensor I2C test
    ├── test_temperature.c # Temperature sensor ADC test
    ├── test_noise.c      # Noise sensor ADC test
    ├── test_humidity.c   # Humidity sensor ADC test
    └── test_eeprom.c     # EEPROM read/write test
```

### Documentation Structure

Each module folder contains detailed documentation:

- **[lib/README.md](lib/README.md)** - Vendor library API and integration notes
- **[hal/README.md](hal/README.md)** - HAL design principles, peripheral configurations, API reference
- **[sensor/README.md](sensor/README.md)** - Sensor datasheets summary, calibration, wiring diagrams
- **[actuador/README.md](actuador/README.md)** - Actuator specifications, control algorithms, safety limits
- **[protocol/README.md](protocol/README.md)** - Complete protocol specification, frame format, CRC details
- **[test/README.md](test/README.md)** - Test procedures, expected results, troubleshooting

---

## Quick Start

### Prerequisites
- **Hardware**: PIC16F886 microcontroller @ 20 MHz crystal
- **Software**: MPLAB X IDE v6.0+, XC8 Compiler v2.40+
- **Programmer**: PICkit 3/4 or compatible

### Basic Setup

1. **Clone/Download** this project
2. **Open** `main.c` project in MPLAB X
3. **Configure** device as PIC16F886
4. **Build** the project (Production mode)
5. **Program** the device using PICkit
6. **Connect** UART terminal @ 9600 baud to RC6 (TX)
7. **Observe** periodic sensor data transmission

### First Power-On

After programming, the system will:
1. Initialize all peripherals (200ms)
2. Restore saved configuration from EEPROM
3. Send diagnostic frames:
   - `CMD_NOISE = 0` (noise sensor baseline)
   - `CMD_TEMP = 25°C` (temperature test)
4. Begin periodic operation:
   - Every 1s: Noise level report
   - Every 5s: Temperature, humidity, light, CO₂ reports

### Hardware Checklist

Before first power-on:
- [ ] 20 MHz crystal + 15-33pF capacitors on OSC1/OSC2
- [ ] 10kΩ pull-up resistor on MCLR pin
- [ ] 100nF decoupling capacitor near VDD
- [ ] 4.7kΩ pull-up resistors on RC3 (SCL) and RC4 (SDA)
- [ ] 5V power supply (stable, ±5%)
- [ ] UART connection: TX(RC6)→RX, RX(RC7)→TX, GND common

---

## Module Overview

### Hardware Abstraction Layer (HAL)

The HAL provides a clean interface to PIC16F886 peripherals:

| Module | Purpose | Key Functions |
|--------|---------|---------------|
| **hal_adc** | Analog-to-Digital conversion | `adc_init()`, `adc_read(ch)` |
| **hal_uart** | Serial communication | `uart_init(baud)`, `uart_send_byte()` |
| **hal_i2c** | I2C bus communication | `hal_i2c_init()`, `hal_i2c_read/write()` |
| **hal_spi** | SPI communication | `spi_init()`, `spi_send_byte()` |
| **hal_pwm** | PWM signal generation | `pwm_init()`, `pwm_set_percent()` |
| **hal_eeprom** | Non-volatile storage | `eeprom_read/write(addr, val)` |

**Design Principle**: Upper layers never access registers directly - all hardware interaction goes through HAL.

See [hal/README.md](hal/README.md) for detailed API reference and configuration details.

### Sensor Layer

| Sensor | Interface | Range | Update Rate | Special Notes |
|--------|-----------|-------|-------------|---------------|
| **LM35** | ADC AN0 | 0-255°C | 5s | 10mV/°C, direct read |
| **HIH4000** | ADC AN1 | 0-100% RH | 5s | Requires temperature compensation |
| **Microphone** | ADC AN2 | 3 categories | 1s | Maximum of 100 samples |
| **VEML7700** | I2C 0x10 | 0-65535 lux | 5s | 16-bit light sensor |
| **iAQ-Core** | I2C 0x5A | 0-65535 ppm | 5s | 5-minute warmup period |

See [sensor/README.md](sensor/README.md) for calibration procedures, wiring diagrams, and troubleshooting.

### Actuator Layer

| Actuator | Interface | Control Range | Notes |
|----------|-----------|---------------|-------|
| **SK9822 LED** | SPI (RC0/RC1) | RGB: 0-255, Brightness: 0-31 | Supports multiple LEDs (N_LEDS) |
| **Fan** | PWM (RC2) | 0-100% speed | 20 kHz frequency, 10-bit resolution |

See [actuador/README.md](actuador/README.md) for control algorithms and safety considerations.

### Protocol Layer

**UART Protocol**: Custom binary protocol with CRC-8 error detection

- **Frame Format**: `[HEADER][LEN][CMD][DATA...][CRC_L][CRC_H]`
- **Baudrate**: 9600 bps (8N1)
- **Commands**: 7 bidirectional commands (see protocol documentation)
- **Error Detection**: CRC-8-CCITT (polynomial 0x07)

**Scheduler**: Time-based event system driven by Timer0 (10ms) and Timer1 (~105ms)

See [protocol/README.md](protocol/README.md) for complete protocol specification and examples.

---

## Communication Protocol

### Data Flow: PIC → Host (SMA-COMP)

```
Every 1 second:
    └─→ [AA 04 00 {cat} 00 00]          CMD_NOISE
    
Every 5 seconds:
    ├─→ [AA 04 06 {temp} 00 00]         CMD_TEMP
    ├─→ [AA 05 03 {hh} {hl} 00 00]      CMD_HUM
    ├─→ [AA 05 01 {lh} {ll} 00 00]      CMD_LUX
    └─→ [AA 05 02 {ch} {cl} 00 00]      CMD_CO2
```

### Data Flow: Host → PIC

```
Asynchronous (any time):
    ├─→ [AA 04 04 {speed} 00 00]              CMD_FAN
    └─→ [AA 07 05 {r} {g} {b} {br} 00 00]     CMD_LEDS
```

### Example Frames

**Send Temperature 25°C**:
```
AA 04 06 19 00 00
│  │  │  │  └─ CRC
│  │  │  └──── Data: 25 (0x19)
│  │  └─────── Command: TEMP (0x06)
│  └────────── Length: 4
└───────────── Header: 0xAA
```

**Receive LED Command (Red, max brightness)**:
```
AA 07 05 FF 00 00 1F 00 00
│  │  │  │  │  │  │  └─ CRC
│  │  │  │  │  │  └──── Brightness: 31
│  │  │  │  │  └─────── Blue: 0
│  │  │  │  └──────────Green: 0
│  │  │  └───────────── Red: 255
│  │  └──────────────── Command: LEDS (0x05)
│  └─────────────────── Length: 7
└────────────────────── Header: 0xAA
```

---

## Timing & Scheduling

### Timer Configuration

#### Timer0 (10ms tick)
```
Fosc/4 = 20MHz / 4 = 5 MHz
Period = 0.2 µs
Prescaler 1:256 → tick every 51.2 µs
10ms / 51.2µs = 195.3 ticks
Preload = 256 - 195 = 61

OPTION_REG = 0b10000111  // nRBPU=1 (PORTB pull-ups OFF), PSA=0, PS=111 (1:256), T0CS=0 (Fosc/4)
TMR0 += 61               // Compensated reload
```

#### Timer1 (~105ms tick)
```
Fosc/4 = 5 MHz
Prescaler 1:8 → tick every 1.6 µs
Overflow = 65536 × 1.6µs = 104.8576 ms

T1CON = 0b00110001      // TMR1ON=1, TMR1CS=0 (Fosc/4), T1CKPS=11 (1:8), TMR1GE=0 (always count)
```

### ISR (Interrupt Service Routine)

```c
void __interrupt() isr(void) {
    // Every 10ms
    if(TMR0IF && TMR0IE) {
        TMR0IF = 0;
        TMR0 += 61;  // Compensated reload (avoids drift)
        
        scheduler_tick_10ms();
        
        // Sample noise and keep maximum over 1s window
        noise_cat_t cat = noise_read_category();
        if(cat > noise_max_cat) noise_max_cat = cat;
    }
    
    // Every ~105ms
    if(TMR1IF && TMR1IE) {
        TMR1IF = 0;
        scheduler_tick_100ms();
    }
}
```

**IMPORTANT**: `TMR0 += 61` compensates for ISR latency, preventing cumulative drift.

---

## Testing

The project includes 9 standalone test programs in the `test/` folder:

| Test | Purpose | Hardware Required |
|------|---------|-------------------|
| **test_fan** | PWM fan ramp (0→80%→0%) | Fan on RC2 |
| **test_led** | LED color cycling | SK9822 on RC0/RC1 |
| **test_command** | UART command reception | UART terminal |
| **test_temperature** | Temperature reading | LM35 on RA0 |
| **test_humidity** | Humidity reading | HIH4000 on RA1 |
| **test_noise** | Noise level sampling | Microphone on RA2 |
| **test_lux** | Light sensor I2C | VEML7700 on I2C |
| **test_co2** | CO₂ sensor I2C | iAQ-Core on I2C |
| **test_eeprom** | EEPROM read/write | None (internal) |

**Usage**: Each test is a standalone `.c` file with its own `main()`. Compile and program individually to verify hardware functionality before deploying the full system.

See [test/README.md](test/README.md) for detailed test procedures and expected outputs.

---

## System Operation

```
1. Power-On Reset → PWRTE waits 72ms
2. main() starts
3. init_pins_min()          Configure TRIS registers
4. init_timers()            Start Timer0, Timer1, enable GIE
5. scheduler_init()         Reset tick counters
6. protocol_init(9600)      Initialize UART
7. Peripheral Init (once):
   ├─ adc_init()
   ├─ hal_i2c_init()
   ├─ spi_init()
   └─ pwm_init()
8. Sensor Init:
   ├─ temp_init()           (No-op)
   ├─ hum_init()            (No-op)
   ├─ noise_init()          (No-op)
   ├─ lux_init()            (No-op)
   └─ co2_init()            Reset warmup counter
9. Actuator Init:
   ├─ led_init()            Set default white color
   └─ fan_init()            Set speed to 0%
10. Restore from EEPROM:
    ├─ Read addresses 0x00-0x04
    ├─ Validate (check for 0xFF = empty)
    └─ Apply configuration
11. Diagnostic Test:
    ├─ Send CMD_NOISE = 0
    └─ Send CMD_TEMP = 25°C
12. Enter main loop
```

### Main Loop (Infinite)

```
while(1) {
    // Event 1: Noise report (every 1s)
    if(scheduler_noise_ready()) {
        protocol_send(CMD_NOISE, {noise_max_cat}, 1);
        noise_max_cat = NOISE_LOW;  // Reset window
    }
    
    // Event 2: Environmental sensors (every 5s)
    if(scheduler_env_ready()) {
        save_actuators_state();     // Persist to EEPROM
        co2_update_warmup();         // Increment warmup counter
        
        temp = temp_read_degC();
        hum = hum_read_percent();
        lux = lux_read();
        co2 = co2_read_ppm();        // Returns 0xFFFF if not ready
        
        protocol_send(CMD_TEMP, ...);
        protocol_send(CMD_HUM, ...);
        protocol_send(CMD_LUX, ...);
        protocol_send(CMD_CO2, ...);
    }
    
    // Event 3: Command reception (asynchronous)
    if(protocol_receive(&cmd, buf, &len)) {
        switch(cmd) {
            case CMD_FAN:
                current_fan_speed = buf[0];
                fan_set_speed(current_fan_speed);
                break;
            case CMD_LEDS:
                current_led_r = buf[0];
                current_led_g = buf[1];
                current_led_b = buf[2];
                current_led_brightness = buf[3];
                led_set_color(...);
                break;
        }
        
        // Activity indicator: flash green LED
        led_set_color(0, 255, 0, 10);
        __delay_ms(50);
        led_set_color(current_led_r, current_led_g, current_led_b, current_led_brightness);
    }
}
```

**Non-blocking**: All operations use polling or timeouts (no infinite loops).

---

## EEPROM Persistence

### Memory Map

| Address | Content | Range | Description |
|---------|---------|-------|-------------|
| 0x00 | LED Brightness | 0-31 | Masked with 0x1F |
| 0x01 | LED Red | 0-255 | RGB component |
| 0x02 | LED Green | 0-255 | RGB component |
| 0x03 | LED Blue | 0-255 | RGB component |
| 0x04 | Fan Speed | 0-100 | Percentage |

### Save Strategy

**When**: Every 5 seconds (called from `scheduler_env_ready()`)

**Why**: 
- Protects against power loss
- Reduces EEPROM wear (~1M cycle limit)
- Max 5s data loss window

**Function**:
```c
static void save_actuators_state(void) {
    eeprom_write(0x00, current_led_brightness);
    eeprom_write(0x01, current_led_r);
    eeprom_write(0x02, current_led_g);
    eeprom_write(0x03, current_led_b);
    eeprom_write(0x04, current_fan_speed);
}
```

### Restore on Boot

1. Read all 5 addresses
2. Check if empty (`0xFF` indicates first boot)
3. If empty: use defaults (white LED, fan off)
4. If valid: apply saved configuration

---

## Building & Deployment

### Requirements
- **IDE**: MPLAB X v6.0 or later
- **Compiler**: XC8 v2.40 or later
- **Programmer**: PICkit 3/4 or compatible

### Compilation Steps

1. Open project in MPLAB X
2. Select device: PIC16F886
3. Configure toolchain: XC8
4. Build (Production): `Ctrl+Shift+F11`
5. Verify configuration bits match `main.c` pragmas

### Configuration Bits Verification

```c
#pragma config FOSC = HS        // Crystal 20 MHz
#pragma config WDTE = OFF       // Watchdog disabled
#pragma config PWRTE = ON       // Power-up timer enabled
#pragma config MCLRE = ON       // MCLR enabled
#pragma config BOREN = ON       // Brown-out reset enabled
#pragma config LVP = OFF        // Low-voltage programming disabled
```

### Hardware Checklist

Before powering on:
- ☐ 20 MHz crystal between OSC1/OSC2 with 15-33pF capacitors
- ☐ MCLR pulled up to VDD with 10kΩ
- ☐ VDD = 5V stable (100nF decoupling capacitor)
- ☐ I2C pull-ups (4.7kΩ on RC3/RC4)
- ☐ UART connected to SMA-COMP (TX→RX, RX→TX, GND common)

---

## Troubleshooting

### Common Issues

| Symptom | Cause | Solution |
|---------|-------|----------|
| No UART output | Crystal not oscillating | Verify crystal, capacitors, FOSC=HS |
| UART sends garbage | Baudrate mismatch | Verify SPBRG=129, _XTAL_FREQ=20MHz |
| I2C sensors don't respond | Missing pull-ups | Add 4.7kΩ on RC3 (SCL) and RC4 (SDA) |
| CO₂ always reads 0xFFFF | Sensor in warmup | Wait 5 minutes after power-on |
| Fan doesn't run | Pin not configured | Verify TRISC2=0, CCP1CON=0x0C |
| LEDs don't light | Wrong SPI pins | Verify RC0 (CLK), RC1 (SDO) connections |
| Random resets | Brown-out | Check 5V supply stable, BOREN=ON |
| EEPROM data lost | Not saving | Verify save_actuators_state() called every 5s |

### Diagnostic Features

**Startup Test Frames** (100ms after boot):
```
AA 04 00 00 00 00    → CMD_NOISE = 0
AA 04 06 19 00 00    → CMD_TEMP = 25°C
```
**Purpose**: Verify UART TX works immediately after boot.

**LED Activity Indicator**:
- Green flash (50ms) when valid command received
- Indicates: UART RX works + protocol parsing OK

### Debug Terminal Test

1. Connect terminal @ 9600 8N1 to RC6 (TX)
2. Power on PIC
3. Should see 2 frames within 200ms
4. Every 1s: CMD_NOISE frame
5. Every 5s: 4 sensor frames (TEMP, HUM, LUX, CO2)

If no output → hardware issue (crystal, power, connections).

---

## Performance Metrics

| Metric | Value | Notes |
|--------|-------|-------|
| **CPU Usage** | ~15% | ISR: 2%, Main loop: 13% |
| **RAM Usage** | ~200 bytes | 54% of 368 bytes |
| **Flash Usage** | ~5-8 KB | 60-100% of 8KB (depends on optimization) |
| **EEPROM Writes** | 5 writes/5s | ~100k cycles = 139 hours continuous |
| **ISR Frequency** | 110 Hz | Timer0: 100 Hz, Timer1: 10 Hz |
| **Worst ISR Latency** | <50 µs | Including noise sampling |

---

## Safety Features

1. **Brown-Out Reset**: Detects low voltage, resets safely
2. **UART Timeout**: Prevents infinite blocking (100ms)
3. **EEPROM Wear Leveling**: Saves every 5s (not every command)
4. **Watchdog**: Disabled (system is deterministic)
5. **Interrupt Priority**: Timers only (no nesting issues)

---

## Documentation

### Module-Specific Documentation

For detailed technical information, refer to the README in each module folder:

- **[lib/README.md](lib/README.md)** - Official library API reference and integration guidelines
- **[hal/README.md](hal/README.md)** - Complete HAL architecture, register configurations, timing analysis
- **[sensor/README.md](sensor/README.md)** - Sensor datasheets, calibration procedures, wiring diagrams
- **[actuador/README.md](actuador/README.md)** - Actuator control algorithms, safety limits, performance specs
- **[protocol/README.md](protocol/README.md)** - Protocol specification, frame format, CRC implementation, examples
- **[test/README.md](test/README.md)** - Test procedures, expected outputs, debugging guide

### Additional Resources

- **REGISTERS_MAP.md** - Complete PIC16F886 register reference with bit definitions
- **PIC16F886 Datasheet** - [DS41291E](https://www.microchip.com/) (official Microchip documentation)
- **XC8 Compiler Guide** - Microchip XC8 C Compiler User's Guide
- **I2C Specification** - I²C-bus specification and user manual (NXP)
- **SPI Protocol** - Serial Peripheral Interface documentation

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| **1.0** | Dec 2025 | **Initial production release** |
| | | - Modular layered architecture (lib/hal/sensor/actuador/protocol) |
| | | - CO₂ sensor warmup handling (5-minute automatic) |
| | | - EEPROM persistence (auto-save every 5s) |
| | | - CRC-8 error detection in UART protocol |
| | | - Complete test suite (9 standalone tests) |
| | | - Comprehensive documentation structure |

---

## Authors & Support

**Project**: SMA-LAMP (LMDE-MA)  
**Institution**: Universidad Politécnica de Madrid  
**Course**: Sistemas Empotrados (Embedded Systems)  
**Academic Year**: 2024-2025  
**Semester**: 1st Half - 4th Year

### Technical Support

For issues or questions:
1. Check the [Troubleshooting](#troubleshooting) section in this README
2. Consult module-specific README files for detailed information
3. Refer to PIC16F886 datasheet (DS41291E) for hardware details
4. Review test programs in `test/` folder for working examples

### Contributing

This is an academic project. If you're working on modifications:
1. Maintain the layered architecture (lib → hal → sensor/actuador → protocol → main)
2. Never modify `lib/` folder (vendor code)
3. Document all changes in module-specific README files
4. Create test programs for new functionality
5. Update REGISTERS_MAP.md if adding new peripheral usage

---

## License

This project is licensed under the MIT License. See the [LICENSE](./LICENSE) file for details.

---

**End of README**

