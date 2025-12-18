# actuador/ - Actuator Driver Implementations

**Purpose**: Provides high-level control interfaces for output devices (LEDs and fan), implementing device-specific protocols and safety limits.

**Design Philosophy**: Each actuator module wraps HAL calls and implements control logic, protocol handling, and parameter validation.

---

## Contents

| Module | Device | Interface | Purpose |
|--------|--------|-----------|---------|
| **actuator_led** | SK9822 RGB LED | SPI (RC0/RC1) | Color and brightness control |
| **actuator_fan** | PWM Fan | CCP1/PWM (RC2) | Speed control 0-100% |

---

## Module: actuator_led (SK9822 RGB LED Strip)

### Overview

SK9822 (APA102 compatible) addressable RGB LED strip with independent brightness control.

### Hardware Specifications

| Parameter | Value | Notes |
|-----------|-------|-------|
| **Device** | SK9822 | APA102 protocol compatible |
| **Interface** | SPI (bit-bang) | 4-wire: CLK, DATA, VDD, GND |
| **Supply** | 5V | Separate power for LED strip |
| **Current** | ~60mA per LED | At full white, max brightness |
| **Colors** | RGB | 8-bit per channel (0-255) |
| **Brightness** | 5-bit | 0-31 independent global control |
| **Pins** | RC0 (CLK), RC1 (DATA) | SPI clock and data |

### API Reference

```c
void led_init(void);
void led_set_color(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness);
```

**led_init()**

Initializes LED strip to default state (white, medium brightness).

**Default Values**:
- Red: 128
- Green: 128
- Blue: 128
- Brightness: 16 (50% of maximum 31)

**led_set_color()**

Sets RGB color and brightness for all LEDs in the strip.

**Parameters**:
- `r`: Red component (0-255)
- `g`: Green component (0-255)
- `b`: Blue component (0-255)
- `brightness`: Global brightness (0-31)

**Brightness Behavior**:
- 0: LED off (regardless of RGB values)
- 1: Minimum brightness (~3%)
- 31: Maximum brightness (100%)
- Values >31 are masked to 0-31 range

**Example**:
```c
// Pure red, full brightness
led_set_color(255, 0, 0, 31);

// Cyan (green + blue), 50% brightness
led_set_color(0, 255, 255, 16);

// White, minimum brightness
led_set_color(255, 255, 255, 1);

// Off (brightness overrides RGB)
led_set_color(255, 255, 255, 0);
```

**Timing**: ~2ms for full strip update (depends on N_LEDS)

---

### SK9822 Protocol

**Frame Structure**:
```
[Start Frame] [LED Frames...] [End Frame]
```

**Start Frame** (4 bytes):
```
0x00 0x00 0x00 0x00
```

**LED Frame** (4 bytes per LED):
```
Byte 0: 111BBBBB    (brightness: 0xE0 | (brightness & 0x1F))
Byte 1: BBBBBBBB    (blue: 0-255)
Byte 2: GGGGGGGG    (green: 0-255)
Byte 3: RRRRRRRR    (red: 0-255)
```

**End Frame** (4 bytes):
```
0xFF 0xFF 0xFF 0xFF
```

**Why This Protocol?**
- Start frame: Synchronization marker
- Brightness prefix (111): Identifies brightness byte
- End frame: Ensures last LED latches data
- Order: Blue-Green-Red (device-specific)

---

### Configuration

**Number of LEDs**:

Defined in `actuator_led.c`:
```c
#define N_LEDS 1  // Adjust for your strip
```

**Multiple LEDs**: All LEDs display same color (can be extended for individual control)

**Individual LED Control** (optional enhancement):
```c
// Add to actuator_led.h:
void led_set_pixel(uint8_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t brightness);

// Implementation would maintain array of N_LEDS colors
```

---

### Wiring Diagram

```
SK9822 Strip
  VCC ──→ +5V (separate power supply recommended for >5 LEDs)
  GND ──→ GND (common ground with PIC)
  CI (Clock Input) ──→ RC0 (PIC SPI_CLK)
  DI (Data Input) ──→ RC1 (PIC SPI_DATA)

IMPORTANT: 
- PIC and LED strip must share common ground
- For strips >5 LEDs, use external 5V power supply
- Add 1000µF capacitor near LED strip power connection
- Add 470Ω resistor in series with DATA line (optional, reduces noise)
```

**Power Calculation**:
```
Total current = N_LEDS × 60mA × (brightness/31) × (color_intensity/255)

Example (10 LEDs, white, 50% brightness):
I = 10 × 60mA × (16/31) × (255/255) ≈ 310mA

PIC cannot source this - use external supply!
```

---

### Color Control

**RGB Color Space**:

| Color | Red | Green | Blue | Example Use |
|-------|-----|-------|------|-------------|
| **Red** | 255 | 0 | 0 | Error indicator |
| **Green** | 0 | 255 | 0 | Success indicator |
| **Blue** | 0 | 0 | 255 | Idle state |
| **Yellow** | 255 | 255 | 0 | Warning |
| **Cyan** | 0 | 255 | 255 | Info |
| **Magenta** | 255 | 0 | 255 | Debug |
| **White** | 255 | 255 | 255 | Normal operation |
| **Orange** | 255 | 165 | 0 | Warmup state |

**Brightness vs RGB**:
- **Brightness** (0-31): Global intensity multiplier
  - Affects all colors equally
  - More energy efficient than RGB reduction
  - Better color accuracy at low intensities

- **RGB** (0-255): Individual color channel
  - Reduces specific colors
  - Mix to create any hue
  - Full RGB = white (at current brightness)

**Best Practice**:
```c
// GOOD: Use brightness for dimming
led_set_color(255, 0, 0, 5);  // Dim red

// AVOID: Reducing RGB for dimming (loses color accuracy)
led_set_color(50, 0, 0, 31);  // Dimmer but less pure red
```

---

### Application Examples

**Status Indicator**:
```c
void indicate_status(system_status_t status) {
    switch(status) {
        case STATUS_INIT:
            led_set_color(0, 0, 255, 10);  // Blue, medium
            break;
        case STATUS_RUNNING:
            led_set_color(0, 255, 0, 5);   // Green, dim
            break;
        case STATUS_WARNING:
            led_set_color(255, 255, 0, 20); // Yellow, bright
            break;
        case STATUS_ERROR:
            led_set_color(255, 0, 0, 31);  // Red, full
            break;
    }
}
```

**Activity Indicator** (from main.c):
```c
// Flash green on command received
led_set_color(0, 255, 0, 10);
__delay_ms(50);
led_set_color(saved_r, saved_g, saved_b, saved_brightness);  // Restore
```

**Breathing Effect** (requires timer):
```c
// In ISR, increment breathing_phase 0-31-0
void isr_breathing(void) {
    static uint8_t brightness = 0;
    static int8_t direction = 1;
    
    brightness += direction;
    if (brightness == 31 || brightness == 0) {
        direction = -direction;
    }
    
    led_set_color(saved_r, saved_g, saved_b, brightness);
}
```

---

### Troubleshooting

| Issue | Cause | Solution |
|-------|-------|----------|
| LEDs don't light | No power or wrong wiring | Check 5V supply, verify CLK/DATA pins |
| First LED wrong color | Start frame missing | Verify sk9822_send_header() called |
| Random colors | SPI timing issue | Reduce bit-bang speed in spi_write_read() |
| LEDs flicker | Insufficient power | Use external 5V supply, add capacitor |
| Only first LED works | DATA not propagating | Check connections between LEDs |
| Stuck on last color | Missing update | Call led_set_color() to refresh |

---

## Module: actuator_fan (PWM Fan Control)

### Overview

Variable speed fan control using hardware PWM at 20 kHz.

### Hardware Specifications

| Parameter | Value | Notes |
|-----------|-------|-------|
| **Control** | PWM (CCP1) | Timer2-based hardware PWM |
| **Frequency** | 20 kHz | Ultrasonic, no audible noise |
| **Resolution** | 10-bit | 1000 steps (0.1% duty cycle) |
| **Duty Cycle** | 0-100% | User-facing percentage |
| **Pin** | RC2 (CCP1) | Must be configured as output |
| **Supply** | 5-12V | Fan-dependent (PIC only controls, doesn't power) |

### API Reference

```c
void fan_init(void);
void fan_set_speed(uint8_t percent);
```

**fan_init()**

Initializes fan to OFF state (0% duty cycle).

**Safety**: Always starts at 0% to prevent unexpected motor start.

**fan_set_speed()**

Sets fan speed as percentage of maximum.

**Parameters**:
- `percent`: Speed 0-100%
  - 0: Fan OFF
  - 1-100: Variable speed (linear)
  - Values >100 clamped to 100%

**Example**:
```c
fan_set_speed(0);    // Turn off
fan_set_speed(25);   // 25% speed (quiet)
fan_set_speed(50);   // 50% speed (moderate)
fan_set_speed(100);  // 100% speed (maximum)
```

**Response Time**: Immediate (register write), fan ramps mechanically in ~100ms

---

### PWM Configuration

**Frequency Selection**:

20 kHz chosen for:
- Above human hearing (16-20 kHz limit)
- Reduces acoustic noise from motor
- Fast enough for smooth speed control
- Within PIC Timer2 capabilities

**Duty Cycle Mapping**:
```
User percent → PWM duty
0%   → 0 steps    (0/1000)
50%  → 500 steps  (500/1000)
100% → 1000 steps (1000/1000)

PWM period = 50µs (1/20kHz)
```

**Hardware Calculation** (done in hal_pwm):
```c
duty_10bit = (percent / 100) × 4 × (PR2 + 1)
           = (percent / 100) × 4 × 250
           = percent × 10

CCPR1L = duty_10bit >> 2        // Upper 8 bits
CCP1CON<5:4> = duty_10bit & 0x03 // Lower 2 bits
```

---

### Wiring Diagram

**Option 1: Direct Drive (small 5V fan, <100mA)**
```
Fan
  + ──→ +5V
  - ──→ RC2 (CCP1)
  
WARNING: PIC can sink ~25mA per pin
Only suitable for very small fans
```

**Option 2: MOSFET Driver (recommended for >25mA)**
```
             +12V (or fan voltage)
              │
              │
         ┌────┴────┐
         │   Fan   │
         └────┬────┘
              │
              D (Drain)
             ┌┴┐
         G ──┤ ├ MOSFET (e.g., IRLZ44N)
             └┬┘
              S (Source)
              │
             GND

RC2 (CCP1) ──→ G (Gate) through 220Ω resistor
Add 10kΩ pull-down on Gate for safety
```

**Why MOSFET?**
- PIC cannot source >25mA
- Fans typically draw 50-500mA
- MOSFET isolates PIC from fan current
- Logic-level MOSFET (Vgs threshold <3V) required

---

### Speed Control Strategy

**Linear Mapping**:
```c
fan_set_speed(percent);  // Direct percentage control
```

**Advantages**:
- Simple, intuitive API
- Predictable behavior
- Easy to implement

**Fan Response**:
- 0-20%: Fan may not start (insufficient torque)
- 20-40%: Minimum speed range (audible)
- 40-100%: Linear airflow increase

**Minimum Speed Considerations**:

Some fans won't start below ~20% duty cycle. If needed:

```c
void fan_set_speed_with_kickstart(uint8_t percent) {
    if (percent > 0 && percent < 20) {
        // Kickstart: brief full speed to overcome inertia
        fan_set_speed(100);
        __delay_ms(50);
    }
    fan_set_speed(percent);
}
```

---

### Safety Features

**1. Safe Initialization**:
```c
fan_init();  // Ensures fan starts at 0%
```

**2. Overcurrent Protection**:
- Use MOSFET with appropriate current rating
- Add fuse in series with fan supply

**3. Back-EMF Protection**:
- Add flyback diode across fan terminals (cathode to +)
- Protects against inductive kickback when fan stops

**4. Thermal Considerations**:
- Monitor fan current (optional)
- Implement thermal shutdown if temperature sensor detects overheating

---

### Application Examples

**Gradual Speed Ramp**:
```c
void fan_ramp_up(uint8_t target_speed) {
    for (uint8_t speed = 0; speed <= target_speed; speed += 5) {
        fan_set_speed(speed);
        __delay_ms(100);  // 100ms per 5% step
    }
}

void fan_ramp_down(void) {
    uint8_t current = get_current_fan_speed();  // Store in global
    for (int16_t speed = current; speed >= 0; speed -= 5) {
        fan_set_speed(speed);
        __delay_ms(100);
    }
}
```

**Temperature-Based Control**:
```c
void fan_auto_control(uint8_t temperature) {
    if (temperature < 25) {
        fan_set_speed(0);   // Off below 25°C
    } else if (temperature < 30) {
        fan_set_speed(30);  // Low speed
    } else if (temperature < 35) {
        fan_set_speed(60);  // Medium speed
    } else {
        fan_set_speed(100); // Full speed above 35°C
    }
}
```

**PWM Soft-Start** (already in hardware):
- PWM naturally provides soft-start
- No inrush current spike
- Extends fan motor lifetime

---

### Troubleshooting

| Issue | Cause | Solution |
|-------|-------|----------|
| Fan doesn't spin | Insufficient PWM duty or no power | Check fan voltage, try >30% speed |
| Fan always on | MOSFET stuck or wrong wiring | Verify gate control, check for shorts |
| Audible whine | PWM frequency too low | Verify 20 kHz setting (PR2=249) |
| Fan speed unstable | Poor power supply | Add capacitor near fan power |
| PIC resets when fan starts | Inductive kickback or power drop | Add flyback diode, improve power supply |
| Low speeds don't work | Fan minimum torque threshold | Use kickstart function or increase minimum |

---

## Integration with Main System

### Typical Usage in Main Loop

```c
void main(void) {
    // Initialize HAL
    spi_init();
    pwm_init();
    
    // Initialize actuators
    led_init();
    fan_init();
    
    // Restore from EEPROM
    uint8_t saved_r = eeprom_read(0x01);
    uint8_t saved_g = eeprom_read(0x02);
    uint8_t saved_b = eeprom_read(0x03);
    uint8_t saved_brightness = eeprom_read(0x00);
    uint8_t saved_fan = eeprom_read(0x04);
    
    if (saved_r != 0xFF) {  // Valid EEPROM data
        led_set_color(saved_r, saved_g, saved_b, saved_brightness);
        fan_set_speed(saved_fan);
    }
    
    while(1) {
        // Handle UART commands
        if (protocol_receive(&cmd, data, &len)) {
            switch(cmd) {
                case CMD_LEDS:
                    led_set_color(data[0], data[1], data[2], data[3]);
                    break;
                case CMD_FAN:
                    fan_set_speed(data[0]);
                    break;
            }
        }
    }
}
```

---

## Performance Characteristics

| Operation | Timing | Blocking | CPU Usage |
|-----------|--------|----------|-----------|
| **led_set_color()** | ~2ms | Yes | High (bit-bang SPI) |
| **fan_set_speed()** | <1µs | No | Minimal (register write) |

**Optimization Tips**:
- LED updates are expensive - only call when color changes
- Fan speed updates are cheap - safe to call frequently
- Consider caching current LED state to avoid redundant updates

---

## Power Budget

| Actuator | Idle | Active | Peak | Notes |
|----------|------|--------|------|-------|
| **LED (1 LED)** | 0mA | 20mA | 60mA | Per LED, at full white |
| **LED (10 LEDs)** | 0mA | 200mA | 600mA | Requires external supply |
| **Fan (5V, small)** | 0mA | 50mA | 100mA | Startup surge |
| **Fan (12V, large)** | 0mA | 200mA | 500mA | Requires external supply |

**Total (worst case)**: 10 LEDs + large fan = 1.1A @ startup
- PIC cannot provide this
- Use external 5V/12V power supply
- PIC controls via MOSFET/transistor

---

## Safety Guidelines

### LED Strip

1. **Voltage Matching**: Verify LED strip is 5V compatible
2. **Current Limits**: Calculate total current, use external supply if >100mA
3. **Polarity**: Reversing power destroys LEDs instantly
4. **ESD Protection**: SK9822 is ESD-sensitive, handle carefully

### Fan

1. **Voltage Rating**: Don't exceed fan's rated voltage
2. **Flyback Diode**: Always use with inductive loads
3. **Current Rating**: Size MOSFET and power supply appropriately
4. **Ventilation**: Ensure fan has unrestricted airflow
5. **Mechanical**: Secure fan to prevent vibration damage

---

## References

### Datasheets

- **SK9822**: SK9822 LED Strip Datasheet (compatible with APA102)
- **PWM Fans**: PC Fan Specifications (4-pin PWM standard)
- **MOSFET**: IRLZ44N Logic-Level N-Channel MOSFET (Infineon)

### Application Notes

- AN1549: PWM Motor Speed Control (Microchip)
- Adafruit: Controlling Motors with MOSFETs
- APA102/SK9822: Addressable LED Protocol Guide

---

[← Back to Main README](../README.md)
