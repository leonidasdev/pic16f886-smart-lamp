# sensor/ - Sensor Driver Implementations

**Purpose**: Provides high-level sensor reading functions that abstract hardware details and apply calibration/conversion formulas.

**Design Philosophy**: Each sensor module wraps HAL calls and implements domain-specific logic (unit conversions, error detection, warmup handling).

---

## Contents

| Module | Sensor | Interface | Purpose |
|--------|--------|-----------|---------|
| **sensor_temp** | LM35 | ADC AN0 | Temperature measurement (0-255°C) |
| **sensor_hum** | HIH4000 | ADC AN1 | Relative humidity (0-100% RH) |
| **sensor_noise** | Microphone | ADC AN2 | Noise level classification (3 categories) |
| **sensor_lux** | VEML7700 | I2C 0x10 | Ambient light (0-65535 lux) |
| **sensor_co2** | iAQ-Core | I2C 0x5A | CO₂ concentration (0-65535 ppm) |

---

## Module: sensor_temp (LM35 Temperature Sensor)

### Overview

LM35 precision analog temperature sensor with linear 10mV/°C output.

### Hardware Specifications

| Parameter | Value | Notes |
|-----------|-------|-------|
| **Sensor** | LM35DZ | TO-92 package |
| **Output** | 10 mV/°C | Linear response |
| **Range** | 0-100°C | Extended range: -55 to 150°C (LM35A) |
| **Accuracy** | ±0.5°C @ 25°C | Typical |
| **Supply** | 4-30V | Operating from 5V rail |
| **Pin** | RA0 (AN0) | Connected to VOUT |

### API Reference

```c
uint8_t temp_read_degC(void);
```

**Description**: Reads current temperature in degrees Celsius

**Returns**: Temperature 0-255°C (8-bit unsigned)

**Conversion Formula**:
```
Vout = 10mV/°C × T
ADC = (Vout / 5V) × 1024
T = ADC × (5000mV / 1024) / (10mV/°C)
T = ADC × 0.4887

Integer approximation:
T ≈ (ADC × 489) / 1000
```

**Example**:
```c
uint8_t temperature = temp_read_degC();
printf("Temperature: %u°C\r\n", temperature);
```

**Timing**: ~30µs (ADC conversion time)

### Wiring Diagram

```
LM35 (TO-92)
  +Vs (1) ──→ +5V
  Vout (2) ──→ RA0 (AN0)
  GND (3) ──→ GND

Optional: 0.1µF capacitor between Vout and GND for noise filtering
```

### Calibration

**Factory Calibrated**: LM35 requires no user calibration

**Verification**: 
- Room temperature (25°C): ADC ≈ 51 → temp_read_degC() ≈ 25
- Ice water (0°C): ADC ≈ 0 → temp_read_degC() ≈ 0
- Hot water (50°C): ADC ≈ 102 → temp_read_degC() ≈ 50

**Error Sources**:
- ADC reference voltage accuracy (VDD stability)
- Thermal coupling to PCB or enclosure
- Self-heating (minimal at low current)

### Troubleshooting

| Issue | Cause | Solution |
|-------|-------|----------|
| Always reads 0 | No sensor connected | Check wiring, verify 5V supply |
| Random values | Floating input | Add 0.1µF capacitor at Vout |
| Reads ~255 | Shorted to VDD | Check for solder bridges |
| Reads too high | Self-heating | Reduce sensor current, improve airflow |

---

## Module: sensor_hum (HIH4000 Humidity Sensor)

### Overview

HIH4000 analog relative humidity sensor with voltage output proportional to %RH.

### Hardware Specifications

| Parameter | Value | Notes |
|-----------|-------|-------|
| **Sensor** | HIH-4000-001 | Analog output |
| **Output** | (0.0062 × RH + 0.16) × Vsupply | Voltage output |
| **Range** | 0-100% RH | Non-condensing |
| **Accuracy** | ±3.5% RH @ 25°C | Typical |
| **Supply** | 4-5.8V | Operating from 5V rail |
| **Pin** | RA1 (AN1) | Connected to VOUT |

### API Reference

```c
uint16_t hum_read_percent(void);
```

**Description**: Reads current relative humidity percentage

**Returns**: Humidity 0-100% (16-bit for compatibility, but range limited to 0-100)

**Conversion Formula**:
```
Vout = (Vsupply) × (0.0062 × RH + 0.16)
At 5V: Vout = 0.031 × RH + 0.8V

ADC = (Vout / 5V) × 1024
RH = (Vout - 0.8V) / 0.031V
RH = ((ADC × 5V / 1024) - 0.8) / 0.031
RH ≈ (ADC - 164) / 6.35

Integer approximation:
RH ≈ (ADC - 164) * 100 / 635
```

**Example**:
```c
uint16_t humidity = hum_read_percent();
printf("Humidity: %u%%\r\n", humidity);
```

**Temperature Compensation** (if needed):
```
True RH = (RH_measured) / (1.0546 - 0.00216 × T)
Where T is temperature in °C
```

**Timing**: ~30µs (ADC conversion time)

### Wiring Diagram

```
HIH4000
  VDD (1) ──→ +5V
  GND (2) ──→ GND
  VOUT (3) ──→ RA1 (AN1)

Optional: 0.1µF capacitor between VOUT and GND
```

### Calibration

**Two-Point Calibration**:

1. **0% RH** (desiccant chamber): ADC ≈ 164
2. **75% RH** (saturated salt solution): ADC ≈ 640

**Verification Points**:
- 30% RH: ADC ≈ 350 → hum_read_percent() ≈ 30
- 60% RH: ADC ≈ 545 → hum_read_percent() ≈ 60

### Troubleshooting

| Issue | Cause | Solution |
|-------|-------|----------|
| Always reads 0 | Sensor disconnected | Check wiring |
| Reads ~100% always | Sensor saturated or damaged | Replace sensor, avoid condensation |
| Unstable readings | Sensor still equilibrating | Wait 5-10 minutes after power-on |
| Offset error | Temperature drift | Apply temperature compensation |

---

## Module: sensor_noise (Microphone Noise Level)

### Overview

Analog microphone with threshold-based classification into three noise categories.

### Hardware Specifications

| Parameter | Value | Notes |
|-----------|-------|-------|
| **Sensor** | Electret microphone | Generic analog output |
| **Output** | 0-5V | AC coupled with DC bias |
| **Pin** | RA2 (AN2) | Connected to MIC output |
| **Sampling** | 10ms interval | ISR-driven |
| **Window** | 1 second | Maximum of 100 samples |

### API Reference

```c
typedef enum {
    NOISE_LOW = 0,
    NOISE_MED = 1,
    NOISE_HIGH = 2
} noise_cat_t;

noise_cat_t noise_read_category(void);
```

**Description**: Classifies current noise level into one of three categories

**Returns**: Noise category (NOISE_LOW, NOISE_MED, or NOISE_HIGH)

**Threshold Mapping**:
```
NOISE_LOW:  ADC ≤ 400  (~1.95V)
NOISE_MED:  401-900    (~1.95-4.40V)
NOISE_HIGH: ADC > 900  (>4.40V)
```

**Sampling Strategy**:
- ISR calls `noise_read_category()` every 10ms
- Keeps maximum value over 1-second window
- Main loop reads maximum and resets

**Example**:
```c
noise_cat_t level = noise_read_category();
switch(level) {
    case NOISE_LOW:  printf("Quiet\r\n"); break;
    case NOISE_MED:  printf("Moderate\r\n"); break;
    case NOISE_HIGH: printf("Loud\r\n"); break;
}
```

### Wiring Diagram

```
Electret Microphone Module
  VCC ──→ +5V
  GND ──→ GND
  OUT ──→ RA2 (AN2)

Typical module includes:
  - Microphone capsule
  - Amplifier (LM358 or similar)
  - DC bias resistors
```

### Calibration

**NOTE**: Thresholds (400, 900) are empirical and may need adjustment based on:
- Microphone sensitivity
- Amplifier gain
- Ambient noise baseline
- Application requirements

**Calibration Procedure**:

1. **Measure baseline** (silent room): ADC should be ~512 (DC bias at 2.5V)
2. **Normal speech** (50cm distance): ADC peaks ~600-700 → adjust to NOISE_MED
3. **Loud music/clapping**: ADC peaks >900 → adjust to NOISE_HIGH

**Adjustment**:
```c
// In sensor_noise.c, modify thresholds:
#define THRESHOLD_LOW_MED  400  // Adjust based on testing
#define THRESHOLD_MED_HIGH 900  // Adjust based on testing
```

### Troubleshooting

| Issue | Cause | Solution |
|-------|-------|----------|
| Always NOISE_HIGH | Module saturated or no bias | Check module power, verify DC offset |
| Always NOISE_LOW | Module disconnected or muted | Verify wiring, check amplifier enable pin |
| No response to sound | AC coupling cap failed | Replace coupling capacitor |
| Erratic readings | Electrical noise interference | Add ferrite bead, shielded cable |

---

## Module: sensor_lux (VEML7700 Light Sensor)

### Overview

VEML7700 digital ambient light sensor with I2C interface and 16-bit resolution.

### Hardware Specifications

| Parameter | Value | Notes |
|-----------|-------|-------|
| **Sensor** | VEML7700 | Vishay semiconductor |
| **Interface** | I2C | Address: 0x10 (7-bit) |
| **Range** | 0-120,000 lux | Configurable gain |
| **Resolution** | 16 bits | 0-65535 counts |
| **Supply** | 2.5-3.6V | Requires level shifting for 5V systems |
| **Pins** | RC3 (SCL), RC4 (SDA) | 4.7kΩ pull-ups required |

### API Reference

```c
uint16_t lux_read(void);
```

**Description**: Reads ambient light level from VEML7700

**Returns**: 
- Light level 0-65535 (raw counts)
- 0xFFFF on I2C communication error

**Register Access**:
```
Register 0x04 (ALS): Ambient Light Sensor data
Read: 2 bytes (LSB first, MSB second)
```

**I2C Transaction**:
```
START
WRITE 0x10 (device address)  → ACK
WRITE 0x04 (register ALS)    → ACK
RESTART
WRITE 0x11 (read mode)       → ACK
READ LSB                     → ACK
READ MSB                     → NACK
STOP
```

**Conversion to Lux** (typical, gain=1, integration 100ms):
```
Lux = raw_counts × 0.0576
```

**Example**:
```c
uint16_t lux = lux_read();
if (lux == 0xFFFF) {
    printf("Sensor error\r\n");
} else {
    printf("Light: %u counts\r\n", lux);
}
```

**Timing**: ~200µs (I2C transaction @ 100kHz)

### Wiring Diagram

```
VEML7700 (assuming 3.3V module with level shifting)
  VDD ──→ +3.3V (or +5V if tolerant)
  GND ──→ GND
  SCL ──→ RC3 (with 4.7kΩ pull-up to VDD)
  SDA ──→ RC4 (with 4.7kΩ pull-up to VDD)

WARNING: Check module voltage compatibility
Some modules have built-in level shifters for 5V
```

### Configuration

**Current Implementation**: Uses default power-on configuration
- Gain: 1x
- Integration time: 100ms
- Power: Auto mode

**Optional Configuration** (add to `lux_init()` if needed):
```c
void lux_init(void) {
    hal_i2c_start();
    hal_i2c_write((VEML7700_ADDR << 1) | 0);
    hal_i2c_write(0x00);  // Configuration register
    hal_i2c_write(0x00);  // LSB: gain=1, IT=100ms
    hal_i2c_write(0x00);  // MSB
    hal_i2c_stop();
}
```

### Calibration

**Factory Calibrated**: No user calibration typically required

**Verification**:
- **Dark room**: Should read <10 counts
- **Office lighting**: ~300-500 counts (typical fluorescent)
- **Direct sunlight**: >50,000 counts (may saturate at 65535)

### Troubleshooting

| Issue | Cause | Solution |
|-------|-------|----------|
| Returns 0xFFFF | I2C communication failure | Check pull-ups, verify address 0x10 |
| Always reads 0 | Sensor covered or damaged | Remove obstruction, test with light |
| Saturates at 65535 | Too bright for current gain | Reduce gain or use auto-gain mode |
| Unstable readings | Flickering light source | Increase integration time |

---

## Module: sensor_co2 (iAQ-Core CO₂ Sensor)

### Overview

iAQ-Core indoor air quality sensor with CO₂ equivalent and TVOC measurements via I2C.

### Hardware Specifications

| Parameter | Value | Notes |
|-----------|-------|-------|
| **Sensor** | iAQ-Core C | ams AG |
| **Interface** | I2C | Address: 0x5A (7-bit) |
| **CO₂ Range** | 450-2000 ppm | Equivalent CO₂ (eCO₂) |
| **Warmup** | 5 minutes | Required for stable readings |
| **Supply** | 3.3V typical | Check module specifications |
| **Pins** | RC3 (SCL), RC4 (SDA) | 4.7kΩ pull-ups required |

### API Reference

```c
void co2_init(void);
void co2_update_warmup(void);
uint8_t co2_is_ready(void);
uint16_t co2_read_ppm(void);
```

**co2_init()**

Initializes warmup counter and sensor state.

**co2_update_warmup()**

Increments warmup counter (call every 5 seconds).

After 60 calls (300 seconds), sensor is marked as ready.

**co2_is_ready()**

Returns: `1` if warmed up, `0` if still warming

**co2_read_ppm()**

Reads CO₂ equivalent concentration in parts per million.

**Returns**: 
- CO₂ ppm (450-2000 typical range)
- 0xFFFF if sensor not ready or I2C error

**Data Frame** (9 bytes):
```
Byte 0: Status
Byte 1-2: CO₂ prediction (ppm, big-endian)
Byte 3-4: TVOC prediction (ppb, big-endian)
Byte 5-8: Resistance values (not used in this implementation)
```

**I2C Transaction**:
```
START
WRITE 0xB5 (0x5A << 1 | 1, read mode) → ACK
READ 9 bytes (ACK all except last)
STOP
```

**Example**:
```c
co2_init();

// In main loop (every 5 seconds):
co2_update_warmup();

if (co2_is_ready()) {
    uint16_t co2 = co2_read_ppm();
    if (co2 != 0xFFFF) {
        printf("CO2: %u ppm\r\n", co2);
    }
}
```

**Timing**: 
- Warmup: 300 seconds (5 minutes)
- Read: ~250µs (I2C transaction @ 100kHz)

### Wiring Diagram

```
iAQ-Core
  VDD ──→ +3.3V (check module specs)
  GND ──→ GND
  SCL ──→ RC3 (with 4.7kΩ pull-up)
  SDA ──→ RC4 (with 4.7kΩ pull-up)

WARNING: Verify supply voltage (3.3V or 5V tolerant)
```

### Warmup Behavior

**Why 5 Minutes?**
- Metal oxide sensor requires thermal stabilization
- Algorithm baseline calibration needs time
- Readings before warmup are unreliable

**During Warmup**:
- `co2_read_ppm()` returns 0xFFFF
- Status byte may indicate "warming up"
- Do not use readings for decisions

**After Warmup**:
- Readings stabilize within ±50 ppm
- Sensor continuously self-calibrates to 400 ppm baseline (outdoor air)

### Calibration

**Automatic Baseline Calibration**:
- Sensor assumes 400 ppm exposure every 7 days (outdoor air)
- For continuous indoor use, manual calibration may be needed

**Manual Calibration** (if sensor always reads high):
1. Expose to fresh outdoor air for 20 minutes
2. Sensor will auto-calibrate to 400 ppm baseline
3. Return to indoor environment

**Typical Values**:
- Outdoor air: 400 ppm
- Well-ventilated room: 600-800 ppm
- Occupied room: 1000-1500 ppm
- Poorly ventilated: >2000 ppm (unhealthy)

### Troubleshooting

| Issue | Cause | Solution |
|-------|-------|----------|
| Always returns 0xFFFF | Warmup not complete | Wait full 5 minutes, call co2_update_warmup() |
| I2C error on read | Wrong address or wiring | Verify address 0x5A, check pull-ups |
| Reads always high | Needs baseline calibration | Expose to fresh air for 20 minutes |
| Unstable readings | Sensor still warming | Wait longer, ensure stable temperature |
| No I2C ACK | Sensor not powered | Check 3.3V supply, verify module compatibility |

---

## Integration with Main Loop

### Typical Usage Pattern

```c
void main(void) {
    // Initialize HAL
    adc_init();
    hal_i2c_init();
    
    // Initialize sensors
    co2_init();
    
    // Restore EEPROM config
    // ...
    
    while(1) {
        // Every 1 second: Noise
        if (scheduler_noise_ready()) {
            noise_cat_t noise = noise_read_category();
            protocol_send(CMD_NOISE, &noise, 1);
        }
        
        // Every 5 seconds: Environmental sensors
        if (scheduler_env_ready()) {
            co2_update_warmup();  // Increment warmup counter
            
            uint8_t temp = temp_read_degC();
            uint16_t hum = hum_read_percent();
            uint16_t lux = lux_read();
            uint16_t co2 = co2_read_ppm();  // Returns 0xFFFF if not ready
            
            // Send via UART protocol
            protocol_send(CMD_TEMP, &temp, 1);
            protocol_send(CMD_HUM, &hum, 2);
            protocol_send(CMD_LUX, &lux, 2);
            protocol_send(CMD_CO2, &co2, 2);
        }
    }
}
```

---

## Performance Characteristics

| Sensor | Read Time | Blocking | Update Rate | Notes |
|--------|-----------|----------|-------------|-------|
| **sensor_temp** | ~30µs | Yes | 5 seconds | ADC conversion |
| **sensor_hum** | ~30µs | Yes | 5 seconds | ADC conversion |
| **sensor_noise** | ~30µs | Yes | 1 second | Max of 100 samples |
| **sensor_lux** | ~200µs | Yes | 5 seconds | I2C transaction |
| **sensor_co2** | ~250µs | Yes | 5 seconds | I2C transaction + warmup |

**Total Read Time** (all sensors): <600µs
- ADC sensors: 3 × 30µs = 90µs
- I2C sensors: 200µs + 250µs = 450µs
- Overhead: ~60µs

**CPU Impact**: <1% @ 20MHz with 5-second update rate

---

## Sensor Specifications Summary

| Sensor | Interface | Range | Accuracy | Power | Response Time |
|--------|-----------|-------|----------|-------|---------------|
| **LM35** | ADC | 0-255°C | ±0.5°C | <100µA | <1s |
| **HIH4000** | ADC | 0-100% RH | ±3.5% RH | 200µA | 5s (63% step) |
| **Microphone** | ADC | 3 categories | N/A | <5mA | 10ms |
| **VEML7700** | I2C | 0-120k lux | ±10% | 120µA | 100ms |
| **iAQ-Core** | I2C | 450-2000 ppm | ±15% ±100ppm | 33mA | 300s warmup |

---

## Error Handling

### Return Value Conventions

- **ADC sensors**: Return 0 on error (unlikely with internal ADC)
- **I2C sensors**: Return 0xFFFF on communication failure or not ready
- **Noise sensor**: Always returns valid category (ADC cannot fail)

### Error Detection Example

```c
uint16_t lux = lux_read();
if (lux == 0xFFFF) {
    // I2C error or sensor disconnected
    // Option 1: Skip transmission
    // Option 2: Send error code to host
    // Option 3: Retry after delay
}

uint16_t co2 = co2_read_ppm();
if (co2 == 0xFFFF) {
    if (!co2_is_ready()) {
        // Still warming up - expected behavior
    } else {
        // Communication error - check I2C bus
    }
}
```

---

## References

### Datasheets

- **LM35**: LM35 Precision Centigrade Temperature Sensors (Texas Instruments)
- **HIH4000**: HIH-4000 Series Humidity Sensor (Honeywell)
- **VEML7700**: VEML7700 Ambient Light Sensor (Vishay)
- **iAQ-Core**: iAQ-Core Indoor Air Quality Sensor (ams AG)

### Application Notes

- AN-1148: LM35 Layout and Filtering (TI)
- HPC004: HIH-4000 Series Application Guide (Honeywell)
- VEML7700 Application Note (Vishay)
- iAQ-Core Quick Start Guide (ams AG)

---

[← Back to Main README](../README.md)
