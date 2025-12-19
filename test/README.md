# test/ - Hardware and Integration Test Programs

**Purpose**: Provides standalone test programs for validating individual hardware components and system integration.

**Test Philosophy**: Each test is a complete program targeting one subsystem, enabling isolated verification before full system integration.

---

## Test Inventory

| Test | Target Component | Hardware | Output | Duration |
|------|------------------|----------|--------|----------|
| **test_fan** | PWM fan control | Fan + CCP1 | Visual (speed ramp) | Continuous |
| **test_led** | SK9822 LED strip | 10 LEDs + SPI | Visual (color cycle) | Continuous |
| **test_command** | UART protocol | UART + Fan + LED | Response to commands | Continuous |
| **test_co2** | iAQ-Core CO2 | I2C sensor + UART | UART (ppm every 1s) | Continuous |
| **test_lux** | VEML7700 lux | I2C sensor + UART | UART (lux every 500ms) | Continuous |
| **test_temperature** | LM35 temperature | ADC + UART | UART (°C every 500ms) | Continuous |
| **test_noise** | Microphone noise | ADC + UART | UART (level every 500ms) | Continuous |
| **test_humidity** | HIH-4000 humidity | ADC + UART | UART (% every 1s) | Continuous |
| **test_eeprom** | Internal EEPROM | None | UART (read/write test) | Single-shot |

---

## General Test Requirements

**Common Hardware**:
- PIC16F886 microcontroller
- 20 MHz crystal oscillator (HS mode)
- 5V power supply
- ICSP programmer (PICkit 3/4, ICD3, etc.)

**Programming Configuration**:
- Compiler: XC8 (MPLAB X IDE)
- Configuration bits: See individual test headers
- Clock: FOSC = HS (20 MHz external)

**UART Terminal Setup** (for tests with serial output):
- Baudrate: 9600 bps
- Format: 8N1 (8 data bits, no parity, 1 stop bit)
- Terminal: RealTerm, Tera Term, PuTTY, or similar
- Pins: RC6 (TX), RC7 (RX)

---

## Test 1: test_fan - Fan PWM Control

### Purpose
Validates PWM generation and fan control using automatic speed ramping (0% to 80% and back).

### Hardware Requirements
- **Fan**: 12V DC fan with MOSFET driver
- **PWM Output**: RC2 (CCP1/PWM)
- **Driver Circuit**: N-channel MOSFET (e.g., IRLZ44N) with flyback diode
- **Power**: Separate 12V supply for fan (GND common with PIC)

### Wiring
```
PIC RC2 (CCP1) ──────────┬─── MOSFET Gate (1kΩ resistor)
                         │
                      10kΩ pull-down
                         │
                        GND

MOSFET Source ────── GND (common)
MOSFET Drain  ────── Fan (-) terminal
Fan (+) ──────────── +12V supply
Flyback diode ────── Across fan (cathode to +12V)
```

### Expected Behavior
- Fan speed ramps from 0% to 80% in 8 seconds (10% every second)
- Speed ramps back down to 0% in 8 seconds
- Cycle repeats continuously
- Smooth speed transitions (100ms steps)

### PWM Specifications
- Frequency: 20 kHz (inaudible, efficient)
- Timer2 prescaler: 1:1
- PR2 = 249 (20 MHz / 4 / 1 / 20000 - 1)
- Duty cycle: CCPR1L controls 0-200 (0% to 80%)

### Timing Analysis
- Timer0 tick: 10ms (prescaler 1:256, preload 61)
- Speed change: Every 100ms (10 ticks)
- Full ramp: 8 steps × 100ms = 800ms per direction

### Troubleshooting

| Problem | Cause | Solution |
|---------|-------|----------|
| Fan doesn't run | No PWM signal | Check RC2 pin, verify Timer2 enabled |
| Fan at constant speed | Timer0 not running | Verify Timer0 interrupt enabled |
| Fan stutters | Wrong PWM frequency | Verify PR2=249, Fosc=20MHz |
| MOSFET overheats | No flyback diode | Add 1N4007 diode across fan |
| Speed too fast/slow | Wrong ramp timing | Adjust TIMER_MAX_ITERATIONS |

### Key Code Sections
```c
fan_init();                    // Initialize PWM at 20kHz
fan_set_speed(speed_percent);  // 0-100% duty cycle
init_T0();                     // Timer0 for 10ms ticks
```

---

## Test 2: test_led - SK9822 LED Strip

### Purpose
Validates SPI communication and LED control by cycling through 4 colors with varying brightness.

### Hardware Requirements
- **LED Strip**: SK9822 compatible (10 LEDs minimum)
- **SPI Pins**: RC0 (CLK), RC1 (SDO/MOSI)
- **Power**: 5V supply capable of 600mA (10 LEDs @ 60mA max each)
- **Level Shifter**: Optional 3.3V → 5V if LEDs require it

### Wiring
```
PIC RC0 (SPI CLK) ──→ LED Strip CLK
PIC RC1 (SPI SDO) ──→ LED Strip DATA
GND ───────────────→ LED Strip GND
+5V ───────────────→ LED Strip VDD (separate supply recommended)
```

### Expected Behavior
1. **Red (medium)**: 255, 0, 0 at 38% brightness (2 seconds)
2. **Green (full)**: 0, 255, 0 at 100% brightness (2 seconds)
3. **Blue (low)**: 0, 0, 255 at 19% brightness (2 seconds)
4. **White (medium)**: 255, 255, 255 at 50% brightness (2 seconds)
5. Repeats cycle

### SK9822 Protocol
```
Start Frame:  32 bits of 0x00 (4 bytes)
LED Frames:   [111xxxxx][Blue][Green][Red] × N LEDs
              xxxxx = brightness (0-31)
End Frame:    32 bits of 0xFF (4 bytes)
```

**Example Frame** (1 red LED, full brightness):
```
00 00 00 00  FF 00 00 FF  FF FF FF FF
│           │            │
Start       LED Data     End
```

### SPI Configuration
- Mode: Bit-bang (software SPI in actuator_led.c)
- Clock: ~2 MHz (software-controlled via hal_spi.h)
- Bit order: MSB first
- Clock polarity: Idle low

### Current Consumption
```
Per LED: Up to 60mA @ full white (255, 255, 255) and brightness 31
10 LEDs: Up to 600mA maximum
Typical: 200-300mA for mixed colors at medium brightness
```

### Troubleshooting

| Problem | Cause | Solution |
|---------|-------|----------|
| No LEDs lit | No power/wrong wiring | Verify VDD, GND, CLK, DATA connections |
| Wrong colors | Swapped RGB order | Check LED datasheet (SK9822 is BGR order) |
| Flickering | Poor power supply | Use capacitor (470µF) near LED strip |
| First LED wrong | Missing start frame | Verify led_init() called |
| Dim LEDs | Low brightness value | Increase brightness parameter (0-31) |

### Key Code Sections
```c
spi_init();                              // Initialize SPI (bit-bang)
led_set_color(255, 0, 0, 12);            // Red at 38% brightness
led_set_color(0, 255, 0, 31);            // Green at 100%
__delay_us(2000000);                     // 2 second delay
```

---

## Test 3: test_command - UART Command Reception

### Purpose
Validates UART protocol parser and actuator control via received commands (fan and LED).

### Hardware Requirements
- **UART**: RC6 (TX), RC7 (RX) at 9600 baud
- **USB-UART**: CH340, FTDI, CP2102, or similar
- **Fan**: Same as test_fan
- **LEDs**: Same as test_led

### Wiring
- See test_fan for fan circuit
- See test_led for LED strip
- UART: RC6 → USB-UART RX, RC7 → USB-UART TX, GND common

### Supported Commands

**CMD_FAN (0x04)**: Fan speed control
```
Frame: AA 04 04 <speed> <CRC> 00
speed: 0-100 (percentage)
Example: AA 04 04 32 XX 00  → 50% speed
```

**CMD_LEDS (0x05)**: LED color/brightness
```
Frame: AA 07 05 <R> <G> <B> <Bright> <CRC> 00
R, G, B: 0-255
Bright: 0-31
Example: AA 07 05 FF 00 00 1F XX 00  → Red, full brightness
```

### Testing with Python Script
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

ser = serial.Serial('COM3', 9600, timeout=1)

# Test fan at 50%
frame = [0xAA, 0x04, 0x04, 50]
crc = crc8([0x04, 0x04, 50])
ser.write(bytes(frame + [crc, 0x00]))

# Test LED green
frame = [0xAA, 0x07, 0x05, 0, 255, 0, 31]
crc = crc8([0x07, 0x05, 0, 255, 0, 31])
ser.write(bytes(frame + [crc, 0x00]))
```

### Expected Behavior
- PIC continuously monitors UART in ISR
- Valid frames trigger immediate actuator response
- Invalid frames (wrong CRC, bad format) ignored
- No acknowledgment sent (one-way protocol in this test)

### Protocol State Machine
```
HEADER → LENGTH → COMMAND → DATA → CRC_LO → CRC_HI
  0xAA    3-31      0x04/05   varies   calc    0x00
```

### Troubleshooting

| Problem | Cause | Solution |
|---------|-------|----------|
| No response | Wrong baudrate | Verify both at 9600 baud |
| Commands ignored | CRC error | Check CRC-8-CCITT implementation |
| Partial response | Frame timeout | Send complete frame quickly |
| Fan/LED not working | Hardware issue | Test with test_fan/test_led first |
| Garbled data | Electrical noise | Shorter USB cable, ferrite beads |

### Key Code Sections
```c
protocol_init(9600);              // Init UART + protocol
protocol_feed_byte(RCREG);        // In ISR: feed byte to parser
if (received_command.type == CMD_FAN) {
    fan_set_speed(received_command.fan_speed);
}
```

---

## Test 4: test_co2 - iAQ-Core CO₂ Sensor

### Purpose
Validates I2C communication and CO₂ concentration measurement with warmup detection.

### Hardware Requirements
- **Sensor**: iAQ-Core (ScioSense CCS811 variant)
- **I2C**: RC3 (SCL), RC4 (SDA) at 100 kHz
- **Pull-ups**: 4.7kΩ on SCL and SDA to +5V
- **UART**: For data output (9600 baud)

### Wiring
```
PIC RC3 (SCL) ──┬── 4.7kΩ ──→ +5V
                └── Sensor SCL

PIC RC4 (SDA) ──┬── 4.7kΩ ──→ +5V
                └── Sensor SDA

Sensor VDD ───────────────→ +5V
Sensor GND ───────────────→ GND
```

### Sensor Specifications
- I2C Address: 0x5A (fixed)
- Warmup time: 5 minutes for stable readings
- Measurement rate: 1 Hz (this test)
- Range: 450-2000 ppm CO₂ (typical)

### Expected Output (UART)
```
--- Starting CO2 test ---
CO2: 65535 ppm (Warmup)  ← First 5 minutes
CO2: 65535 ppm (Warmup)
...
CO2: 1234 ppm            ← After warmup
CO2: 1242 ppm
CO2: 1238 ppm
```

**Special Values**:
- `65535 ppm`: Sensor in warmup mode (< 5 minutes)
- `0 ppm`: I2C communication error
- `450-2000 ppm`: Normal indoor CO₂ levels
- `>2000 ppm`: Poor ventilation warning

### I2C Protocol
```
1. Start condition
2. Send 0xB4 (0x5A << 1, write bit)
3. Send ACK
4. Repeated start
5. Send 0xB5 (0x5A << 1, read bit)
6. Read 9 bytes:
   [0-1]: CO₂ (ppm, big-endian)
   [2]:   Status (0x00=OK, 0x10=Warmup, 0x80=Error)
   [3-8]: VOC and resistance data
7. Stop condition
```

### Reading Interpretation
```c
uint16_t co2 = (data[0] << 8) | data[1];
uint8_t status = data[2];

if (status == 0x10) {
    // Warmup mode (return 65535)
} else if (status == 0x00) {
    // Valid reading
}
```

### Troubleshooting

| Problem | Cause | Solution |
|---------|-------|----------|
| Always 0 ppm | I2C not connected | Check wiring, pull-ups, sensor power |
| Always 65535 ppm | Sensor warming up | Wait 5 minutes after power-on |
| Unstable readings | Insufficient warmup | Wait longer, ensure stable power |
| I2C timeout | Wrong SCL/SDA pins | Verify RC3=SCL, RC4=SDA |
| Erratic values | Loose connections | Check breadboard contacts |

### Key Code Sections
```c
hal_i2c_init();                    // I2C at 100 kHz
co2_init();                        // Reset warmup counter
uint16_t co2_ppm = co2_read_ppm(); // Returns ppm or 65535
if (co2_ppm == 65535) {
    printf("Warmup\n");
}
```

---

## Test 5: test_lux - VEML7700 Light Sensor

### Purpose
Validates I2C light sensor and lux conversion.

### Hardware Requirements
- **Sensor**: VEML7700 ambient light sensor
- **I2C**: RC3 (SCL), RC4 (SDA) at 100 kHz
- **Pull-ups**: 4.7kΩ on SCL and SDA
- **UART**: For data output

### Wiring
Same as test_co2 but with VEML7700 at address 0x10.

### Expected Output
```
--- Starting lux test ---
Lux: 234      ← Indoor office lighting
Lux: 15000    ← Direct sunlight through window
Lux: 5        ← Dark room
Lux: 10000    ← Outdoor overcast
```

### Sensor Specifications
- I2C Address: 0x10 (fixed)
- Range: 0 to ~120,000 lux
- Resolution: 16-bit (0-65535 counts)
- Conversion: Lux = counts × 0.0576 (gain 1/8, 100ms integration)

### I2C Register Access
```
Config Register (0x00): [Gain][Integration Time][Power]
ALS Register (0x04):    [ALS_MSB][ALS_LSB] (read only)
```

### Troubleshooting

| Problem | Cause | Solution |
|---------|-------|----------|
| Always 0 lux | Sensor not responding | Check I2C address 0x10 |
| Saturated (65535) | Too bright / wrong gain | Reduce gain or integration time |
| Noisy readings | Electrical interference | Add decoupling capacitor (100nF) |
| Slow response | Long integration time | Reduce to 100ms or 50ms |

### Key Code Sections
```c
hal_i2c_init();
uint16_t lux = lux_read();  // Returns lux value (0-65535)
printf("Lux: %u\n", lux);
```

---

## Test 6: test_temperature - LM35 Temperature Sensor

### Purpose
Validates ADC configuration and analog temperature measurement.

### Hardware Requirements
- **Sensor**: LM35 precision temperature sensor
- **ADC**: RA0 (AN0)
- **UART**: For data output

### Wiring
```
LM35 Vout ──→ PIC RA0 (AN0)
LM35 VDD  ──→ +5V
LM35 GND  ──→ GND
```

### Sensor Specifications
- Output: 10 mV/°C linear
- Range: 0°C to +100°C (can extend to 255°C)
- Accuracy: ±0.5°C @ 25°C
- Current: < 60 µA

### Expected Output
```
--- Temp Test ---
ADC: 512, Temp: 25.0C   ← Room temperature
ADC: 614, Temp: 30.0C   ← Hand near sensor
ADC: 410, Temp: 20.0C   ← Air conditioning
```

### Conversion Formula
```
ADC: 10-bit (0-1023)
Vref: 5.0V
Temperature: 0-100°C

Voltage (mV) = ADC × (5000 / 1024)
Temperature (°C) = Voltage / 10

Simplified: °C = ADC × 0.4887
```

### ADC Configuration
- Clock: Fosc/32 = 1.6 µs TAD (valid: 1.6 - 6.4 µs)
- Channel: AN0 (RA0)
- Justification: Right (ADCON1bits.ADFM=1)
- Acquisition time: ~20 µs (12 TAD)

### Troubleshooting

| Problem | Cause | Solution |
|---------|-------|----------|
| Always 0°C | ADC not configured | Check ADCON0, ADCON1 settings |
| Wrong temperature | Vref mismatch | Verify Vdd = 5.0V, not 3.3V |
| Noisy readings | No decoupling | Add 100nF cap near LM35 |
| Out of range | Wrong sensor | Verify LM35 (not LM35D or LM34) |

### Key Code Sections
```c
hal_adc_init(ADC_FOSC_32);          // TAD = 1.6 µs
hal_adc_select_channel(0);          // Select AN0
uint16_t adc = hal_adc_read();      // 0-1023
uint8_t temp = temp_read_degC();    // Convert to °C
```

---

## Test 7: test_noise - Microphone Noise Level

### Purpose
Validates ADC noise categorization (LOW/MEDIUM/HIGH).

### Hardware Requirements
- **Microphone**: Electret with amplifier (analog output)
- **ADC**: RA2 (AN2)
- **UART**: For data output

### Wiring
```
Mic Vout ──→ PIC RA2 (AN2)
Mic VDD  ──→ +5V
Mic GND  ──→ GND
```

### Expected Output
```
--- Noise Test ---
ADC: 250, Nivel: BAJO    ← Quiet room
ADC: 600, Nivel: MEDIO   ← Normal conversation
ADC: 950, Nivel: ALTO    ← Loud music
```

### Classification Thresholds
```c
ADC ≤ 400  → NOISE_LOW    (~1.95V)
ADC ≤ 900  → NOISE_MED    (~4.40V)
ADC > 900  → NOISE_HIGH   (>4.40V)
```

### Calibration Procedure
1. Run test in quiet environment
2. Note baseline ADC value (e.g., 200)
3. Generate noise (clap, talk, music)
4. Adjust thresholds in sensor_noise.h if needed

### Troubleshooting

| Problem | Cause | Solution |
|---------|-------|----------|
| Always LOW | Mic not connected | Check wiring, verify Vdd |
| Always HIGH | Mic saturated | Reduce microphone gain |
| No variation | AC coupling missing | Check mic circuit has DC bias |
| Inverted | Wrong threshold order | Swap LOW/HIGH thresholds |

### Key Code Sections
```c
hal_adc_init(ADC_FOSC_32);
hal_adc_select_channel(2);           // AN2
noise_cat_t level = noise_read_category();
// Returns: NOISE_LOW, NOISE_MED, NOISE_HIGH
```

---

## Test 8: test_humidity - HIH-4000 Humidity Sensor

### Purpose
Validates ADC humidity measurement and conversion.

### Hardware Requirements
- **Sensor**: HIH-4000 (or HIH-4030) capacitive humidity sensor
- **ADC**: RA1 (AN1)
- **UART**: For data output

### Wiring
```
HIH-4000 Vout ──→ PIC RA1 (AN1)
HIH-4000 VDD  ──→ +5V
HIH-4000 GND  ──→ GND
```

### Sensor Specifications
- Output: 0.8V to 3.9V (0% to 100% RH @ 5V supply)
- Accuracy: ±3.5% RH typical
- Response time: 5 seconds (63% of final value)

### Expected Output
```
--- Humidity Test ---
ADC: 400, Hum: 40%    ← Dry indoor air
ADC: 600, Hum: 60%    ← Comfortable humidity
ADC: 800, Hum: 80%    ← High humidity
```

### Conversion Formula
```
Datasheet: Vout = Vsupply × (0.0062 × RH + 0.16)
At 5V:     Vout = 0.8V + (RH × 0.031V)

ADC = Vout × 1023 / 5V = Vout × 204.6

Solving for RH:
RH% = (Vout - 0.8) / 0.031
RH% = (ADC / 204.6 - 0.8) / 0.031
RH% ≈ (ADC - 164) / 6.34
```

### Calibration
For better accuracy, measure actual Vout at known RH:
```
Salt method: 75% RH using saturated NaCl solution
Measure ADC value, adjust formula in sensor_hum.c
```

### Troubleshooting

| Problem | Cause | Solution |
|---------|-------|----------|
| Always 0% | Sensor not powered | Check VDD = 5V |
| Always 100% | Saturated / wrong formula | Verify conversion constants |
| Slow response | Normal behavior | Wait 5-10 seconds for stabilization |
| Negative values | ADC < offset | Clamp to 0 in software |

### Key Code Sections
```c
hal_adc_init(ADC_FOSC_32);
hal_adc_select_channel(1);        // AN1
uint16_t hum_percent = hum_read_percent();
printf("Hum: %u%%\n", hum_percent);
```

---

## Test 9: test_eeprom - Internal EEPROM

### Purpose
Validates internal EEPROM read/write functionality.

### Hardware Requirements
- No external hardware required (internal EEPROM)
- UART for output

### EEPROM Specifications
- Size: 256 bytes (addresses 0x00 to 0xFF)
- Endurance: 100,000 erase/write cycles minimum
- Data retention: 40 years @ 25°C
- Write time: ~5 ms typical

### Test Sequence
1. Read first 16 bytes (show initial contents)
2. Write test pattern to addresses 0x10-0x13:
   - 0x10 ← 0xAA
   - 0x11 ← 0xBB
   - 0x12 ← 0xCC
   - 0x13 ← 0xDD
3. Read back and verify
4. Display results via UART

### Expected Output
```
--- EEPROM Test ---
Initial read [0x00-0x0F]:
00: FF FF FF FF FF FF FF FF
08: FF FF FF FF FF FF FF FF

Writing test pattern...
0x10 <- 0xAA
0x11 <- 0xBB
0x12 <- 0xCC
0x13 <- 0xDD

Verification read:
0x10: 0xAA [OK]
0x11: 0xBB [OK]
0x12: 0xCC [OK]
0x13: 0xDD [OK]

Test complete!
```

### Write Sequence (Hardware)
```c
1. EEADR = address;
2. EEDATA = data;
3. EECON1bits.EEPGD = 0;  // Select EEPROM
4. EECON1bits.WREN = 1;   // Enable writes
5. INTCONbits.GIE = 0;    // Disable interrupts
6. EECON2 = 0x55;         // Required sequence
7. EECON2 = 0xAA;
8. EECON1bits.WR = 1;     // Initiate write
9. while (EECON1bits.WR); // Wait for completion
10. INTCONbits.GIE = 1;   // Re-enable interrupts
11. EECON1bits.WREN = 0;  // Disable writes
```

### Troubleshooting

| Problem | Cause | Solution |
|---------|-------|----------|
| Write fails | Wrong sequence | Verify 0x55, 0xAA unlock sequence |
| Read returns 0xFF | Unprogrammed location | Normal for unused EEPROM |
| Write timeout | Polling WR bit | Add timeout counter |
| Data corruption | GIE not disabled | Always disable interrupts during write |

### Key Code Sections
```c
eeprom_write(0x10, 0xAA);     // Write byte to address
uint8_t data = eeprom_read(0x10);  // Read byte
```

---

## Compilation and Programming

### Build Commands (MPLAB X)
```bash
# Clean build
make clean
make -f nbproject/Makefile-default.mk SUBPROJECTS= .build-conf

# Or use GUI: Production → Build Main Project
```

### Programming (PICkit)
```bash
# Command line (replace paths):
"C:\Program Files\Microchip\MPLABX\v6.xx\mplab_platform\bin\prjMakefilesGenerator.bat" ...

# Or use GUI: Production → Make and Program Device Main Project
```

### Compiler Optimizations
For consistent test behavior, use optimization level 1:
```
XC8 Global Options → Optimizations → Level 1
```

---

## Integration Testing Strategy

**Recommended Test Order**:

1. **test_eeprom**: No external hardware, validates basic system
2. **test_temperature**: Simple ADC test
3. **test_humidity**: ADC with formula validation
4. **test_noise**: ADC with classification
5. **test_lux**: I2C communication basics
6. **test_co2**: I2C with warmup logic
7. **test_fan**: PWM output validation
8. **test_led**: SPI bit-bang protocol
9. **test_command**: Full protocol integration

**After All Tests Pass**:
- Proceed to main.c integration
- Run full system with all sensors + actuators
- Verify UART protocol with host computer

---

## Common Issues Across All Tests

### Hardware Issues

| Symptom | Likely Cause | Solution |
|---------|--------------|----------|
| No serial output | Wrong UART pins | Verify RC6=TX, RC7=RX |
| Garbled UART | Wrong baudrate | Set terminal to 9600 baud |
| Intermittent behavior | Poor power supply | Use stable 5V regulator |
| Sensors unresponsive | Missing pull-ups | Add 4.7kΩ on I2C lines |
| Random resets | Brownout | Add decoupling caps (100nF, 10µF) |

### Software Issues

| Symptom | Likely Cause | Solution |
|---------|--------------|----------|
| Compilation errors | Missing includes | Verify all .h files in path |
| Link errors | Missing .c files | Add to project sources |
| Config bits wrong | Device mismatch | Select PIC16F886 in project |
| Program won't start | Oscillator failure | Verify HS mode, 20MHz crystal |
| Watchdog resets | WDT enabled | Ensure WDTE=OFF in config |

### Measurement Issues

| Symptom | Likely Cause | Solution |
|---------|--------------|----------|
| ADC always 0 | Wrong Vref | Verify VCFG=00 (Vdd/Vss) |
| ADC noisy | No filtering | Add 100nF cap on analog input |
| I2C timeout | Wrong clock | Verify 100kHz, check with scope |
| Timing drift | Clock inaccurate | Verify 20MHz crystal, not ceramic |

---

## Test Results Documentation

**Template for Test Reports**:

```
Test: test_<name>
Date: YYYY-MM-DD
Tester: [Name]
Hardware: [Board revision, sensor model]

Results:
- Compilation: [PASS/FAIL]
- Programming: [PASS/FAIL]
- Basic function: [PASS/FAIL]
- Accuracy: [Value vs expected]
- Notes: [Any observations]

Issues:
- [List any problems encountered]

Conclusion: [PASS/FAIL with summary]
```

---

## Performance Benchmarks

| Test | Flash Usage | RAM Usage | CPU Load | Test Duration |
|------|-------------|-----------|----------|---------------|
| test_fan | ~1.2 KB | 12 bytes | 2% | Continuous |
| test_led | ~1.5 KB | 45 bytes | 8% | Continuous |
| test_command | ~2.8 KB | 62 bytes | 15% | Continuous |
| test_co2 | ~2.1 KB | 28 bytes | 5% | Continuous |
| test_lux | ~1.8 KB | 22 bytes | 4% | Continuous |
| test_temperature | ~1.6 KB | 18 bytes | 3% | Continuous |
| test_noise | ~1.6 KB | 18 bytes | 3% | Continuous |
| test_humidity | ~1.7 KB | 20 bytes | 3% | Continuous |
| test_eeprom | ~1.3 KB | 24 bytes | <1% | Single-shot |

**PIC16F886 Resources**:
- Flash: 8K words (14-bit) = ~11 KB usable
- RAM: 368 bytes
- EEPROM: 256 bytes

All tests fit comfortably within resource limits.

---

## Safety and Precautions

### Electrical Safety
- Verify polarity before powering sensors
- Use current-limited supply (<500mA) during initial testing
- Add reverse polarity protection diode if using external power
- Never exceed 5.5V on any PIC pin

### ESD Protection
- Use anti-static wrist strap when handling bare PCBs
- Keep humidity >30% to reduce static buildup
- Store unused chips in conductive foam

### Component Protection
- Add flyback diode across inductive loads (fan, relay)
- Limit LED current with appropriate brightness values
- Never hot-plug I2C devices (power off first)

---

## References

### Datasheets
- **PIC16F886**: DS41291 (Microchip)
- **LM35**: SNIS159 (Texas Instruments)
- **HIH-4000**: HIH-4000-003 (Honeywell)
- **VEML7700**: 84323 (Vishay)
- **iAQ-Core**: 0278 (ScioSense/ams)
- **SK9822**: SK9822-EC20 (OPSCO)

### Application Notes
- **AN682**: Using Single Supply Sensor Signal Conditioners
- **AN1166**: Handling I2C Communication Errors
- **AN723**: Using PWM to Generate Analog Output

### Tools
- **MPLAB X IDE**: v6.00+ (Microchip)
- **XC8 Compiler**: v2.40+ (Microchip)
- **PICkit 3/4**: Programmer/Debugger
- **Logic Analyzer**: Saleae, DSLogic (for protocol debug)

---

[← Back to Main README](../README.md)
