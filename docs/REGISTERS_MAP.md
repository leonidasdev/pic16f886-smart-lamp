# PIC16F886 - Register Map

**Microcontroller:** PIC16F886 @ 20 MHz  
**Project:** SMA-LAMP (LMDE-MA)  
**Date:** December 2025

---

## Table of Contents

1. [Port Control Registers](#port-control-registers)
2. [Timer0 Registers](#timer0-registers)
3. [Timer1 Registers](#timer1-registers)
4. [Timer2 Registers](#timer2-registers)
5. [ADC Registers](#adc-registers)
6. [UART Registers (EUSART)](#uart-registers-eusart)
7. [I2C Registers (MSSP)](#i2c-registers-mssp)
8. [PWM Registers (CCP1)](#pwm-registers-ccp1)
9. [EEPROM Registers](#eeprom-registers)
10. [Interrupt Registers](#interrupt-registers)
11. [Configuration Summary](#configuration-summary)

---

## Port Control Registers

### TRISA (Address: 0x85)
**Function:** Direction control for Port A (1=input, 0=output)

| Bit | Name | Configuration | Function |
|-----|--------|---------------|---------|
| 0 | TRISA0 | **1** | Analog input AN0 (LM35 temp) |
| 1 | TRISA1 | **1** | Analog input AN1 (HIH4000 hum) |
| 2 | TRISA2 | **1** | Analog input AN2 (Microphone) |
| 3-7 | - | x | Not used in project |

**Code:**
```c
TRISAbits.TRISA0 = 1;
TRISAbits.TRISA1 = 1;
TRISAbits.TRISA2 = 1;
```

---

### TRISC (Address: 0x87)
**Function:** Direction control for Port C

| Bit | Name | Configuration | Function |
|-----|--------|---------------|---------|
| 0 | TRISC0 | **0** | Output SPI CLK (SK9822 LEDs) |
| 1 | TRISC1 | **0** | Output SPI SDO (SK9822 LEDs) |
| 2 | TRISC2 | **0** | Output PWM CCP1 (Fan) |
| 3 | TRISC3 | **1** | Input I2C SCL (MSSP controlled) |
| 4 | TRISC4 | **1** | Input I2C SDA (MSSP controlled) |
| 5 | TRISC5 | x | Not used |
| 6 | TRISC6 | **0** | Output UART TX |
| 7 | TRISC7 | **1** | Input UART RX |

**Code:**
```c
TRISCbits.TRISC0 = 0; // SPI CLK
TRISCbits.TRISC1 = 0; // SPI SDO
TRISCbits.TRISC2 = 0; // PWM
TRISCbits.TRISC3 = 1; // I2C SCL
TRISCbits.TRISC4 = 1; // I2C SDA
TRISCbits.TRISC6 = 0; // UART TX
TRISCbits.TRISC7 = 1; // UART RX
```

---

## Timer0 Registers

### OPTION_REG (Address: 0x81)
**Function:** Timer0 and prescaler configuration

| Bit | Name | Value | Description |
|-----|--------|-------|-------------|
| 7 | nRBPU | **1** | **PORTB pull-ups disabled** (PORTB not used) |
| 6 | INTEDG | 0 | INT edge (not used) |
| 5 | T0CS | **0** | **Clock source = Fosc/4** |
| 4 | T0SE | 0 | Source edge (not used) |
| 3 | PSA | **0** | **Prescaler assigned to Timer0** |
| 2-0 | PS<2:0> | **111** | **Prescaler 1:256** |

**Configured value:** `0b10000111`

**Period calculation:**
```
Fosc/4 = 20 MHz / 4 = 5 MHz
Base tick = 0.2 µs
With prescaler 1:256 → Tick = 51.2 µs
For 10 ms → Required ticks = 10000 µs / 51.2 µs ≈ 195
Preload = 256 - 195 = 61
```

**Code:**
```c
OPTION_REG = 0b10000111; // nRBPU=1 (PORTB pull-ups OFF)
TMR0 += 61; // Compensated reload
```

---

### TMR0 (Address: 0x01)
**Function:** Timer0 counter (8 bits)

**Usage:** Increments every 51.2 µs, generates interrupt every 10 ms

**Code in ISR:**
```c
TMR0 += 61; // Compensated reload (avoids accumulated drift)
```

---

## Timer1 Registers

### T1CON (Address: 0x10)
**Function:** Timer1 control (16 bits)

| Bit | Name | Value | Description |
|-----|--------|-------|-------------|
| 7 | T1GINV | 0 | Gate invert (0=active-low, not used) |
| 6 | TMR1GE | **0** | **Gate disable (always counting)** |
| 5-4 | T1CKPS<1:0> | **11** | **Prescaler 1:8** |
| 3 | T1OSCEN | **0** | **LP oscillator OFF** |
| 2 | T1SYNC | 0 | Sync external clock (irrelevant with TMR1CS=0) |
| 1 | TMR1CS | **0** | **Clock source = Internal (Fosc/4)** |
| 0 | TMR1ON | **1** | **Timer1 ON** |

**Configured value:** `0b00110001` (0x31)

**Period calculation:**
```
Fosc/4 = 5 MHz
With prescaler 1:8 → Tick = 1.6 µs
16-bit overflow = 65536 × 1.6 µs = 104.8576 ms ≈ 105 ms
```

**Code:**
```c
T1CON = 0b00110001;
PIR1bits.TMR1IF = 0;
PIE1bits.TMR1IE = 1;
```

---

### TMR1H and TMR1L (Addresses: 0x0F, 0x0E)
**Function:** Timer1 counter (16 bits, auto-reload from 0x0000)

**Usage:** Overflow every ~105 ms for scheduler tick

---

## Timer2 Registers

### T2CON (Address: 0x12)
**Function:** Timer2 control (used for PWM)

| Bit | Name | Value | Description |
|-----|--------|-------|-------------|
| 6-3 | TOUTPS<3:0> | 0000 | Postscaler 1:1 (not used) |
| 2 | TMR2ON | **1** | **Timer2 ON** |
| 1-0 | T2CKPS<1:0> | **00** | **Prescaler 1:1** |

**Configured value:** `0b00000100`

**Code:**
```c
T2CON = 0b00000100;
```

---

### PR2 (Address: 0x92)
**Function:** Timer2 period (determines PWM frequency)

**Configured value:** `249`

**PWM frequency calculation:**
```
PWM_freq = Fosc / (4 × Prescaler × (PR2+1))
PWM_freq = 20 MHz / (4 × 1 × 250) = 20 kHz
```

**Code:**
```c
PR2 = 249;
```

---

## ADC Registers

### ADCON0 (Address: 0x1F)
**Function:** ADC control

| Bit | Name | Value | Description |
|-----|--------|-------|-------------|
| 7-6 | ADCS<1:0> | **10** | **Clock = FOSC/32 (TAD = 1.6 µs)** |
| 5-2 | CHS<3:0> | Variable | **Selected channel (AN0-AN2)** |
| 1 | GO/nDONE | 0/1 | Start conversion / Busy flag |
| 0 | ADON | **1** | **ADC ON** |

**Initialization:** `ADCON0 = 0b10000001` (ADCS=10, CHS=0000, ADON=1)

**Channel selection:**
```c
ADCON0 &= 0b11000001; // Keep ADCS and ADON, clear CHS
ADCON0 |= (channel << 2); // Configure channel (0-2)
```

---

### ADCON1 (Address: 0x9F)
**Function:** ADC reference and format configuration

| Bit | Name | Value | Description |
|-----|--------|-------|-------------|
| 7 | ADFM | **1** | **Result right justified** |
| 6 | - | 0 | Not implemented |
| 5 | VCFG1 | **0** | **Negative reference = VSS** (0=VSS, 1=VREF-) |
| 4 | VCFG0 | **0** | **Positive reference = VDD** (0=VDD, 1=VREF+) |
| 3-0 | - | 0000 | Not implemented |

**Configured value:** `0b10000000`

**Code:**
```c
ADCON1 = 0b10000000; // ADFM=1 (right), VCFG1:VCFG0=00 (VDD/VSS)
```

---

### ANSEL (Address: 0x188)
**Function:** Analog pin selection (AN0-AN7)

| Bit | Name | Value | Description |
|-----|--------|-------|-------------|
| 0 | ANS0 | **1** | **RA0/AN0 = Analog (LM35)** |
| 1 | ANS1 | **1** | **RA1/AN1 = Analog (HIH4000)** |
| 2 | ANS2 | **1** | **RA2/AN2 = Analog (Mic)** |
| 3-7 | ANS3-7 | 0 | Digital |

**Configured value:** `0b00000111`

**Code:**
```c
ANSEL = 0b00000111;
ANSELH = 0x00;
```

---

### ADRESH and ADRESL (Addresses: 0x1E, 0x9E)
**Function:** ADC conversion result (10 bits)

**Format (right justified):**
```
ADRESH[7:0] = bits 9-2
ADRESL[7:0] = bits 1-0 in positions 7-6
```

**Reading:**
```c
uint16_t result = ((uint16_t)ADRESH << 8) | ADRESL;
```

---

## UART Registers (EUSART)

### TXSTA (Address: 0x98)
**Function:** Transmit status and control

| Bit | Name | Value | Description |
|-----|--------|-------|-------------|
| 7 | CSRC | 0 | Clock source (async) |
| 6 | TX9 | 0 | 8-bit transmission |
| 5 | TXEN | **1** | **Transmit enable** |
| 4 | SYNC | 0 | **Asynchronous mode** |
| 3 | SENDB | 0 | Sync break (not used) |
| 2 | BRGH | **1** | **High speed baud rate** |
| 1 | TRMT | - | TSR empty (read only) |
| 0 | TX9D | 0 | 9th bit (not used) |

**Configured value:** `0b00100100`

**Code:**
```c
TXSTA = 0b00100100; // BRGH=1, TXEN=1
```

---

### RCSTA (Address: 0x18)
**Function:** Receive status and control

| Bit | Name | Value | Description |
|-----|--------|-------|-------------|
| 7 | SPEN | **1** | **Serial port enable** |
| 6 | RX9 | 0 | 8-bit reception |
| 5 | SREN | 0 | Single receive (not used) |
| 4 | CREN | **1** | **Continuous receive enable** |
| 3 | ADDEN | 0 | Address detect (not used) |
| 2 | FERR | - | Framing error (read only) |
| 1 | OERR | - | Overrun error (read only) |
| 0 | RX9D | - | 9th bit (read only) |

**Configured value:** `0b10010000`

**Code:**
```c
RCSTA = 0b10010000; // SPEN=1, CREN=1
```

---

### SPBRG (Address: 0x99)
**Function:** Baud rate generator

**Configured value:** `129`

**Calculation for 9600 baud:**
```
SPBRG = (Fosc / (16 × baud)) - 1
SPBRG = (20000000 / (16 × 9600)) - 1
SPBRG = 129.2 ≈ 129
Error = (129 - 129.2) / 129.2 × 100% = -0.16%
```

**Código:**
```c
SPBRG = 129;
```

---

### TXREG (Address: 0x19)
**Function:** UART transmit buffer

**Usage:** Write byte to send via TX

```c
TXREG = byte; // Sends byte through RC6 (TX)
```

---

### RCREG (Address: 0x1A)
**Function:** UART receive buffer

**Usage:** Read received byte from RX

```c
uint8_t byte = RCREG; // Reads byte from RC7 (RX)
```

---

### BAUDCTL (Address: 0x187)
**Function:** Additional baud rate control

**Configured value:** `0x00` (BRG16=0, 8-bit SPBRG mode)

```c
BAUDCTL = 0x00;
```

---

## I2C Registers (MSSP)

### SSPCON (Address: 0x14)
**Function:** MSSP module control (I2C Master)

| Bit | Name | Value | Description |
|-----|--------|-------|-------------|
| 7 | WCOL | - | Write collision (read only) |
| 6 | SSPOV | - | Receive overflow (read only) |
| 5 | SSPEN | **1** | **SSP enable** |
| 4 | CKP | 0 | Clock polarity (not used in master) |
| 3-0 | SSPM<3:0> | **1000** | **I2C Master mode, clock = Fosc/(4×(SSPADD+1))** |

**Configured value:** `0b00101000`

**Code:**
```c
SSPCON = 0b00101000; // I2C Master mode
```

---

### SSPCON2 (Address: 0x91)
**Function:** I2C condition control

| Bit | Name | Description |
|-----|--------|-------------|
| 7 | GCEN | General call enable |
| 6 | ACKSTAT | ACK status (solo lectura) |
| 5 | ACKDT | ACK data bit |
| 4 | ACKEN | ACK enable |
| 3 | RCEN | Receive enable |
| 2 | PEN | Stop condition enable |
| 1 | RSEN | Repeated start enable |
| 0 | SEN | Start condition enable |

**Initialization:** `0x00`

**Usage:** Library `i2c-v2.c` manages these bits automatically

---

### SSPADD (Address: 0x93)
**Function:** Address register / I2C clock divider

**Configured value:** `49`

**I2C frequency calculation:**
```
I2C_clock = Fosc / (4 × (SSPADD + 1))
I2C_clock = 20 MHz / (4 × 50) = 100 kHz
```

**Code:**
```c
SSPADD = 49; // 100 kHz
```

---

### SSPSTAT (Address: 0x94)
**Function:** MSSP module status

| Bit | Name | Value | Description |
|-----|--------|-------|-------------|
| 7 | SMP | **1** | **Slew rate control disabled** (100 kHz) |
| 6 | CKE | 0 | SMBus levels (not used) |
| 5-0 | Flags | - | Communication status (read only) |

**Configured value:** `0b10000000`

**Code:**
```c
SSPSTAT = 0b10000000; // SMP=1 (standard mode)
```

---

### SSPBUF (Address: 0x13)
**Function:** I2C data buffer

**Usage:** Write/read data during I2C transactions

```c
SSPBUF = data;        // Write
uint8_t data = SSPBUF; // Read
```

---

## PWM Registers (CCP1)

### CCP1CON (Address: 0x17)
**Function:** CCP1 module control in PWM mode

| Bit | Name | Value | Description |
|-----|--------|-------|-------------|
| 7-6 | - | 00 | Not implemented |
| 5-4 | DC1B<1:0> | Variable | **2 LSB of duty cycle** |
| 3-0 | CCP1M<3:0> | **1100** | **PWM mode** |

**Configured value:** `0b00001100`

**Code:**
```c
CCP1CON = 0b00001100; // PWM mode
CCP1CONbits.DC1B = duty & 0x03; // 2 LSB de duty
```

---

### CCPR1L (Address: 0x15)
**Function:** 8 MSB of PWM duty cycle

**Usage:** Controls pulse width (0-100%)

**Duty cycle calculation:**
```
Duty_10bit = (pct/100) × 4 × (PR2+1)
CCPR1L = Duty_10bit >> 2        // 8 MSB
DC1B = Duty_10bit & 0x03        // 2 LSB
```

**Code:**
```c
uint16_t duty = (uint32_t)pct * 4U * 250U / 100U;
CCPR1L = (duty >> 2) & 0xFF;
CCP1CONbits.DC1B = duty & 0x03;
```

---

## EEPROM Registers

### EEADR (Address: 0x10D)
**Function:** EEPROM address (0-255)

**Usage in project:** Addresses 0x00-0x04 (5 bytes)

```c
EEADR = addr; // Select address
```

---

### EEDATA (Address: 0x10C)
**Function:** EEPROM data (read/write)

```c
EEDATA = val;           // Write
uint8_t val = EEDATA;   // Read
```

---

### EECON1 (Address: 0x18C)
**Function:** EEPROM/Flash control

| Bit | Name | Description |
|-----|--------|-------------|
| 7 | EEPGD | **Program/Data EEPROM Select** (1=Flash, **0=EEPROM**) |
| 6-4 | - | Not implemented (reads as 0) |
| 3 | WRERR | **EEPROM Error Flag** (1=Write aborted, 0=OK) - Read only |
| 2 | WREN | **Write Enable** (1=Enables write, 0=Inhibits write) |
| 1 | WR | **Write Control** (1=Initiates write, auto-clear by hardware) |
| 0 | RD | **Read Control** (1=Initiates read, auto-clear by hardware) |

**Critical notes:**
- **WR and RD:** Can only be set to 1 by software, hardware clears them automatically
- **WRERR:** Activates if write is aborted by reset, BOR or WDT
- **GIE:** MUST be disabled during 0x55/0xAA sequence to avoid corruption

**Write sequence:**
```c
EEPGD = 0;              // Select EEPROM (not Flash)
WREN = 1;               // Enable write
INTCONbits.GIE = 0;     // CRITICAL: Disable interrupts
EECON2 = 0x55;          // Unlock sequence
EECON2 = 0xAA;          // (must be atomic)
WR = 1;                 // Start write (hardware clears when done)
INTCONbits.GIE = 1;     // Re-enable interrupts
WREN = 0;               // Disable write for safety
while(WR);              // Wait for completion (~4ms, WR=0 when done)
```

**Read sequence:**
```c
EEPGD = 0;    // Select EEPROM
RD = 1;       // Start read (immediate)
return EEDATA;
```

---

### EECON2 (Address: 0x18D)
**Function:** EEPROM unlock sequence (write only)

**Required values:**
1. Write `0x55`
2. Write `0xAA`
3. Activate WR bit

**CRITICAL:** This sequence must be executed with interrupts disabled.

---

## Interrupt Registers

### INTCON (Address: 0x0B)
**Function:** Global interrupt control

| Bit | Name | Value | Description |
|-----|--------|-------|-------------|
| 7 | GIE | **1** | **Global interrupt enable** |
| 6 | PEIE | **1** | **Peripheral interrupt enable** |
| 5 | TMR0IE | **1** | **Timer0 interrupt enable** |
| 4 | INTE | 0 | External interrupt (not used) |
| 3 | RBIE | 0 | Port B change (not used) |
| 2 | TMR0IF | 0/1 | **Timer0 interrupt flag** (clear in ISR) |
| 1 | INTF | 0 | External flag (not used) |
| 0 | RBIF | 0 | Port B flag (not used) |

**Initialization:**
```c
INTCONbits.TMR0IE = 1; // Enable Timer0
INTCONbits.TMR0IF = 0; // Clear flag
INTCONbits.PEIE = 1;   // Enable peripherals
INTCONbits.GIE = 1;    // Enable global
```

**ISR:**
```c
if(INTCONbits.TMR0IF && INTCONbits.TMR0IE) {
    INTCONbits.TMR0IF = 0; // Clear flag manually
    // ... ISR code
}
```

---

### PIE1 (Address: 0x8C)
**Function:** Peripheral Interrupt Enable 1

| Bit | Name | Value | Description |
|-----|--------|-------|-------------|
| 7 | - | 0 | Not implemented (reads as 0) |
| 6 | RCIE | 0 | **USART Receive Interrupt Enable** (project uses polling) |
| 5 | TXIE | 0 | **USART Transmit Interrupt Enable** (project uses polling) |
| 4 | SSPIE | 0 | **Synchronous Serial Port Interrupt Enable** (I²C/SPI, not used) |
| 3 | CCP1IE | 0 | **CCP1 Interrupt Enable** (Capture/Compare/PWM, not used) |
| 2 | TMR2IE | 0 | **Timer2 to PR2 Match Interrupt Enable** (not used) |
| 1 | - | 0 | Not implemented or reserved (depends on PIC16 family) |
| 0 | TMR1IE | **1** | **Timer1 Overflow Interrupt Enable** |

**Note:** In PIC16F886, TMR1IE is at bit 0 of PIE1.

**Initialization:**
```c
PIE1bits.TMR1IE = 1; // Enable Timer1 overflow interrupt (bit 0)
```

---

### PIR1 (Address: 0x0C)
**Function:** Peripheral interrupt flags

| Bit | Name | Usage | Description |
|-----|--------|-----|-------------|
| 7 | - | - | Not implemented (reads as 0) |
| 6 | ADIF | - | **ADC Interrupt Flag** - 1=Conversion complete, 0=No (not used) |
| 5 | RCIF | Polling | **USART Receive Interrupt Flag** - 1=Data received, 0=No |
| 4 | TXIF | Polling | **USART Transmit Interrupt Flag** - 1=TX register empty, 0=Busy |
| 3 | SSPIF | - | **SSP Interrupt Flag** - 1=SPI/I²C event, 0=No (not used) |
| 2 | CCP1IF | - | **CCP1 Interrupt Flag** - 1=Event occurred, 0=No (not used) |
| 1 | TMR2IF | - | **Timer2 Interrupt Flag** - 1=Period reached, 0=No (not used) |
| 0 | TMR1IF | ISR | **Timer1 Interrupt Flag** - 1=Overflow, 0=No |

**ISR:**
```c
if(PIR1bits.TMR1IF && PIE1bits.TMR1IE) {
    PIR1bits.TMR1IF = 0; // Clear flag manually
    // ... ISR code
}
```

**UART Polling:**
```c
while(!PIR1bits.TXIF); // Wait for TX buffer empty
while(!PIR1bits.RCIF); // Wait for data in RX
```

---

## Configuration Summary

### Register Table by Module

| Module | Used Registers | Addresses | Key Values |
|--------|------------------|-------------|---------------|
| **Timer0** | TMR0, OPTION_REG, INTCON | 0x01, 0x81, 0x0B | `OPTION_REG=0x87`, `TMR0+=61` |
| **Timer1** | TMR1H/L, T1CON, PIE1, PIR1 | 0x0F-0E, 0x10, 0x8C, 0x0C | `T1CON=0x31` |
| **Timer2** | TMR2, T2CON, PR2 | 0x11, 0x12, 0x92 | `PR2=249`, `T2CON=0x04` |
| **ADC** | ADCON0/1, ANSEL/H, ADRESH/L | 0x1F, 0x9F, 0x188-189, 0x1E-9E | `ANSEL=0x07`, `ADCON1=0x80` |
| **UART** | TXSTA, RCSTA, SPBRG, TXREG, RCREG | 0x98, 0x18, 0x99, 0x19, 0x1A | `SPBRG=129` |
| **I2C** | SSPCON/2, SSPADD, SSPSTAT, SSPBUF | 0x14, 0x91, 0x93, 0x94, 0x13 | `SSPADD=49`, `SSPCON=0x28` |
| **PWM** | CCP1CON, CCPR1L | 0x17, 0x15 | `CCP1CON=0x0C` |
| **EEPROM** | EEADR, EEDATA, EECON1/2 | 0x10D, 0x10C, 0x18C-18D | Sequence 0x55/0xAA |
| **Ports** | TRISA, TRISC, PORTA, PORTC | 0x85, 0x87, 0x05, 0x07 | I/O Config |
| **Interrupts** | INTCON, PIE1, PIR1 | 0x0B, 0x8C, 0x0C | `GIE=1`, `PEIE=1` |

---

### EEPROM Memory Map

| Address | Content | Range | Update Rate |
|-----------|-----------|-------|---------------|
| **0x00** | LED Brightness | 0-31 | Every 5s |
| **0x01** | LED Red | 0-255 | Every 5s |
| **0x02** | LED Green | 0-255 | Every 5s |
| **0x03** | LED Blue | 0-255 | Every 5s |
| **0x04** | Fan Speed | 0-100 | Every 5s |

---

### Critical Configuration Bits

#### Config Words (Fuses)
```c
#pragma config FOSC = HS        // XT/HS oscillator (20 MHz crystal)
#pragma config WDTE = OFF       // Watchdog timer disabled
#pragma config PWRTE = ON       // Power-up timer enabled (72ms)
#pragma config MCLRE = ON       // MCLR pin enabled
#pragma config BOREN = ON       // Brown-out reset enabled
#pragma config LVP = OFF        // Low-voltage programming disabled
```

---

### Read-Only Registers (Status)

| Register | Relevant Bits | Description |
|----------|-----------------|-------------|
| **PIR1** | TXIF, RCIF | UART flags (auto-clear) |
| **SSPSTAT** | BF, R/W, S, P | I2C status (automatic management) |
| **ADCON0** | GO/nDONE | ADC conversion status |
| **EECON1** | WR | EEPROM write status |

---

### Correctness Verification

| Aspect | Register | Value | Status |
|---------|----------|-------|--------|
| Timer0 10ms | OPTION_REG, TMR0 | 0x07, +61 | Correct |
| Timer1 ~105ms | T1CON | 0x31 | Correct |
| PWM 20kHz | PR2, T2CON | 249, 0x04 | Correct |
| ADC 10-bit | ADCON1, ANSEL | 0x80, 0x07 | Correct |
| UART 9600 | TXSTA, RCSTA, SPBRG | 0x24, 0x90, 129 | Correct |
| I2C 100kHz | SSPCON, SSPADD | 0x28, 49 | Correct |
| EEPROM | Sequence 0x55/0xAA | GIE=0 during write | Correct |
| Interrupts | INTCON, PIE1 | GIE=1, PEIE=1 | Correct |

---

## References

- **Datasheet:** PIC16F886 (DS41291E-page 408)
- **Key sections:**
  - Section 4: I/O Ports
  - Section 5-7: Timers 0/1/2
  - Section 9: ADC
  - Section 11: EUSART
  - Section 13: MSSP (I2C/SPI)
  - Section 15: CCP (PWM)
  - Section 16: EEPROM

---

**Conclusion:** All registers used in the project are correctly configured according to PIC16F886 specifications and SMA-LAMP system requirements.

---

**End of document**
