#include <xc.h>
#include "hal_pwm.h"

// Frecuencia PWM fija a 20 kHz
// Fosc = 20 MHz, Prescaler = 1:1
// Freq = 20MHz / (4 * 1 * 250) = 20 kHz
#define PWM_PERIOD 249

void pwm_init(void)
{
    // Configuración Timer2 para ~20 kHz
    PR2 = PWM_PERIOD;     // Periodo
    T2CON = 0b00000100;   // Timer2 ON, prescaler=1
    CCP1CON = 0b00001100; // CCP1 en modo PWM
    CCPR1L = 0;           // Duty inicial 0
    TRISCbits.TRISC2 = 0; // RC2 como salida
}

void pwm_set_percent(uint8_t pct)
{
    if (pct > 100)
        pct = 100;
    // Duty = (pct/100) * (4*(PR2+1))
    uint16_t duty = (uint16_t)(((uint32_t)pct * 4U * (PWM_PERIOD + 1U)) / 100U);
    CCP1CONbits.DC1B = duty & 0x03; // 2 LSB
    CCPR1L = (uint8_t)((duty >> 2) & 0xFF);
}
