/*
 * Test de control de ventilador con PWM
 *
 * Descripción:
 *   Genera una rampa automática de velocidad del ventilador, incrementando
 *   y decrementando el duty cycle cada 100ms para crear efecto visual.
 *
 * Hardware:
 *   - Ventilador en RC2 (CCP1/PWM)
 *
 * Funcionamiento:
 *   - Timer0: Genera ticks cada 10ms
 *   - Timer2: Genera PWM a 20 kHz
 *   - Duty cycle: Rampa de 0% a 80% y viceversa
 *   - Cambio: Cada 100ms (10 ticks × 10ms)
 */

// CONFIG1
#pragma config FOSC = HS   // Oscillator Selection bits (HS oscillator: High-speed crystal/resonator on RA6/OSC2/CLKOUT and RA7/OSC1/CLKIN)
#pragma config WDTE = OFF  // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = ON  // Power-up Timer Enable bit (PWRT enabled)
#pragma config MCLRE = ON  // RE3/MCLR pin function select bit (RE3/MCLR pin function is MCLR)
#pragma config CP = OFF    // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF   // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF  // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF   // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR21V // Brown-out Reset Selection bit (Brown-out Reset set to 2.1V)
#pragma config WRT = OFF      // Flash Program Memory Self Write Enable bits (Write protection off)

#include <xc.h>
#include "../actuador/actuator_fan.h"

#define TMR0_PRESCALER (0b111)    // Prescaler 1:256 → 10ms por overflow
#define TMR0_OFFSET (61)          // Preload para conseguir 10ms exactos
#define TIMER_MAX_ITERATIONS (10) // Contador: 10 × 10ms = 100ms entre cambios
#define MAX_SPEED 80              // Límite superior: 80% duty cycle
#define MIN_SPEED 0               // Límite inferior: 0% duty cycle

// Variables globales
int timer_iterations = 0; // Contador de ticks para timing
int increment = 1;        // Dirección: 1=incrementa, 0=decrementa
char speed_percent = 0;   // Velocidad actual (0-80%)

/* Inicializar Timer0 para tick de 10ms */
void init_T0()
{
    OPTION_REGbits.PSA = 0;             // Prescaler asignado a Timer0
    OPTION_REGbits.T0CS = 0;            // Clock source = Fosc/4
    OPTION_REGbits.PS = TMR0_PRESCALER; // Prescaler 1:256
    INTCONbits.T0IE = 1;                // Habilitar interrupción Timer0
}

/* Interrupción cada 10ms para cambio gradual de velocidad */
void __interrupt() interrupt_handler()
{

    if (INTCONbits.T0IF == 1)
    {
        TMR0 = TMR0_OFFSET;  // Recargar Timer0 para siguiente tick 10ms
        INTCONbits.T0IF = 0; // Limpiar flag

        timer_iterations += 1;
        if (timer_iterations == TIMER_MAX_ITERATIONS)
        { // Cada 100ms
            timer_iterations = 0;

            // Cambiar velocidad gradualmente (efecto rampa)
            if (increment)
            {
                speed_percent++; // Incrementar velocidad
                fan_set_speed(speed_percent);
                if (speed_percent == MAX_SPEED)
                {
                    increment = 0; // Cambiar dirección
                }
            }
            else
            {
                speed_percent--; // Decrementar velocidad
                fan_set_speed(speed_percent);
                if (speed_percent == MIN_SPEED)
                {
                    increment = 1; // Cambiar dirección
                }
            }
        }
    }
}

void main(void)
{
    OSCCONbits.OSTS = 1;  // Seleccionar oscilador externo (cristal 20 MHz)
    TRISCbits.TRISC2 = 0; // RC2 (CCP1) como salida

    // Inicializar periféricos
    fan_init(); // Inicializa ventilador (PWM a 20 kHz)
    init_T0();  // Timer0 para cambios graduales

    INTCONbits.GIE = 1; // Habilitar interrupciones globales

    // Loop infinito - el ventilador cambia velocidad automáticamente vía ISR
    while (1)
        ;
}
