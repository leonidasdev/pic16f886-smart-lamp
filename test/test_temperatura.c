/*
 * Test de sensor de temperatura LM35
 *
 * Descripción:
 *   Lee temperatura del sensor LM35 (10mV/°C) conectado a AN0, convierte
 *   el valor ADC a grados Celsius y transmite por UART cada 500ms.
 *
 * Hardware:
 *   - LM35 en RA0 (AN0)
 *   - UART en RC6/RC7 (9600 baud)
 *
 * Conversión:
 *   ADC (0-1023) → Tensión (mV) → Temperatura (°C)
 *   mV = (ADC × 5000) / 1024
 *   °C = mV / 10
 *   Simplificado: °C = ADC × 0.4887
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
#include <stdio.h>
#include "../hal/hal_adc.h"
#include "../hal/hal_uart.h"
#include "../sensor/sensor_temp.h"

#define TMR0_PRESCALER (0b110)     // Prescaler 1:128 → 5ms por overflow
#define TMR0_OFFSET (61)           // Preload para conseguir 5ms exactos
#define TIMER_MAX_ITERATIONS (100) // Contador: 100 × 5ms = 500ms entre lecturas
#define CLOCK_CONVERSION (0b10)    // FOSC/32 → TAD = 1.6µs @ 20MHz
#define CHANNEL_SELECT (0b0000)    // Canal AN0 (RA0) donde está conectado el LM35

/* Inicializar Timer0 para tick de 5ms (prescaler 1:128) */
void init_T0()
{
    OPTION_REGbits.PSA = 0;             // Prescaler asignado a Timer0
    OPTION_REGbits.T0CS = 0;            // Clock source = Fosc/4
    OPTION_REGbits.PS = TMR0_PRESCALER; // Prescaler 1:128
    INTCONbits.T0IE = 1;                // Habilitar interrupción
}

/* Función requerida por printf() */
void putch(char data)
{
    while (!TXSTAbits.TRMT)
        ;         // Esperar buffer libre
    TXREG = data; // Enviar byte
}

// Variable global: contador para muestreo cada 500ms
char timer_iterations = 0;

/* ISR: Muestreo cada 500ms + envío por UART */
void __interrupt() interrupt_handler()
{

    // Timer0: cada 5ms
    if (INTCONbits.T0IF == 1)
    {
        TMR0 = TMR0_OFFSET;  // Recargar para siguiente tick
        INTCONbits.T0IF = 0; // Limpiar flag

        timer_iterations += 1;
        if (timer_iterations == TIMER_MAX_ITERATIONS)
        { // Cada 500ms
            timer_iterations = 0;

            // Leer temperatura usando sensor HAL
            uint8_t temperature = temp_read_degC();

            // Enviar por UART
            printf("Temperature: %d C\r\n", temperature);
        }
    }
}

void main(void)
{
    OSCCONbits.OSTS = 1;  // Seleccionar oscilador externo
    TRISAbits.TRISA0 = 1; // RA0 como entrada (AN0, LM35)

    // Inicializar periféricos usando HAL
    uart_init(9600); // UART 9600 baud usando HAL
    adc_init();      // ADC configurado por HAL
    init_T0();       // Timer0 para muestreo 500ms

    INTCONbits.GIE = 1; // Habilitar interrupciones globales

    // Loop infinito - todo ocurre en ISR
    while (1)
        ;
}
