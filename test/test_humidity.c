/*
 * Test de sensor de humedad HIH-4000
 *
 * Descripción:
 *   Lee humedad relativa del sensor HIH-4000 conectado a AN1,
 *   convierte el valor ADC a porcentaje y transmite por UART cada 1s.
 *
 * Hardware:
 *   - HIH-4000 en RA1 (AN1)
 *   - UART en RC6/RC7 (9600 baud)
 *
 * Conversión:
 *   Según datasheet: Vout = (Vsupply)(0.0062 × RH + 0.16)
 *   A 5V: Vout = 0.8V + (RH × 0.031V)
 *   ADC = Vout × 1023 / 5V
 *   Simplificado: RH% ≈ (ADC - 169) / 6
 *     donde: 169 ≈ 0.8V × 204.6
 *            6 ≈ 0.031V × 204.6 / 100
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
#include "../sensor/sensor_hum.h"

#define TMR0_PRESCALER (0b111)     // Prescaler 1:256 → 10ms por overflow
#define TMR0_OFFSET (61)           // Preload para conseguir 10ms exactos
#define TIMER_MAX_ITERATIONS (100) // Contador: 100 × 10ms = 1s entre lecturas
#define CLOCK_CONVERSION (0b10)    // FOSC/32 → TAD = 1.6µs @ 20MHz
#define CHANNEL_SELECT (0b0001)    // Canal AN1 (RA1) donde está el HIH-4000

/* Inicializar Timer0 para tick de 10ms */
void init_T0()
{
    OPTION_REGbits.PSA = 0;             // Asignar prescaler a Timer0
    OPTION_REGbits.T0CS = 0;            // Fuente: reloj interno
    OPTION_REGbits.PS = TMR0_PRESCALER; // Prescaler 1:256
    INTCONbits.T0IE = 1;                // Habilitar interrupción Timer0
}

char timer_iterations = 0;

/*
 * ISR: Timer0 (cada 10ms)
 * - Timer0 incrementa contador hasta 100 (1s), luego lee sensor
 * - Lee HIH-4000 usando sensor HAL y envía por UART
 */
void __interrupt() interrupt_handler()
{
    // Timer0: Tick cada 10ms
    if (INTCONbits.T0IF == 1)
    {
        TMR0 = TMR0_OFFSET;  // Recargar para siguiente 10ms
        INTCONbits.T0IF = 0; // Limpiar flag

        // Incrementar contador cada 10ms
        timer_iterations += 1;
        if (timer_iterations == TIMER_MAX_ITERATIONS)
        { // Cada 100 ticks = 1s
            timer_iterations = 0;

            // Leer humedad usando sensor HAL (incluye calibración y compensación temperatura)
            uint16_t humidity = hum_read_percent();

            // Enviar resultado por UART
            printf("Humidity: %u%%\r\n", humidity);
        }
    }
}

/* Función requerida por printf() */
void putch(char c)
{
    while (!TXSTAbits.TRMT)
        ;      // Esperar buffer libre
    TXREG = c; // Enviar byte
}

void main(void)
{
    OSCCONbits.OSTS = 1;  // Seleccionar oscilador externo
    TRISAbits.TRISA1 = 1; // RA1 (AN1) como entrada analógica

    // Inicializar periféricos usando HAL
    uart_init(9600); // UART 9600 baud
    adc_init();      // ADC configurado por HAL (3 canales)
    init_T0();       // Timer0 para tick de 10ms

    INTCONbits.GIE = 1; // Habilitar interrupciones globales

    // Loop infinito - todo ocurre en ISR
    while (1)
        ;
}

INTCONbits.GIE = 1; // Habilitar interrupciones globales

// Loop infinito - todo ocurre en ISR
while (1)
    ;
}
