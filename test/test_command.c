/*
 * Test de recepción de comandos UART
 *
 * Descripción:
 *   Recibe comandos por UART para controlar velocidad del ventilador (PWM)
 *   y color/brillo de la tira de LEDs SK9822.
 *
 * Hardware:
 *   - UART en RC6/RC7 (9600 baud) - Recepción de comandos
 *   - PWM en RC2 (CCP1) - Control de ventilador
 *   - SPI en RB2/RB3 - Control de LEDs (verificar hardware específico)
 *
 * Comandos:
 *   - CMD_FAN (0x04): Ajusta velocidad del ventilador (0-100%)
 *   - CMD_LEDS (0x05): Ajusta color RGB (0-255) y brillo (0-31) de LEDs
 *
 * Dependencias:
 *   - ../protocol/protocol_uart.h: Parser de comandos con máquina de estados
 *   - ../actuador/actuator_led.h: Driver de LEDs SK9822
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
#include "../protocol/protocol_uart.h" // Parser de comandos
#include "../actuador/actuator_led.h"  // Control de LEDs SK9822
#include "../actuador/actuator_fan.h"  // Control de ventilador
#include "../hal/hal_spi.h"            // Necesario para SPI (LEDs)

/*
 * ISR: Recepción de comandos por UART
 * - Lee byte del UART
 * - Alimenta máquina de estados del protocolo
 * - Ejecuta comando cuando está completo
 */
void __interrupt() interrupt_handler()
{
    if (PIR1bits.RCIF == 1)
    {
        // Alimentar máquina de estados con byte recibido
        if (protocol_feed_byte(RCREG))
        {
            // Comando completo recibido

            if (received_command.type == CMD_FAN)
            {
                // Comando de ventilador: ajustar velocidad (0-100%)
                fan_set_speed(received_command.fan_speed);
            }
            else if (received_command.type == CMD_LEDS)
            {
                // Comando de LED: actualizar color RGB y brillo
                led_set_color(
                    received_command.led.red,
                    received_command.led.green,
                    received_command.led.blue,
                    received_command.led.brightness);
            }
        }
    }
}

void main(void)
{
    OSCCONbits.OSTS = 1; // Seleccionar oscilador externo

    // Configurar pines
    TRISCbits.TRISC2 = 0; // RC2 salida (PWM/CCP1 ventilador)
    TRISCbits.TRISC0 = 0; // RC0 salida (SPI CLK para LEDs)
    TRISCbits.TRISC1 = 0; // RC1 salida (SPI SDO para LEDs)
    TRISCbits.TRISC6 = 0; // RC6 salida (UART TX)
    TRISCbits.TRISC7 = 1; // RC7 entrada (UART RX)

    // Inicializar periféricos
    spi_init();          // SPI para LEDs SK9822
    fan_init();          // Ventilador (PWM a 20 kHz)
    protocol_init(9600); // UART a 9600 baud + habilita RX interrupt
    led_init();          // Estado inicial de LEDs

    // Habilitar interrupciones
    PIE1bits.RCIE = 1;   // Interrupción UART RX
    INTCONbits.PEIE = 1; // Interrupciones periféricas
    INTCONbits.GIE = 1;  // Interrupciones globales

    // Loop infinito - todo ocurre en ISR
    while (1)
    {
        // Esperar comandos por UART
    }
}
