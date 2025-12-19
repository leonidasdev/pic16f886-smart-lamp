/*
 * Test de sensor VEML7700 (luminosidad) por I2C
 *
 * Este test inicializa el sensor VEML7700 y lee periódicamente
 * la luminosidad ambiental en lux. El sensor responde en 16 bits
 * que se convierten a lux según el rango configurado.
 *
 * Hardware:
 * - VEML7700 en dirección I2C 0x10
 * - I2C en RC3 (SCL) y RC4 (SDA) a 100 kHz
 * - UART en RC6/RC7 (9600 baud) para salida de resultados
 *
 * Protocolo I2C:
 * - Start → Dir 0x10 → Cmd 0x00 (Config) → Datos → Stop
 * - Start → Dir 0x10 → Cmd 0x04 (ALS) → Leer 2 bytes → Stop
 * - Conversión a lux según ganancia y tiempo de integración
 *
 * Dependencias:
 * - ../sensor/sensor_lux.h para lectura del sensor
 * - ../hal/hal_i2c.h para comunicación I2C
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
#include <stdint.h>
#include <stdio.h>
#include "../sensor/sensor_lux.h"
#include "../hal/hal_i2c.h"
#include "../hal/hal_uart.h"

#pragma intrinsic(_delay)
#define _XTAL_FREQ 20000000 // Necesario para __delay_us

void main(void)
{
    // Configurar pines I2C como entradas (modo open-drain con pull-ups externos)
    TRISCbits.TRISC3 = 1; // RC3 (SCL) entrada
    TRISCbits.TRISC4 = 1; // RC4 (SDA) entrada

    // Inicializar periféricos usando HAL
    hal_i2c_init();   // I2C a 100 kHz (configurado en HAL)
    uart_init(9600);  // UART a 9600 baud usando HAL
    __delay_us(2500); // Esperar estabilización del sensor

    // Loop de lectura cada 500ms
    while (1)
    {
        // Leer luminosidad en lux (16 bits)
        uint16_t lux = lux_read();
        printf("Luminosity: %u lux\r\n", lux);
        __delay_us(500000); // Esperar 500ms
    }
}

/* Función requerida por printf() */
void putch(char data)
{
    while (!TXSTAbits.TRMT)
        ;         // Esperar buffer libre
    TXREG = data; // Enviar byte
}
