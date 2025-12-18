/*
 * Test de tira de LEDs SK9822
 *
 * Descripción:
 *   Cicla entre diferentes colores en una tira de 10 LEDs SK9822,
 *   mostrando capacidades de control de brillo y mezcla RGB.
 *   El test muestra 4 colores diferentes con distintos niveles de brillo.
 *
 * Hardware:
 *   - Tira de 10 LEDs SK9822
 *   - RC0 (CLK) - Reloj SPI
 *   - RC1 (SDO) - Datos SPI
 *
 * Protocolo SK9822:
 *   [Start: 32 bits a 0] + [LED Data: 32 bits × N] + [End: 32 bits a 1]
 *   LED Data = [111 + brightness(5)] [Blue(8)] [Green(8)] [Red(8)]
 *
 * Dependencias:
 *   - ../hal/hal_spi.h para comunicación SPI
 *   - ../actuador/actuator_led.h para control de LEDs SK9822
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
#include "../hal/hal_spi.h"
#include "../actuador/actuator_led.h"

#pragma intrinsic(_delay)
#define _XTAL_FREQ 20000000 // 20 MHz para __delay_us

/* Funci\u00f3n principal: cicla entre diferentes colores */
void main(void)
{
    OSCCONbits.OSTS = 1; // Oscilador externo

    // Inicializar SPI antes de usar LEDs
    spi_init();

    while (1)
    {
        // Rojo intermedio (brightness 12/31 ≈ 38%)
        led_set_color(255, 0, 0, 12);
        __delay_us(2000000); // 2 segundos

        // Verde m\u00e1ximo brillo (brightness 31/31 = 100%)
        led_set_color(0, 255, 0, 31);
        __delay_us(2000000);

        // Azul tenue (brightness 5/31 \u2248 16%)
        led_set_color(0, 0, 255, 5);
        __delay_us(2000000);

        // P\u00farpura/Rosa (RGB mix, m\u00e1ximo brillo)
        led_set_color(140, 57, 200, 31);
        __delay_us(2000000);
    }
}
[Start Frame][LED Data...][End Frame] * /
    void write_led(char brightness, char red, char green, char blue)
{
    // Start Frame: 32 bits en 0
    for (int i = 0; i < 4; i++)
    {
        spi_write_read(0x00);
    }

    // LED Data: 32 bits por LED (Global + B + G + R)
    for (int i = 0; i < 10; i++)
    {
        // Global: 111 + brightness[4:0] (brightness debe ser 0-31)
        spi_write_read(0b11100000 | (brightness & 0x1F));
        spi_write_read(blue);  // Blue byte
        spi_write_read(green); // Green byte
        spi_write_read(red);   // Red byte
    }

    // End Frame: 32 bits en 1 (m\u00ednimo (n+1)/2 bits seg\u00fan SK9822)
    for (int i = 0; i < 4; i++)
    {
        spi_write_read(0xFF);
    }
}

/*
 * SPI bit-bang: transmisi\u00f3n de 1 byte MSB-first
 * Solo transmisi\u00f3n (MOSI), no lectura (MISO no conectado)
 */
char spi_write_read(char one_byte)
{
    char answer, x;
    answer = 0;

    // Transmitir 8 bits, MSB primero
    for (x = 8; x > 0; x--)
    {
        // Poner bit en MOSI (RC1)
        spi_dat_out = (__bit)((one_byte >> (x - 1)) & 0b00000001);

        // Clock alto (captura dato en LED)
        spi_clk = 1;

        // Clock bajo (preparar siguiente bit)
        spi_clk = 0;

        // No se lee MISO (solo transmisión)
        // answer |= (char)spi_dat_in;
    }

    return answer; // Retorna 0 (no hay lectura)
}
