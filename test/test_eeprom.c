/*
 * Test de EEPROM interna del PIC16F886
 *
 * Este test verifica las funciones de lectura y escritura de la
 * EEPROM interna (256 bytes), leyendo los primeros 16 bytes,
 * escribiendo 4 bytes de prueba, y volviendo a leer para confirmar.
 *
 * Hardware:
 * - EEPROM interna del PIC16F886 (256 bytes)
 * - UART en RC6/RC7 (9600 baud) para salida de resultados
 *
 * Secuencia de escritura EEPROM:
 * 1. Escribir dirección en EEADR
 * 2. Escribir dato en EEDATA
 * 3. Deshabilitar GIE
 * 4. Secuencia 0x55 → EECON2, 0xAA → EECON2
 * 5. Activar WR bit
 * 6. Esperar WR=0 (escritura completa)
 * 7. Restaurar GIE
 *
 * Dependencias:
 * - hal/hal_eeprom.h para funciones de acceso (API byte a byte)
 */

// CONFIG1
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator: High-speed crystal/resonator on RA6/OSC2/CLKOUT and RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
#pragma config MCLRE = ON       // RE3/MCLR pin function select bit (RE3/MCLR pin function is MCLR)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR21V   // Brown-out Reset Selection bit (Brown-out Reset set to 2.1V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)


#include <xc.h>
#include <stdio.h>
#include "../hal/hal_eeprom.h"
#include "../hal/hal_uart.h"




/* Función requerida por printf() */
void putch(char data) {
    while(!TXSTAbits.TRMT);  // Esperar buffer libre
    TXREG = data;            // Enviar byte
}


void main(void) {
    OSCCONbits.OSTS = 1;  // Seleccionar oscilador externo
    
    // Inicializar UART para salida de resultados
    uart_init(9600);
    
    // Leer primeros 16 bytes de EEPROM (estado inicial)
    printf("Reading EEPROM\r\n");
    char buffer[16];
    for(uint8_t i = 0; i < sizeof(buffer); i++) {
        buffer[i] = eeprom_read(i);
    }
    printf("First %d Bytes:\r\n", sizeof(buffer));
    for(char i = 0; i < sizeof(buffer); i++) {
        printf("%d ", buffer[i]);
    }
    printf("\r\n");
    
    // Escribir 4 bytes de prueba: {0, 1, 2, 3}
    printf("Writing to EEPROM\r\n");
    buffer[0] = 0;
    buffer[1] = 1;
    buffer[2] = 2;
    buffer[3] = 3;
    for(uint8_t i = 0; i < 4; i++) {
        eeprom_write(i, buffer[i]);
    }
    
    // Verificar escritura - releer EEPROM
    printf("Written data:\r\n");
    for(uint8_t i = 0; i < sizeof(buffer); i++) {
        buffer[i] = eeprom_read(i);
    }
    for(char i = 0; i < sizeof(buffer); i++) {
        printf("%d ", buffer[i]);
    }
    printf("\r\n");
    
    // Test completado - loop infinito
    while(1);
}

