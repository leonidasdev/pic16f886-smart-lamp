/*
 * Test de sensor iAQ-Core CO2 por I2C
 *
 * Este test inicializa el sensor iAQ-Core (CO2 y VOC) por I2C
 * y lee periódicamente los valores de CO2 en ppm. El sensor
 * requiere calentamiento de hasta 5 minutos antes de dar lecturas
 * válidas.
 *
 * Hardware:
 * - iAQ-Core en dirección I2C 0x5A
 * - I2C en RC3 (SCL) y RC4 (SDA) a 100 kHz
 * - UART en RC6/RC7 (9600 baud) para salida de resultados
 *
 * Protocolo I2C:
 * - Start → Dir 0x5A → Leer 9 bytes → Stop
 * - Byte 0-1: CO2 (ppm, big-endian)
 * - Byte 2-3: Estado (0x00=OK, 0x10=Calentando, 0x80=Error)
 * - Bytes 4-8: VOC y resistance
 *
 * Dependencias:
 * - i2c-v2.h para comunicación I2C
 * - Funciones iaq_read() definidas inline
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
#include "../hal/hal_i2c.h"
#include "../hal/hal_uart.h"
#include "../sensor/sensor_co2.h"

#pragma intrinsic(_delay)
#define _XTAL_FREQ 20000000 // necessary for __delay_us

void main(void)
{
    // Configurar pines I2C como entradas (modo open-drain con pull-ups externos)
    TRISCbits.TRISC3 = 1; // RC3 (SCL) entrada
    TRISCbits.TRISC4 = 1; // RC4 (SDA) entrada

    // Inicializar periféricos usando HAL
    hal_i2c_init();  // I2C a 100 kHz
    co2_init();      // Sensor CO2 (contador warmup)
    uart_init(9600); // UART a 9600 baud usando HAL

    // Contador de actualizaciones para warmup (cada 5s)
    uint16_t update_count = 0;

    // Loop de lectura cada 500ms
    while (1)
    {
        uint16_t co2_ppm = co2_read_ppm();

        // Actualizar warmup cada 5 segundos (10 iteraciones * 500ms)
        if (++update_count >= 10)
        {
            co2_update_warmup();
            update_count = 0;
        }

        if (co2_ppm == 0xFFFF)
        {
            // Sensor aún en warmup o error I2C
            puts("Sensor warming up or error\n\r");
        }
        else
        {
            // Lectura válida
            printf("CO2: %u ppm\n\r", co2_ppm);
        }

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

enum iaq_status
{
    // Lectura correcta.
    IAQ_STATUS_OK = 0x00,
    // Sensor calentandose.
    IAQ_STATUS_WARMUP = 0x10,
    // El sensor ha sobreescrito el valor de CO2 mientras lo leíamos.
    // Hay que volver a leer.
    IAQ_STATUS_BUSY = 0x01,
    // Error interno del sensor.
    IAQ_STATUS_ERROR = 0x80
};
/*
 * Leer CO2 del sensor iAQ-Core
 *
 * Protocolo I2C:
 * 1. START + dirección 0x5A + Read bit
 * 2. Leer 3 bytes: CO2_MSB, CO2_LSB, STATUS
 * 3. STOP
 *
 * Reintenta hasta IAQ_SENSOR_BUSY_RETRIES veces si STATUS=BUSY
 *
 * Retorna:
 * - IAQ_OK: Lectura válida en *co2
 * - IAQ_NOT_READY: Sensor calentándose (ignorar *co2)
 * - IAQ_ERROR: Error I2C o interno (ignorar *co2)
 */
enum iaq_result iaq_read(uint16_t *co2)
{
    unsigned char read_attempts = IAQ_SENSOR_BUSY_RETRIES;

    // Si el sensor devuelve `IAQ_STATUS_BUSY`, lo volvemos a intentar
    while (read_attempts--)
    {

        // El sensor solo soporta un modo de comunicacion: lectura.
        // Al leer, el sensor envía 9 bytes de datos con la siguiente estructura:
        //
        // CO2 MSB ~ CO2 LSB ~ STATUS ~ Ohm MMSB ~ Ohm MLSB ~ Ohm LMSB ~ Ohm LLSB ~ VOC MSB ~ VOC LSB
        //
        // Únicamente nos interesa el CO2 y el estado (para saber si el CO2 es
        // válido), así que solo leemos 3 bytes.

        // Solo podemos leer.
        i2c_start();
        if (!i2c_write(IAQ_SENSOR_I2C_ADDRESS << 1 | 1))
        { // Direcci�n + Read
            // Si el sensor no ha respondido con ACK, devolvemos un error.
            i2c_stop();
            return IAQ_ERROR;
        }

        // El primer byte es el más significativo.
        *co2 = (uint16_t)i2c_read(1) << 8;
        *co2 |= (uint16_t)i2c_read(1);

        // Enviamos un NACK en la última lectura para indicarle al sensor que no
        // queremos seguir leyendo.
        enum iaq_status status = i2c_read(0);
        i2c_stop();

        // Comprobamos el estado del sensor.
        switch (status)
        {
        case IAQ_STATUS_OK:
        case IAQ_STATUS_WARMUP:
        case IAQ_STATUS_ERROR:
            // Los tres valores son iguales en ambos enums.
            return (enum iaq_result)status;

        // Volver a intentar.
        case IAQ_STATUS_BUSY:
            break;

        // Si el sensor devuelve un valor que no corresponde con el
        // datasheet, devolvemos un error.
        default:
            return IAQ_ERROR;
        }
    }

    // Si el sensor ha devuelto `IAQ_STATUS_BUSY` demasiadas veces,
    // devolvemos un error.
    return IAQ_ERROR;
}
