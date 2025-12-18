#include <xc.h>
#include "hal_uart.h"

#define _XTAL_FREQ 20000000UL

void uart_init(uint32_t baud)
{
    // Configuración EUSART: Async, 8N1
    TXSTA = 0b00100100; // BRGH=1, TXEN=1
    RCSTA = 0b10010000; // SPEN=1, CREN=1
    BAUDCTL = 0x00;     // BRG16=0

    // SPBRG cálculo: SPBRG = (Fosc/(16*baud)) - 1
    uint16_t spbrg = (uint16_t)((_XTAL_FREQ / (16UL * baud)) - 1UL);

    // Para 8-bit BRG, solo usar byte bajo (máximo 255)
    // Si spbrg > 255, el baudrate es muy bajo y se truncará
    SPBRG = (uint8_t)(spbrg & 0xFF);
}

void uart_send_byte(uint8_t b)
{
    while (!PIR1bits.TXIF)
        ; // esperar buffer libre
    TXREG = b;
}

// Versión con timeout para evitar bloqueo indefinido
// Timeout ~100ms @ 20MHz (ajustable según necesidad)
bool uart_read_byte_timeout(uint8_t *byte)
{
    uint16_t timeout = 0xFFFF; // ~100ms con loop overhead

    while (!PIR1bits.RCIF)
    {
        if (--timeout == 0)
        {
            return false; // Timeout: no hay dato
        }
    }

    // Verificar errores de hardware
    if (RCSTAbits.OERR)
    {
        // Overrun error: limpiar deshabilitando/habilitando receptor
        RCSTAbits.CREN = 0;
        RCSTAbits.CREN = 1;
        return false;
    }

    if (RCSTAbits.FERR)
    {
        // Framing error: leer y descartar byte corrupto
        uint8_t dummy = RCREG;
        (void)dummy; // Suprimir warning unused
        return false;
    }

    *byte = RCREG;
    return true; // Éxito: dato leído
}

bool uart_data_available(void)
{
    return PIR1bits.RCIF;
}
