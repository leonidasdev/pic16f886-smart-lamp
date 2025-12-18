#include <xc.h>
#include "hal_eeprom.h"

void eeprom_write(uint8_t addr, uint8_t val)
{
    EEADR = addr; // Dirección
    EEDATA = val; // Dato
    EEPGD = 0;    // Seleccionar EEPROM (no Flash)
    WREN = 1;     // Habilitar escritura

    // Secuencia obligatoria de desbloqueo
    INTCONbits.GIE = 0; // Deshabilitar interrupciones
    EECON2 = 0x55;
    EECON2 = 0xAA;
    WR = 1; // Iniciar escritura

    while (WR)
        ; // Esperar fin de escritura

    INTCONbits.GIE = 1; // Rehabilitar interrupciones
    WREN = 0;           // Deshabilitar escritura
}

uint8_t eeprom_read(uint8_t addr)
{
    EEADR = addr;  // Dirección
    EEPGD = 0;     // Seleccionar EEPROM
    RD = 1;        // Iniciar lectura
    return EEDATA; // Devolver dato
}
