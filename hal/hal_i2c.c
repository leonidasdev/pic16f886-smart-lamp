#include <xc.h>
#include "hal_i2c.h"
#include "../lib/i2c-v2.h" // funciones oficiales: i2c_start/stop/rstart/write/read

void hal_i2c_init(void)
{
    // MSSP como maestro a 100 kHz
    SSPCON = 0b00101000; // I2C Master mode, SSPEN=0 (se habilita después)
    SSPCON2 = 0x00;
    SSPADD = 49;          // 100 kHz @ 20 MHz: (20MHz/(4*100kHz))-1 = 49
    SSPSTAT = 0b10000000; // Slew rate off para 100kHz

    SSPCONbits.SSPEN = 1; // Habilitar módulo MSSP (CRÍTICO)
}

void hal_i2c_start(void) { i2c_start(); }
void hal_i2c_stop(void) { i2c_stop(); }
void hal_i2c_rstart(void) { i2c_rstart(); }
uint8_t hal_i2c_write(uint8_t data) { return i2c_write(data); }
uint8_t hal_i2c_read(uint8_t ack) { return i2c_read(ack); }
