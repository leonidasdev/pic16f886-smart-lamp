#include "../lib/spi-master-v1.h"
#include "hal_spi.h"
#include <xc.h>

void spi_init(void)
{
    TRISCbits.TRISC0 = 0; // CLK salida
    TRISCbits.TRISC1 = 0; // SDO salida (solo transmisión)
    spi_clk = 0;
    spi_dat_out = 0;
}

void spi_send_byte(uint8_t b)
{
    spi_write_read(b); // usa la rutina oficial
}
