#ifndef HAL_SPI_H
#define HAL_SPI_H

#include <stdint.h>

// Inicializa pines (TRIS) según macros del profesorado
void spi_init(void);

// Envía un byte (ignora lo recibido)
void spi_send_byte(uint8_t b);

#endif
