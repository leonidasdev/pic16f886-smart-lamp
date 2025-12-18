#ifndef SENSOR_LUX_H
#define SENSOR_LUX_H

#include <stdint.h>

// Devuelve la iluminancia en lux (entero)
// Retorna 0xFFFF si hay error de comunicación I2C
uint16_t lux_read(void);

#endif /* SENSOR_LUX_H */
