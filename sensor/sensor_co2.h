#ifndef SENSOR_CO2_H
#define SENSOR_CO2_H

#include <stdint.h>

// Inicializa el sensor de CO₂ iAQ-Core
void co2_init(void);

// Actualiza contador de warmup (llamar cada 5 segundos)
void co2_update_warmup(void);

// Devuelve la concentración de CO₂ en ppm
// Retorna 0xFFFF si:
//   - Sensor aún en warmup (primeros 5 minutos)
//   - Error de comunicación I2C
//   - Sensor reporta error (bit 0 del status byte)
uint16_t co2_read_ppm(void);

#endif /* SENSOR_CO2_H */
