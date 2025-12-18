#ifndef ACTUATOR_FAN_H
#define ACTUATOR_FAN_H

#include <stdint.h>

// Inicializa ventilador (apagado por defecto)
void fan_init(void);

// Ajusta la velocidad del ventilador en porcentaje (0–100%)
void fan_set_speed(uint8_t percent);

#endif /* ACTUATOR_FAN_H */
