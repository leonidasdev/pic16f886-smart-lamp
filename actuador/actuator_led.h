#ifndef ACTUATOR_LED_H
#define ACTUATOR_LED_H

#include <stdint.h>

// Inicializa LEDs con estado por defecto (blanco medio, brillo 16/31)
void led_init(void);

// Ajusta color y brillo de toda la tira
// r,g,b: 0–255, brightness: 0–31
void led_set_color(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness);

#endif /* ACTUATOR_LED_H */
