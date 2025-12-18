#ifndef HAL_PWM_H
#define HAL_PWM_H

#include <stdint.h>

// Inicializa CCP1 en RC2 como salida PWM
void pwm_init(void);

// Ajusta el duty cycle en porcentaje (0–100%)
void pwm_set_percent(uint8_t pct);

#endif /* HAL_PWM_H */
