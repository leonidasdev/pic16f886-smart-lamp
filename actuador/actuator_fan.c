#include "../hal/hal_pwm.h"
#include "actuator_fan.h"

void fan_init(void)
{
    // Estado inicial: ventilador apagado
    fan_set_speed(0);
}

void fan_set_speed(uint8_t percent)
{
    pwm_set_percent(percent);
}
