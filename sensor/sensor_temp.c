#include "../hal/hal_adc.h"
#include "sensor_temp.h"

/**
 * LM35: 10 mV/ºC (lineal desde 0°C)
 * Vref = 5V, ADC = 10 bits → resolución = 5000mV / 1024 = 4.887 mV/LSB
 * Temperatura [°C] = ADC * 4.887mV / 10mV/°C = ADC * 0.4887
 *
 * Aproximación entera: T ≈ (ADC * 489) / 1000
 */

uint8_t temp_read_degC(void)
{
    uint16_t adc = adc_read(ADC_CH_TEMP);
    uint16_t t = (uint16_t)(((uint32_t)adc * 489U) / 1000U); // ≈ adc * 0.4887
    if (t > 255)
        t = 255;
    return (uint8_t)t;
}
