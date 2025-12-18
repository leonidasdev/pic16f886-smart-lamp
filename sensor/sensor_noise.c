#include "../hal/hal_adc.h"
#include "sensor_noise.h"

/**
 * Micrófono MO-10: clasificación por umbrales ADC (10 bits)
 * Umbrales calibrados empíricamente:
 *   NOISE_LOW:  ADC ≤ 400 (~1.95V)
 *   NOISE_MED:  400 < ADC ≤ 900 (~4.39V)
 *   NOISE_HIGH: ADC > 900
 *
 */

noise_cat_t noise_read_category(void)
{
    uint16_t adc = adc_read(ADC_CH_NOISE);
    if (adc <= 400)
        return NOISE_LOW;
    if (adc <= 900)
        return NOISE_MED;
    return NOISE_HIGH;
}
