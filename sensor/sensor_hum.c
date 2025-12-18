#include "../hal/hal_adc.h"
#include "sensor_hum.h"
#include "sensor_temp.h"

/**
 * HIH-4000 con calibración específica de unidad @ 5V:
 *   1. Conversión ADC a voltaje: Vout = (5.0 * ADC) / 1023
 *   2. Linealización (calibrada): RH_sensor = (Vout - 0.826) / 0.031483
 *   3. Compensación de temperatura: RH_true = RH_sensor / (1.0546 - 0.00216 * TempC)
 *
 * Constantes calibradas:
 *   - zero_offset_V = 0.826V (offset de salida @ 0%RH)
 *   - slope_V_per_RH = 0.031483 V/%RH (pendiente específica de unidad)
 *
 * Nota: Usa aritmética float (emulada en PIC16, ~ciclos elevados)
 */

// Conversión ADC 10-bit a voltaje (Vref = 5.0V)
static inline float adc_to_voltage(uint16_t adc)
{
    return (5.0f * (float)adc) / 1023.0f;
}

uint16_t hum_read_percent(void)
{
    uint16_t adc = adc_read(ADC_CH_HUM);
    uint8_t temp_c = temp_read_degC();

    // Constantes de calibración específicas de la unidad @ 5V
    const float zero_offset_V = 0.826f;     // Voltaje @ 0%RH
    const float slope_V_per_RH = 0.031483f; // Voltios por %RH

    // Convertir ADC a voltaje
    float vout = adc_to_voltage(adc);

    // Calcular humedad sin compensar (linealización específica)
    float rh_sensor = (vout - zero_offset_V) / slope_V_per_RH;

    // Aplicar compensación de temperatura
    float denom = (1.0546f - 0.00216f * (float)temp_c);
    float rh_true = rh_sensor / denom;

    // Saturar a rango válido [0, 100]
    if (rh_true < 0.0f)
        rh_true = 0.0f;
    if (rh_true > 100.0f)
        rh_true = 100.0f;

    return (uint16_t)rh_true;
}
