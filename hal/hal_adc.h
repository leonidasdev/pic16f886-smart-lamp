#ifndef HAL_ADC_H
#define HAL_ADC_H

#include <stdint.h>

// Definición de canales según tu PCB
#define ADC_CH_TEMP 0  // AN0 → RA0
#define ADC_CH_HUM 1   // AN1 → RA1
#define ADC_CH_NOISE 2 // AN2 → RA2

// Inicializa el módulo ADC
void adc_init(void);

// Lee un canal analógico y devuelve un valor de 10 bits (0–1023)
uint16_t adc_read(uint8_t channel);

#endif /* HAL_ADC_H */
