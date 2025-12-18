#include <xc.h>
#include "hal_adc.h"

#define _XTAL_FREQ 20000000UL

void adc_init(void)
{
    // Configuración de entradas analógicas
    ANSEL = 0b00000111; // AN0, AN1, AN2 activados
    ANSELH = 0x00;

    // Referencia VDD/VSS, resultado justificado a la derecha
    ADCON1 = 0b10000000; // ADFM=1 (right), VCFG1:VCFG0=00 (VDD/VSS)

    // Encender ADC, canal inicial AN0, clock FOSC/32
    ADCON0 = 0b10000001; // ADCS=10 (FOSC/32, TAD=1.6µs), CHS=0000 (AN0), ADON=1
}

uint16_t adc_read(uint8_t channel)
{
    uint16_t result;

    // Validar canal (0-15 máximo para PIC16F886)
    if (channel > 15)
        return 0;

    // Protección contra interrupciones durante conversión ADC
    // Evita race condition con ISR que también usa el ADC
    INTCONbits.GIE = 0; // Deshabilitar interrupciones globales

    // Seleccionar canal
    ADCON0 &= 0b11000001;     // limpiar CHS
    ADCON0 |= (channel << 2); // set CHS

    __delay_us(10); // tiempo de adquisición

    GO_nDONE = 1; // iniciar conversión
    while (GO_nDONE)
        ; // esperar fin

    result = ((uint16_t)ADRESH << 8) | ADRESL;

    INTCONbits.GIE = 1; // Rehabilitar interrupciones

    return result;
}
