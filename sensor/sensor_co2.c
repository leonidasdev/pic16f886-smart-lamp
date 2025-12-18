#include "../hal/hal_i2c.h"
#include "sensor_co2.h"

// Dirección I2C del iAQ-Core (ejemplo: 0x5A, confirmar en hoja técnica)
#define IAQCORE_ADDR 0x5A

// iAQ-Core requiere 5 minutos (300 segundos) de precalentamiento
// Contador de warmup: se incrementa cada 5 segundos
// 300s / 5s = 60 lecturas necesarias
#define WARMUP_READS 60

static uint8_t warmup_counter = 0;
static uint8_t is_warmed_up = 0;

void co2_init(void)
{
    // El iAQ-Core suele entregar datos en streaming, no requiere configuración inicial
    warmup_counter = 0;
    is_warmed_up = 0;
}

void co2_update_warmup(void)
{
    if (!is_warmed_up)
    {
        warmup_counter++;
        if (warmup_counter >= WARMUP_READS)
        {
            is_warmed_up = 1; // Sensor listo después de 5 minutos
        }
    }
}

uint16_t co2_read_ppm(void)
{
    // Durante el período de warmup, retornar valor indicativo
    if (!is_warmed_up)
    {
        return 0xFFFF; // Valor especial: sensor aún no listo
    }

    uint16_t ppm = 0;
    uint8_t buffer[9]; // iAQ-Core: 9 bytes (status, CO2_MSB, CO2_LSB, TVOC_MSB, TVOC_LSB, resist_MSB, resist_LSB, resist2_MSB, resist2_LSB)

    hal_i2c_start();

    // Validar ACK del dispositivo
    if (hal_i2c_write((IAQCORE_ADDR << 1) | 1) != 0)
    {
        hal_i2c_stop();
        return 0xFFFF; // Error: dispositivo no responde
    }

    // Leer 9 bytes del protocolo completo
    for (uint8_t i = 0; i < 9; i++)
    {
        buffer[i] = hal_i2c_read(i < 8 ? 1 : 0); // ACK todos excepto último
    }
    hal_i2c_stop();

    // Verificar status byte (byte 0)
    // Bit 0: error, Bit 1: busy, Bit 7-6: runin
    if (buffer[0] & 0x01)
    {
        return 0xFFFF; // Error reportado por sensor
    }

    // Extraer CO2 (bytes 1-2)
    ppm = ((uint16_t)buffer[1] << 8) | buffer[2];
    return ppm;
}
