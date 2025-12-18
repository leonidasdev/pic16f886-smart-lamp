#include "../hal/hal_i2c.h"
#include "sensor_lux.h"

#define VEML7700_ADDR 0x10

uint16_t lux_read(void)
{
    uint16_t raw = 0;

    hal_i2c_start();

    // Validar ACK en dirección de escritura
    if (hal_i2c_write((VEML7700_ADDR << 1) | 0) != 0)
    {
        hal_i2c_stop();
        return 0xFFFF; // Error: dispositivo no responde
    }

    // Validar ACK en escritura de registro
    if (hal_i2c_write(0x04) != 0)
    {
        hal_i2c_stop();
        return 0xFFFF; // Error: escritura de registro fallida
    }

    hal_i2c_rstart();

    // Validar ACK en dirección de lectura
    if (hal_i2c_write((VEML7700_ADDR << 1) | 1) != 0)
    {
        hal_i2c_stop();
        return 0xFFFF; // Error: lectura fallida
    }

    uint8_t lsb = hal_i2c_read(1); // ACK
    uint8_t msb = hal_i2c_read(0); // NACK
    hal_i2c_stop();

    raw = ((uint16_t)msb << 8) | lsb;
    return raw;
}
