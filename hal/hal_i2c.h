#ifndef HAL_I2C_H
#define HAL_I2C_H

#include <stdint.h>

// Inicializa el MSSP en modo I2C maestro
void hal_i2c_init(void);

// Wrappers HAL que llaman a las funciones del profesorado
void hal_i2c_start(void);
void hal_i2c_stop(void);
void hal_i2c_rstart(void);
uint8_t hal_i2c_write(uint8_t data);
uint8_t hal_i2c_read(uint8_t ack);

#endif /* HAL_I2C_H */
