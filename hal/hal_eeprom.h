#ifndef HAL_EEPROM_H
#define HAL_EEPROM_H

#include <stdint.h>

// Escribe un byte en la dirección indicada
void eeprom_write(uint8_t addr, uint8_t val);

// Lee un byte de la dirección indicada
uint8_t eeprom_read(uint8_t addr);

#endif /* HAL_EEPROM_H */
