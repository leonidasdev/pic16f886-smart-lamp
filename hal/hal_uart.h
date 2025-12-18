#ifndef HAL_UART_H
#define HAL_UART_H

#include <stdint.h>
#include <stdbool.h>

// Inicializa UART con baudrate especificado
void uart_init(uint32_t baud);

// Envía un byte
void uart_send_byte(uint8_t b);

// Recibe un byte con timeout (~100ms). Retorna true si éxito, false si timeout
bool uart_read_byte_timeout(uint8_t *byte);

// Comprueba si hay datos disponibles
bool uart_data_available(void);

#endif /* HAL_UART_H */
