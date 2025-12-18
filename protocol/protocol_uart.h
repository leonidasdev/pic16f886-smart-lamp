#ifndef PROTOCOL_UART_H
#define PROTOCOL_UART_H

#include <stdint.h>
#include <stdbool.h>

// Cabecera de trama
#define FRAME_HDR 0xAA

// CRC-8 con polinomio 0x07 (CRC-8-CCITT)
// Nota: Solo usamos CRC_LO (1 byte), CRC_HI siempre 0x00 para compatibilidad

// Comandos definidos en requisitos
#define CMD_NOISE 0x00
#define CMD_LUX 0x01
#define CMD_CO2 0x02
#define CMD_HUM 0x03
#define CMD_FAN 0x04
#define CMD_LEDS 0x05
#define CMD_TEMP 0x06

// Estructura para comando recibido (usado por máquina de estados)
struct protocol_command_t
{
    uint8_t type; // Tipo de comando (CMD_FAN, CMD_LEDS, etc)
    union
    {
        uint8_t fan_speed; // Para CMD_FAN: velocidad 0-100
        struct
        { // Para CMD_LEDS: color RGB + brillo
            uint8_t red;
            uint8_t green;
            uint8_t blue;
            uint8_t brightness;
        } led;
    };
};

// Variable global con último comando recibido completo
extern struct protocol_command_t received_command;

// Flag para indicar nuevo comando recibido (setear en ISR, limpiar en main)
extern volatile bool new_command_received;

// Inicializa UART y protocolo
void protocol_init(uint32_t baud);

// Calcula CRC-8 de un buffer (polinomio 0x07)
uint8_t protocol_crc8(const uint8_t *data, uint8_t len);

// Alimenta máquina de estados con un byte (para usar en ISR)
// Retorna true cuando comando completo está disponible en received_command
bool protocol_feed_byte(uint8_t byte);

// Envía una trama con comando y datos
void protocol_send(uint8_t cmd, const uint8_t *data, uint8_t len);

// Intenta leer una trama recibida; devuelve true si válida
bool protocol_receive(uint8_t *cmd, uint8_t *data, uint8_t *len);

#endif /* PROTOCOL_UART_H */
