/**
 * Protocolo UART para control del sistema de calidad del aire.
 *
 * Formato de trama: [HDR][LEN][CMD][DATA...][CRC_LO][CRC_HI]
 *   - HDR: 0xAA (header fijo)
 *   - LEN: longitud = cmd(1) + datos(0-28) + CRC(2) → rango [3, 31]
 *   - CMD: código de comando (CMD_NOISE, CMD_LUX, etc.)
 *   - DATA: 0-28 bytes según comando
 *   - CRC: 2 bytes (CRC-8-CCITT en CRC_LO, 0x00 en CRC_HI)
 *
 * Comandos:
 *   - CMD_NOISE/LUX/CO2/HUM/TEMP: solo envío (sin datos)
 *   - CMD_FAN: 1 byte (velocidad 0-100%)
 *   - CMD_LEDS: 4 bytes (R, G, B, brightness)
 */

#include "../hal/hal_uart.h"
#include "protocol_uart.h"

// Variable global con comando recibido
struct protocol_command_t received_command = {0};

// Flag para indicar nuevo comando disponible
volatile bool new_command_received = false;

// CRC-8-CCITT: polinomio 0x07, init 0x00
uint8_t protocol_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ 0x07;
            }
            else
            {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// Estados de la máquina de parseo
enum protocol_state_t
{
    STATE_HEADER,
    STATE_LENGTH,
    STATE_CMD,
    STATE_DATA,
    STATE_CRC0,
    STATE_CRC1
};

static enum protocol_state_t parse_state = STATE_HEADER;
static uint8_t expected_length = 0;
static uint8_t data_index = 0;
static uint8_t data_buffer[28]; // Buffer temporal para datos
static uint8_t crc_buffer[32];  // Buffer para verificación CRC: [LEN][CMD][DATA...]
static uint8_t crc_buffer_len = 0;

void protocol_init(uint32_t baud)
{
    uart_init(baud);
    parse_state = STATE_HEADER;
}

// Máquina de estados byte a byte (ideal para ISR)
bool protocol_feed_byte(uint8_t byte)
{
    switch (parse_state)
    {
    case STATE_HEADER:
        if (byte == FRAME_HDR)
        {
            parse_state = STATE_LENGTH;
        }
        return false;

    case STATE_LENGTH:
        if (byte < 3 || byte > 31)
        {
            parse_state = STATE_HEADER; // Longitud inválida
            return false;
        }
        expected_length = byte - 3; // datos = length - cmd - crc(2)

        // Protección buffer overflow
        if (expected_length > sizeof(data_buffer))
        {
            parse_state = STATE_HEADER;
            return false;
        }

        // Iniciar buffer CRC con LENGTH
        crc_buffer[0] = byte;
        crc_buffer_len = 1;

        data_index = 0;
        parse_state = STATE_CMD;
        return false;

    case STATE_CMD:
        // Validar comando en rango válido (0x00-0x06)
        if (byte > CMD_TEMP)
        {
            parse_state = STATE_HEADER; // Comando inválido
            return false;
        }

        received_command.type = byte;

        // Agregar CMD a buffer CRC
        crc_buffer[crc_buffer_len++] = byte;

        // Inicializar union a 0 (evita datos residuales)
        received_command.fan_speed = 0;

        // Si no hay datos, ir directo a CRC
        if (expected_length == 0)
        {
            parse_state = STATE_CRC0;
        }
        else
        {
            parse_state = STATE_DATA;
        }
        return false;

    case STATE_DATA:
        data_buffer[data_index] = byte;
        crc_buffer[crc_buffer_len++] = byte; // Agregar a buffer CRC
        data_index++;

        if (data_index >= expected_length)
        {
            // Datos completos - copiar a estructura según comando
            switch (received_command.type)
            {
            case CMD_FAN:
                if (expected_length >= 1)
                {
                    received_command.fan_speed = data_buffer[0];
                }
                break;

            case CMD_LEDS:
                if (expected_length >= 4)
                {
                    received_command.led.red = data_buffer[0];
                    received_command.led.green = data_buffer[1];
                    received_command.led.blue = data_buffer[2];
                    received_command.led.brightness = data_buffer[3];
                }
                break;

                // Otros comandos ignoran datos (solo recepción)
            }
            parse_state = STATE_CRC0;
        }
        return false;

    case STATE_CRC0:
        // Calcular CRC-8 del buffer [LEN][CMD][DATA...]
        {
            uint8_t calculated_crc = protocol_crc8(crc_buffer, crc_buffer_len);
            if (byte != calculated_crc)
            {
                // CRC inválido
                parse_state = STATE_HEADER;
                return false;
            }
        }
        parse_state = STATE_CRC1;
        return false;

    case STATE_CRC1:
        // CRC_HI siempre 0x00 (compatibilidad futura con CRC-16)
        if (byte != 0x00)
        {
            parse_state = STATE_HEADER;
            return false;
        }
        parse_state = STATE_HEADER;
        new_command_received = true; // Señalizar comando válido
        return true;                 // Comando completo
    }

    return false;
}

void protocol_send(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    // Validar parámetros
    if (len > 0 && data == ((void *)0))
        return; // datos nulos
    if (len > 28)
        return; // máximo razonable: 32 - 4(hdr+len+cmd+crc)

    // Construir trama: [HDR][LEN][CMD][DATA...][CRC_LO][CRC_HI]
    uint8_t frame_len = 1 + len + 2; // cmd + datos + CRC(2)

    // Construir buffer para CRC: [LEN][CMD][DATA...]
    uint8_t crc_data[32];
    crc_data[0] = frame_len;
    crc_data[1] = cmd;
    for (uint8_t i = 0; i < len; i++)
    {
        crc_data[2 + i] = data[i];
    }

    // Calcular CRC-8
    uint8_t crc = protocol_crc8(crc_data, 2 + len);

    uart_send_byte(FRAME_HDR); // 0xAA
    uart_send_byte(frame_len); // longitud total
    uart_send_byte(cmd);       // comando

    // Enviar datos byte a byte (hardware limita a buffer 2 niveles)
    for (uint8_t i = 0; i < len; i++)
    {
        uart_send_byte(data[i]);
    }

    // Enviar CRC calculado
    uart_send_byte(crc);  // CRC_LO (CRC-8)
    uart_send_byte(0x00); // CRC_HI (reservado, siempre 0x00)
}

// Parser de trama con timeout (para polling, NO usar en ISR)
// Para ISR usar protocol_feed_byte() byte a byte
bool protocol_receive(uint8_t *cmd, uint8_t *data, uint8_t *len)
{
    if (!uart_data_available())
        return false;

    uint8_t byte;

    // Leer cabecera con timeout
    if (!uart_read_byte_timeout(&byte))
        return false;
    if (byte != FRAME_HDR)
        return false;

    // Leer longitud con timeout
    if (!uart_read_byte_timeout(&byte))
        return false;
    uint8_t length = byte;
    if (length < 3 || length > 31)
        return false; // min=3 (cmd+crc), max=31 (cmd+28datos+crc)

    uint8_t data_len = length - 3;
    if (data_len > 28)
        return false; // Protección buffer overflow

    // Leer comando con timeout
    if (!uart_read_byte_timeout(cmd))
        return false;
    *len = data_len;

    // Leer datos con timeout
    for (uint8_t i = 0; i < data_len; i++)
    {
        if (!uart_read_byte_timeout(&data[i]))
            return false;
    }

    // Verificar CRC
    uint8_t crc_lo, crc_hi;
    if (!uart_read_byte_timeout(&crc_lo))
        return false;
    if (!uart_read_byte_timeout(&crc_hi))
        return false;
    (void)crc_lo;
    (void)crc_hi; // Actualmente no se valida

    return true;
}
