#include "../hal/hal_spi.h"
#include "actuator_led.h"

// Número de LEDs en la tira (ajusta según tu prototipo)
#define N_LEDS 1

void led_init(void)
{
    // Estado inicial: blanco intermedio, brillo medio
    led_set_color(128, 128, 128, 16);
}

static void sk9822_send_header(void)
{
    for (uint8_t i = 0; i < 4; i++)
        spi_send_byte(0x00);
}

static void sk9822_send_tail(void)
{
    for (uint8_t i = 0; i < 4; i++)
        spi_send_byte(0xFF);
}

static void sk9822_send_led(uint8_t brightness, uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t gb = 0xE0 | (brightness & 0x1F);
    spi_send_byte(gb);
    spi_send_byte(b);
    spi_send_byte(g);
    spi_send_byte(r);
}

void led_set_color(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness)
{
    sk9822_send_header();
    for (uint8_t i = 0; i < N_LEDS; i++)
    {
        sk9822_send_led(brightness, r, g, b);
    }
    sk9822_send_tail();
}
