/*
 * File: main.c
 * Target: PIC16F886 @ 20 MHz
 * Project: SMA-LAMP (LMDE-MA)
 */

#pragma config FOSC = HS  // Oscilador con cristal 20 MHz
#pragma config WDTE = OFF // Watchdog deshabilitado
#pragma config PWRTE = ON // Power-up Timer habilitado
#pragma config MCLRE = ON // MCLR habilitado
#pragma config CP = OFF, CPD = OFF
#pragma config BOREN = ON // Brown-out Reset habilitado
#pragma config IESO = OFF, FCMEN = OFF
#pragma config LVP = OFF // Programación en bajo voltaje deshabilitada

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#define _XTAL_FREQ 20000000UL

// Interfaces de aplicación (capas superiores)
#include "protocol/protocol_uart.h"
#include "protocol/scheduler.h"

// HAL
#include "hal/hal_adc.h"
#include "hal/hal_i2c.h"
#include "hal/hal_spi.h"
#include "hal/hal_pwm.h"
#include "hal/hal_eeprom.h"

// Sensores
#include "sensor/sensor_noise.h"
#include "sensor/sensor_temp.h"
#include "sensor/sensor_hum.h"
#include "sensor/sensor_lux.h"
#include "sensor/sensor_co2.h"

// Actuadores
#include "actuador/actuator_led.h"
#include "actuador/actuator_fan.h"

// Constantes de configuración
#define UART_BAUDRATE 9600
#define FAN_MAX_SPEED 100

// ------------------------
// Inicialización del sistema
// ------------------------

static void init_timers(void)
{
    // Timer0: tick cada ~10 ms
    // Fosc/4 = 5 MHz → tick prescaler 1:256 = 51.2 us
    // 10 ms / 51.2 us ≈ 195 → preload TMR0 = 256 - 195 = 61
    OPTION_REG = 0b10000111; // nRBPU=1 (pull-ups off), PSA=0, PS=1:256, T0CS=0
    TMR0 += 61;
    INTCONbits.TMR0IE = 1;
    INTCONbits.TMR0IF = 0;

    // Timer1: overflow cada ~104.858 ms (prescaler 1:8)
    T1CON = 0b00110001; // TMR1ON=1, TMR1CS=0 (Fosc/4), T1CKPS=11 (1:8), Gate OFF
    PIR1bits.TMR1IF = 0;
    PIE1bits.TMR1IE = 1;

    // Interrupciones globales
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;
}

static void init_pins_min(void)
{
    // UART: RC6 (TX) salida, RC7 (RX) entrada
    TRISCbits.TRISC6 = 0;
    TRISCbits.TRISC7 = 1;

    // I2C: RC3 (SCL) y RC4 (SDA) entradas (MSSP controla niveles)
    TRISCbits.TRISC3 = 1;
    TRISCbits.TRISC4 = 1;

    // PWM: RC2 (CCP1) salida para ventilador
    TRISCbits.TRISC2 = 0;

    // SPI bit-bang: RC0 (CLK) salida, RC1 (SDO) salida
    TRISCbits.TRISC0 = 0;
    TRISCbits.TRISC1 = 0;

    // ADC: RA0, RA1, RA2 como entradas analógicas (configurado en hal_adc)
    TRISAbits.TRISA0 = 1;
    TRISAbits.TRISA1 = 1;
    TRISAbits.TRISA2 = 1;
}

// ------------------------
// Variables del runtime
// ------------------------

// volatile: variable compartida entre ISR y main loop
static volatile noise_cat_t noise_max_cat = NOISE_LOW;

// Estado actual de actuadores (para guardado periódico en EEPROM)
static uint8_t current_led_r = 128;
static uint8_t current_led_g = 128;
static uint8_t current_led_b = 128;
static uint8_t current_led_brightness = 16;
static uint8_t current_fan_speed = 0;

// ------------------------
// Funciones auxiliares
// ------------------------

// Guarda el estado actual de actuadores en EEPROM
static void save_actuators_state(void)
{
    eeprom_write(0x00, current_led_brightness);
    eeprom_write(0x01, current_led_r);
    eeprom_write(0x02, current_led_g);
    eeprom_write(0x03, current_led_b);
    eeprom_write(0x04, current_fan_speed);
}

// ------------------------
// ISR
// ------------------------

void __interrupt() isr(void)
{
    // Timer0 → 10 ms
    if (INTCONbits.TMR0IF && INTCONbits.TMR0IE)
    {
        INTCONbits.TMR0IF = 0;
        TMR0 += 61; // Recarga compensada: evita deriva por latencia ISR

        // Tick scheduler
        scheduler_tick_10ms();

        // Muestreo de ruido y acumulación del máximo de la ventana de 1s
        noise_cat_t cat = noise_read_category();
        if (cat > noise_max_cat)
            noise_max_cat = cat;
    }

    // Timer1 → ~100 ms
    if (PIR1bits.TMR1IF && PIE1bits.TMR1IE)
    {
        PIR1bits.TMR1IF = 0;
        scheduler_tick_100ms();
    }

    // UART RX → Recepción de comandos por máquina de estados
    if (PIR1bits.RCIF && PIE1bits.RCIE)
    {
        // Alimentar máquina de estados con byte recibido
        if (protocol_feed_byte(RCREG))
        {
            // Comando completo recibido - procesarlo en main loop
            // No ejecutar acciones largas en ISR, solo marcar flag
        }
        // PIR1bits.RCIF se limpia automáticamente al leer RCREG
    }
}

// ------------------------
// main()
// ------------------------

int main(void)
{
    // Inicializaciones básicas
    init_pins_min();
    init_timers();
    scheduler_init();

    // Comunicación
    protocol_init(UART_BAUDRATE);
    PIE1bits.RCIE = 1; // Habilitar interrupción UART RX para protocol_feed_byte()

    // Inicializar periféricos HAL UNA SOLA VEZ
    adc_init();     // Para sensores analógicos (temp, hum, noise)
    hal_i2c_init(); // Para sensores I2C (lux, CO2)
    spi_init();     // Para actuador LED SK9822
    pwm_init();     // Para actuador ventilador

    // Inicialización de sensores con estado interno
    co2_init(); // Inicializa contador de warmup (5 min)

    // Actuadores (ya no inicializan periféricos internamente)
    led_init();
    fan_init();

    // Restauración de configuración (ENC-30)
    uint8_t last_brightness = eeprom_read(0x00);
    uint8_t last_r = eeprom_read(0x01);
    uint8_t last_g = eeprom_read(0x02);
    uint8_t last_b = eeprom_read(0x03);
    uint8_t last_fan = eeprom_read(0x04);

    // Validar brightness [0, 31]
    if (last_brightness > 31)
        last_brightness = 16; // Default si inválido

    // Validar datos de EEPROM
    // Si EEPROM está vacía (primer encendido), aplica defaults
    // Default: luz blanca intermedia, ventilador apagado
    if (last_r == 0xFF && last_g == 0xFF && last_b == 0xFF)
    {
        current_led_r = 128;
        current_led_g = 128;
        current_led_b = 128;
        current_led_brightness = 16;
        current_fan_speed = 0;
    }
    else
    {
        // Validar rango del ventilador antes de aplicar
        if (last_fan > FAN_MAX_SPEED)
            last_fan = 0;
        current_led_r = last_r;
        current_led_g = last_g;
        current_led_b = last_b;
        current_led_brightness = last_brightness;
        current_fan_speed = last_fan;
    }

    // Aplicar estado restaurado
    led_set_color(current_led_r, current_led_g, current_led_b, current_led_brightness);
    fan_set_speed(current_fan_speed);

    // Estado inicial del ruido acumulado
    noise_max_cat = NOISE_LOW;

    // Bucle principal
    while (1)
    {
        // Reporte de ruido cada 1 s con la categoría máxima alcanzada
        if (scheduler_noise_ready())
        {
            uint8_t dNoise[1] = {(uint8_t)noise_max_cat};
            protocol_send(CMD_NOISE, dNoise, 1);
            noise_max_cat = NOISE_LOW; // reset ventana
        }

        // Reporte de sensores ambientales cada ~5 s
        if (scheduler_env_ready())
        {
            // Guardar estado de actuadores en EEPROM periódicamente
            // (protege contra pérdida de energía)
            save_actuators_state();

            // Actualizar contador de warmup del sensor CO2 (requiere 5 min)
            co2_update_warmup();

            // Lecturas de sensores
            uint8_t tC = temp_read_degC();     // Temperatura en ºC (0-255)
            uint16_t hum = hum_read_percent(); // Humedad en % (0-100)
            uint16_t lux = lux_read();         // Luminosidad en lux (0-65535)
            uint16_t ppm = co2_read_ppm();     // CO2 en ppm (0-65535, 0xFFFF=warmup)

            // Validar rangos razonables
            if (hum > 100)
                hum = 100; // Humedad máximo 100%

            // Construir tramas (big-endian para valores de 16 bits)
            uint8_t dT[1] = {tC};
            uint8_t dH[2] = {(uint8_t)(hum >> 8), (uint8_t)(hum & 0xFF)};
            uint8_t dL[2] = {(uint8_t)(lux >> 8), (uint8_t)(lux & 0xFF)};
            uint8_t dC[2] = {(uint8_t)(ppm >> 8), (uint8_t)(ppm & 0xFF)};

            // Enviar comandos con validación automática en protocol_send
            protocol_send(CMD_TEMP, dT, 1);
            protocol_send(CMD_HUM, dH, 2);
            protocol_send(CMD_LUX, dL, 2);
            protocol_send(CMD_CO2, dC, 2);
        }

        // Recepción y aplicación de comandos desde SMA-COMP
        // Los comandos son parseados en ISR por protocol_feed_byte()
        // El flag volatile 'new_command_received' se activa en ISR al completar trama

        if (new_command_received)
        {
            new_command_received = false; // Limpiar flag
            // Validar tipo de comando
            if (received_command.type > CMD_TEMP)
            {
                // Comando inválido, ignorar
                continue;
            }

            switch (received_command.type)
            {
            case CMD_FAN: // 1 byte: porcentaje 0–100
            {
                uint8_t speed = received_command.fan_speed;
                if (speed > 100)
                    speed = 100; // limitar a rango válido
                current_fan_speed = speed;
                fan_set_speed(current_fan_speed);
                // Se guardará automáticamente cada 5s
            }
            break;

            case CMD_LEDS: // 4 bytes: R, G, B, Brillo (0–31)
            {
                current_led_r = received_command.led.red;
                current_led_g = received_command.led.green;
                current_led_b = received_command.led.blue;
                current_led_brightness = received_command.led.brightness & 0x1F;
                led_set_color(current_led_r, current_led_g, current_led_b, current_led_brightness);
                // Se guardará automáticamente cada 5s
            }
            break;

            default:
                // Otros comandos no requieren acción en SMA-LAMP
                break;
            }
        }
    }
}
