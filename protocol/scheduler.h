#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <stdint.h>
#include <stdbool.h>

// Inicializa los temporizadores del sistema
void scheduler_init(void);

// Debe llamarse en la ISR de Timer0 cada 10 ms
void scheduler_tick_10ms(void);

// Debe llamarse en la ISR de Timer1 cada ~100 ms
void scheduler_tick_100ms(void);

// Comprueba si toca enviar ruido (cada 1 s)
bool scheduler_noise_ready(void);

// Comprueba si toca enviar sensores ambientales (cada 5 s)
bool scheduler_env_ready(void);

#endif /* SCHEDULER_H */
