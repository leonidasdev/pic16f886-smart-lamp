#include "scheduler.h"

static uint16_t noise_ticks = 0;
static uint16_t env_ticks = 0;
static bool noise_flag = false;
static bool env_flag = false;

void scheduler_init(void)
{
    noise_ticks = 0;
    env_ticks = 0;
    noise_flag = false;
    env_flag = false;
}

void scheduler_tick_10ms(void)
{
    // Cada 10 ms: acumular para ruido
    noise_ticks++;
    if (noise_ticks >= 100)
    { // 100 * 10 ms = 1 s
        noise_ticks = 0;
        noise_flag = true;
    }
}

void scheduler_tick_100ms(void)
{
    // Cada ~100 ms: acumular para sensores ambientales
    env_ticks++;
    if (env_ticks >= 50)
    { // 50 * 100 ms ≈ 5 s
        env_ticks = 0;
        env_flag = true;
    }
}

bool scheduler_noise_ready(void)
{
    if (noise_flag)
    {
        noise_flag = false;
        return true;
    }
    return false;
}

bool scheduler_env_ready(void)
{
    if (env_flag)
    {
        env_flag = false;
        return true;
    }
    return false;
}
