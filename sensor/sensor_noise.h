#ifndef SENSOR_NOISE_H
#define SENSOR_NOISE_H

#include <stdint.h>

typedef enum
{
    NOISE_LOW = 0,
    NOISE_MED = 1,
    NOISE_HIGH = 2
} noise_cat_t;

// Devuelve la categoría de ruido según MO-10
noise_cat_t noise_read_category(void);

#endif /* SENSOR_NOISE_H */
