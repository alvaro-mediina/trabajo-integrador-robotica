#pragma once
#include "driver/gpio.h"

#define PIN_ENC_RIGHT   GPIO_NUM_21
#define PIN_ENC_LEFT    GPIO_NUM_26
#define PPR             20
#define PERIOD_MS       1000

void  encoders_init(void);
float encoder_get_rpm_right(void);
float encoder_get_rpm_left(void);