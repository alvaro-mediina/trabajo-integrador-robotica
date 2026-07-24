#pragma once
#include <stdint.h>
#include "driver/ledc.h"
#include "driver/gpio.h"

#define MAX_DUTY  1023U

typedef enum {
    MOTOR_FWD,
    MOTOR_BWD
} motor_dir_t;

typedef struct {
    ledc_channel_t channel;
    gpio_num_t     pin_pwm;
    gpio_num_t     pin_dir;
} motor_t;

void motors_init(motor_t *m1, motor_t *m2);
void motor_set(motor_t *m, uint32_t duty, motor_dir_t dir);