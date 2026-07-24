#pragma once
#include <stddef.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "esp_err.h"

esp_err_t ultrasonic_init(gpio_num_t trig, gpio_num_t echo);
float     ultrasonic_read_filtered_cm(size_t samples, gpio_num_t trig, gpio_num_t echo,
                                      uint32_t start_timeout_ms, uint32_t max_distance_cm);