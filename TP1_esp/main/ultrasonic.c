#include "ultrasonic.h"
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_log.h"
#include "rom/ets_sys.h"

#define delay_us(us)    ets_delay_us(us)
#define SAMPLE_DELAY_MS 10

static const char *TAG = "ultrasonic";

static bool wait_for_level(gpio_num_t pin, int level, uint32_t timeout_us)
{
    int cur = gpio_get_level(pin);
    uint64_t start = esp_timer_get_time();
    while (cur != level) {
        if ((esp_timer_get_time() - start) > timeout_us)
            return false;
        cur = gpio_get_level(pin);
    }
    return true;
}

static esp_err_t ultrasonic_read_cm(float *out_cm, gpio_num_t trig, gpio_num_t echo,
                                    uint32_t start_timeout_ms, uint32_t max_distance_cm)
{
    if (!out_cm) return ESP_ERR_INVALID_ARG;

    gpio_set_level(trig, 0);
    delay_us(2);
    gpio_set_level(trig, 1);
    delay_us(10);
    gpio_set_level(trig, 0);

    if (!wait_for_level(echo, 1, start_timeout_ms * 1000UL))
        return ESP_ERR_TIMEOUT;

    uint64_t t_start = esp_timer_get_time();
    uint32_t max_us  = max_distance_cm * 58U * 2U;

    if (!wait_for_level(echo, 0, max_us))
        return ESP_ERR_TIMEOUT;

    uint64_t dt_us = esp_timer_get_time() - t_start;
    *out_cm = (float)dt_us / 58.0f;
    return ESP_OK;
}

esp_err_t ultrasonic_init(gpio_num_t trig, gpio_num_t echo)
{
    gpio_config_t io_trig = {
        .pin_bit_mask = 1ULL << trig,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_trig));
    gpio_set_level(trig, 0);

    gpio_config_t io_echo = {
        .pin_bit_mask = 1ULL << echo,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_echo));

    ESP_LOGI(TAG, "Inicializado (TRIG=%d, ECHO=%d)", trig, echo);
    return ESP_OK;
}

float ultrasonic_read_filtered_cm(size_t samples, gpio_num_t trig, gpio_num_t echo,
                                   uint32_t start_timeout_ms, uint32_t max_distance_cm)
{
    float  acc = 0.0f;
    size_t ok  = 0;
    for (size_t i = 0; i < samples; i++) {
        float cm = 0.0f;
        if (ultrasonic_read_cm(&cm, trig, echo, start_timeout_ms, max_distance_cm) == ESP_OK) {
            acc += cm;
            ok++;
        }
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_DELAY_MS));
    }
    return (ok == 0) ? -1.0f : acc / (float)ok;
}