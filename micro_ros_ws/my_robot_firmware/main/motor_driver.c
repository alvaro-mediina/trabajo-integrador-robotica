#include "motor_driver.h"
#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "motor";

static void motor_init(motor_t *m)
{
    ledc_channel_config_t ch = {0};
    ch.gpio_num   = m->pin_pwm;
    ch.speed_mode = LEDC_HIGH_SPEED_MODE;
    ch.channel    = m->channel;
    ch.intr_type  = LEDC_INTR_DISABLE;
    ch.timer_sel  = LEDC_TIMER_0;
    ch.duty       = 0;
    ESP_ERROR_CHECK(ledc_channel_config(&ch));

    gpio_reset_pin(m->pin_dir);
    gpio_set_direction(m->pin_dir, GPIO_MODE_OUTPUT);
    gpio_set_level(m->pin_dir, 0);
}

void motors_init(motor_t *m1, motor_t *m2)
{
    ledc_timer_config_t timer = {0};
    timer.speed_mode      = LEDC_HIGH_SPEED_MODE;
    timer.duty_resolution = LEDC_TIMER_10_BIT;
    timer.timer_num       = LEDC_TIMER_0;
    timer.freq_hz         = 500;
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    motor_init(m1);
    motor_init(m2);

    ESP_LOGI(TAG, "Motores inicializados");
}

void motor_set(motor_t *m, uint32_t duty, motor_dir_t dir)
{
    if (dir == MOTOR_FWD) {
        gpio_set_level(m->pin_dir, 0);
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, m->channel, duty);
    } else {
        gpio_set_level(m->pin_dir, 1);
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, m->channel, MAX_DUTY - duty);
    }
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, m->channel);
}