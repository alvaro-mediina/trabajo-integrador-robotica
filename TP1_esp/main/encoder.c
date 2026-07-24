#include "encoder.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

static const char *TAG = "encoder";

static volatile uint32_t count_right = 0;
static volatile uint32_t count_left  = 0;
static float rpm_right = 0.0f;
static float rpm_left  = 0.0f;

static void IRAM_ATTR isr_right(void *arg) { count_right++; }
static void IRAM_ATTR isr_left(void *arg)  { count_left++;  }

static void speed_timer_cb(void *arg)
{
    portDISABLE_INTERRUPTS();
    uint32_t cr = count_right; count_right = 0;
    uint32_t cl = count_left;  count_left  = 0;
    portENABLE_INTERRUPTS();

    rpm_right = (float)cr / PPR * (60000.0f / PERIOD_MS);
    rpm_left  = (float)cl / PPR * (60000.0f / PERIOD_MS);
}

void encoders_init(void)
{
    gpio_config_t cfg = {
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_POSEDGE,
        .pin_bit_mask = (1ULL << PIN_ENC_RIGHT) | (1ULL << PIN_ENC_LEFT),
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_ENC_RIGHT, isr_right, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_ENC_LEFT,  isr_left,  NULL));

    esp_timer_handle_t timer;
    const esp_timer_create_args_t timer_args = {
        .callback = speed_timer_cb,
        .name     = "encoder_speed",
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer, PERIOD_MS * 1000ULL));

    ESP_LOGI(TAG, "Inicializado (RIGHT=%d, LEFT=%d, PPR=%d, periodo=%dms)",
             PIN_ENC_RIGHT, PIN_ENC_LEFT, PPR, PERIOD_MS);
}

float encoder_get_rpm_right(void) { return rpm_right; }
float encoder_get_rpm_left(void)  { return rpm_left;  }