/**
 * @file actuator.c
 * @brief AntigravityX — Actuator pre-charge and reset implementation.
 *
 * No IRAM placement required here: all actuator functions are called
 * from normal task context, never from the ISR. The ISR fires the
 * actuator pin directly via a register write in main.c.
 */

#include "actuator.h"
#include "antigravityx_config.h"

#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ACTUATOR";

static volatile agx_actuator_state_t s_state = AGX_ACT_STATE_UNINIT;

/* ─── Helpers ─────────────────────────────────────────────────────────── */

static void set_actuator_pin(bool high)
{
    gpio_set_level(AGX_PIN_ACTUATOR, high ? 1 : 0);
}

static void set_led(int pin, bool on)
{
    gpio_set_level(pin, on ? 1 : 0);
}

/* ─── Public API ──────────────────────────────────────────────────────── */

esp_err_t actuator_init(void)
{
    s_state = AGX_ACT_STATE_PRECHARGING;

    /* Configure actuator trigger pin — output, start LOW (de-energised) */
    gpio_config_t trig_cfg = {
        .pin_bit_mask = (1ULL << AGX_PIN_ACTUATOR),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,  /* Ensure MOSFET gate starts low */
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&trig_cfg);
    set_actuator_pin(false);

    /* Configure READY sense pin — input, pull-down */
    gpio_config_t ready_cfg = {
        .pin_bit_mask = (1ULL << AGX_PIN_ACTUATOR_READY),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&ready_cfg);

    /* Configure status LEDs */
    gpio_config_t led_cfg = {
        .pin_bit_mask = (1ULL << AGX_PIN_LED_ARMED) | (1ULL << AGX_PIN_LED_TRIGGERED),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&led_cfg);
    set_led(AGX_PIN_LED_ARMED,     false);
    set_led(AGX_PIN_LED_TRIGGERED, false);

    /* Wait for pre-charge rail — READY pin must assert within 500 ms.
     * The boost converter / supercap is energised by the PCB power rail;
     * this just waits for it to reach operating voltage. */
    const int timeout_ticks = pdMS_TO_TICKS(500);
    int       elapsed       = 0;
    const int poll_ms       = 10;

    ESP_LOGI(TAG, "Waiting for actuator rail pre-charge...");
    while (!gpio_get_level(AGX_PIN_ACTUATOR_READY)) {
        if (elapsed >= timeout_ticks) {
            ESP_LOGE(TAG, "Pre-charge READY timeout after 500 ms");
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(poll_ms));
        elapsed += pdMS_TO_TICKS(poll_ms);
    }

    ESP_LOGI(TAG, "Actuator rail pre-charged and READY");
    s_state = AGX_ACT_STATE_ARMED;
    set_led(AGX_PIN_LED_ARMED, true);

    return ESP_OK;
}

void actuator_hold_and_reset(void)
{
    /* Called from STATUS task after ISR has already fired the pin.
     * At this point, AGX_PIN_ACTUATOR is HIGH (set by ISR register write).
     * Hold for the configured period so the actuator mechanism completes. */
    s_state = AGX_ACT_STATE_TRIGGERED;
    set_led(AGX_PIN_LED_TRIGGERED, true);
    set_led(AGX_PIN_LED_ARMED,     false);

    ESP_LOGI(TAG, "Actuator triggered — holding for %d ms", AGX_ACTUATOR_HOLD_MS);
    vTaskDelay(pdMS_TO_TICKS(AGX_ACTUATOR_HOLD_MS));

    /* De-energise actuator */
    set_actuator_pin(false);
    ESP_LOGI(TAG, "Actuator de-energised");

    s_state = AGX_ACT_STATE_RESETTING;

    /* Blink triggered LED twice to indicate event */
    for (int i = 0; i < 2; i++) {
        set_led(AGX_PIN_LED_TRIGGERED, false);
        vTaskDelay(pdMS_TO_TICKS(150));
        set_led(AGX_PIN_LED_TRIGGERED, true);
        vTaskDelay(pdMS_TO_TICKS(150));
    }
    set_led(AGX_PIN_LED_TRIGGERED, false);

    /* Re-arm: system is ready for next event */
    s_state = AGX_ACT_STATE_ARMED;
    set_led(AGX_PIN_LED_ARMED, true);
    ESP_LOGI(TAG, "System re-armed");
}

agx_actuator_state_t actuator_get_state(void)
{
    return s_state;
}

void actuator_disarm(void)
{
    set_actuator_pin(false);
    set_led(AGX_PIN_LED_ARMED,     false);
    set_led(AGX_PIN_LED_TRIGGERED, false);
    s_state = AGX_ACT_STATE_UNINIT;
    ESP_LOGW(TAG, "Actuator disarmed by software request");
}

