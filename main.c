/**
 * @file main.c
 * @brief AntigravityX — Main application entry point.
 *
 * Critical path (from physical free-fall to actuator trigger):
 *
 *   [Gravity removed from device]
 *       ↓  625 µs/cycle × AGX_FF_DURATION_VAL cycles  ← HW debounce (IMU)
 *   [ICM-42688-P INT1 asserts]
 *       ↓  nanoseconds                                 ← trace propagation
 *   [ESP32 GPIO edge interrupt fires]
 *       ↓  < 1 µs (IRAM-safe, high-priority ISR)       ← freefall_isr_handler
 *   [REG_WRITE(GPIO_OUT_W1TS_REG) executes]
 *       ↓  sub-µs                                      ← MOSFET gate charges
 *   [Actuator fires]
 *
 * Total MCU + IMU latency: dominated entirely by AGX_FF_DURATION_VAL.
 * With duration=2 cycles at 1600 Hz: hardware debounce ≈ 1.25 ms.
 * Remaining ESP32-side latency: < 2 µs.
 *
 * ─── IRAM ISR Design Rules ──────────────────────────────────────────────
 *  1. IRAM_ATTR — code and any called functions must reside in IRAM.
 *  2. Direct register write — no gpio_set_level(), no HAL function.
 *  3. No I2C/SPI reads — clearing the IMU interrupt flag is deferred.
 *  4. Minimal queue send — only to notify the STATUS task after pin toggle.
 *  5. No logging — ESP_LOG functions are NOT IRAM-safe.
 */

#include "antigravityx_config.h"
#include "imu_icm42688.h"
#include "actuator.h"

#include "esp_log.h"
#include "esp_attr.h"           /* IRAM_ATTR */
#include "soc/gpio_reg.h"       /* GPIO_OUT_W1TS_REG, GPIO_OUT_W1TC_REG */
#include "soc/soc.h"            /* REG_WRITE */
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

static const char *TAG = "AntigravityX";

/* ─── Globals ─────────────────────────────────────────────────────────── */

static icm42688_dev_t s_imu;
static TaskHandle_t   s_status_task_handle = NULL;

/* ─── ISR: THE ENTIRE CRITICAL PATH ──────────────────────────────────── */

/**
 * @brief  IRAM-resident free-fall ISR.
 *
 * Called when ICM-42688-P INT1 asserts a rising edge on AGX_PIN_IMU_INT1.
 *
 * This function does exactly two things:
 *   1. Write 1 to the actuator's bit in GPIO_OUT_W1TS_REG (atomic set HIGH).
 *   2. Notify the STATUS task so it can run the hold/reset sequence.
 *
 * It does NOT:
 *   - Read any IMU registers (SPI transfer would block here)
 *   - Call any ESP-IDF GPIO helper (not guaranteed IRAM-safe)
 *   - Log anything (esp_log uses flash-resident code)
 *   - Run any filter or threshold comparison (IMU hardware did that already)
 */
static void IRAM_ATTR freefall_isr_handler(void *arg)
{
    /* ── ACTUATOR FIRE ── single register write: tens of nanoseconds ── */
    REG_WRITE(GPIO_OUT_W1TS_REG, (1UL << AGX_PIN_ACTUATOR));

    /* ── Deferred wake: notify STATUS task to run hold + reset ─────── */
    BaseType_t higher_prio_woken = pdFALSE;
    vTaskNotifyGiveFromISR(s_status_task_handle, &higher_prio_woken);

    /* Yield to the notified task immediately if it has higher priority */
    if (higher_prio_woken) {
        portYIELD_FROM_ISR();
    }
}

/* ─── Status Task ─────────────────────────────────────────────────────── */

/**
 * @brief  Low-priority deferred task.
 *
 * Woken by the ISR after actuator fires. Responsible for:
 *   1. Disabling the GPIO interrupt to prevent re-entry during hold.
 *   2. Clearing the IMU interrupt latch via SPI (safe here, not in ISR).
 *   3. Holding actuator for AGX_ACTUATOR_HOLD_MS.
 *   4. Re-enabling the GPIO interrupt to re-arm the system.
 */
static void status_task(void *arg)
{
    uint32_t    notification;
    uint8_t     imu_status = 0;
    uint32_t    event_count = 0;

    ESP_LOGI(TAG, "Status task started — system ARMED");

    while (1) {
        /* Block indefinitely until ISR sends notification */
        notification = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (notification == 0) continue;   /* Spurious wakeup */

        event_count++;
        ESP_LOGI(TAG, "▶ FREE-FALL EVENT #%lu detected", (unsigned long)event_count);

        /* 1. Disable GPIO interrupt — prevent re-trigger during hold period */
        gpio_intr_disable(AGX_PIN_IMU_INT1);

        /* 2. Clear IMU interrupt latch (SPI transfer — safe outside ISR) */
        icm42688_clear_interrupt(&s_imu, &imu_status);
        ESP_LOGD(TAG, "IMU INT_STATUS: 0x%02X (FF bit: %s)",
                 imu_status,
                 (imu_status & ICM_INT_STATUS_FF) ? "SET" : "clear");

        /* 3. Hold actuator and reset sequence (blocks for AGX_ACTUATOR_HOLD_MS) */
        actuator_hold_and_reset();

        /* 4. Re-enable GPIO interrupt → system is re-armed */
        gpio_intr_enable(AGX_PIN_IMU_INT1);
        ESP_LOGI(TAG, "✓ System RE-ARMED — waiting for next event");
    }
}

/* ─── GPIO Interrupt Configuration ───────────────────────────────────── */

/**
 * @brief  Configure AGX_PIN_IMU_INT1 as rising-edge interrupt input.
 *
 * Design notes:
 *   - IRAM-safe ISR flag ensures handler is loaded before cache can evict it.
 *   - Rising edge matches ICM-42688-P INT1 active-high pulsed configuration.
 *   - Internal pull-down keeps pin defined when IMU is not driving it.
 */
static esp_err_t configure_imu_interrupt_pin(void)
{
    gpio_config_t int_pin_cfg = {
        .pin_bit_mask = (1ULL << AGX_PIN_IMU_INT1),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_POSEDGE,     /* Rising edge = INT1 asserts */
    };
    esp_err_t ret = gpio_config(&int_pin_cfg);
    if (ret != ESP_OK) return ret;

    /* Install GPIO ISR service with IRAM flag — entire dispatch path in IRAM */
    ret = gpio_install_isr_service(ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL3);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) return ret;

    /* Register our IRAM-resident handler */
    ret = gpio_isr_handler_add(AGX_PIN_IMU_INT1, freefall_isr_handler, NULL);
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "GPIO %d interrupt configured (rising edge, IRAM ISR)",
             AGX_PIN_IMU_INT1);
    return ESP_OK;
}

/* ─── Boot Sequence ───────────────────────────────────────────────────── */

void app_main(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "╔══════════════════════════════════════╗");
    ESP_LOGI(TAG, "║       AntigravityX  v1.0.0           ║");
    ESP_LOGI(TAG, "║  Free-fall → actuator in < 10 ms     ║");
    ESP_LOGI(TAG, "╚══════════════════════════════════════╝");

    /* Step 1: Start STATUS task before enabling any interrupts.
     * The task handle must exist before the ISR can notify it. */
    xTaskCreate(status_task,
                "agx_status",
                AGX_STATUS_TASK_STACK,
                NULL,
                AGX_STATUS_TASK_PRIO,
                &s_status_task_handle);
    if (s_status_task_handle == NULL) {
        ESP_LOGE(TAG, "Failed to create status task — halting");
        return;
    }

    /* Step 2: Initialise and pre-charge actuator rail.
     * Must complete BEFORE enabling IMU interrupts. An interrupt arriving
     * before the actuator is ready would fire into an un-initialised pin. */
    ret = actuator_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Actuator init failed (%s) — halting", esp_err_to_name(ret));
        return;
    }

    /* Step 3: Initialise ICM-42688-P via SPI */
    ret = icm42688_init(&s_imu,
                        AGX_SPI_HOST,
                        AGX_PIN_CS_IMU,
                        AGX_PIN_SCK,
                        AGX_PIN_MOSI,
                        AGX_PIN_MISO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "IMU init failed (%s) — halting", esp_err_to_name(ret));
        return;
    }

    /* Step 4: Configure free-fall engine in ICM-42688-P Bank 4.
     * threshold=0x08 → 0.2 g sum-of-abs limit.
     * duration=0x02  → 2 ODR cycles at 1600 Hz = 1.25 ms hardware debounce. */
    ret = icm42688_configure_freefall(&s_imu,
                                      AGX_FF_THRESHOLD_VAL,
                                      AGX_FF_DURATION_VAL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FF configure failed (%s) — halting", esp_err_to_name(ret));
        return;
    }

    /* Step 5: Configure ESP32 GPIO interrupt for IMU INT1 */
    ret = configure_imu_interrupt_pin();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO interrupt config failed (%s) — halting",
                 esp_err_to_name(ret));
        return;
    }

    /* Step 6: Enable IMU interrupt output — system is now ARMED */
    ret = icm42688_enable_interrupt(&s_imu);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "IMU interrupt enable failed (%s) — halting",
                 esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    ESP_LOGI(TAG, "  SYSTEM ARMED — Latency budget:");
    ESP_LOGI(TAG, "  HW debounce : %u µs  (2 × 625 µs @ 1600 Hz)",
             (unsigned)(AGX_FF_DURATION_VAL * 625));
    ESP_LOGI(TAG, "  ISR + GPIO  : < 2 µs  (IRAM, direct reg write)");
    ESP_LOGI(TAG, "  Target total: < 10 ms");
    ESP_LOGI(TAG, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

    /* app_main returns — FreeRTOS scheduler runs status_task indefinitely */
}

