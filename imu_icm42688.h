/**
 * @file imu_icm42688.h
 * @brief AntigravityX — TDK InvenSense ICM-42688-P driver interface.
 *
 * Responsibilities:
 *   1. SPI initialisation and register read/write helpers.
 *   2. Free-fall interrupt engine configuration (Bank 4 APEX registers).
 *   3. Interrupt source routing (INT1 → ESP32 GPIO).
 *   4. Interrupt status clear (called from deferred task, NOT from ISR).
 *
 * The driver does NOT contain any ISR-resident code. The ISR lives in
 * main.c and only performs a direct GPIO register write — it never
 * calls back into this driver.
 */

#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/spi_master.h"

/* ─── Device Handle ───────────────────────────────────────────────────── */

typedef struct {
    spi_device_handle_t spi_handle;
    int                 cs_pin;
} icm42688_dev_t;

/* ─── Interrupt Status ────────────────────────────────────────────────── */

/** Bitmask returned by icm42688_clear_interrupt() */
#define ICM_INT_STATUS_FF       (1 << 5)   /* Free-fall detect bit in INT_STATUS */

/* ─── Public API ──────────────────────────────────────────────────────── */

/**
 * @brief  Initialise SPI bus, attach IMU, verify WHO_AM_I, reset device,
 *         and place accelerometer in low-noise mode at 1600 Hz / 2 g FS.
 *
 * @param[out] dev      Driver handle to populate.
 * @param[in]  host     SPI host (e.g. SPI2_HOST).
 * @param[in]  cs_pin   Chip-select GPIO.
 * @param[in]  sck_pin  Clock GPIO.
 * @param[in]  mosi_pin MOSI GPIO.
 * @param[in]  miso_pin MISO GPIO.
 * @return  ESP_OK on success, ESP_ERR_NOT_FOUND if WHO_AM_I mismatch.
 */
esp_err_t icm42688_init(icm42688_dev_t *dev,
                        spi_host_device_t host,
                        int cs_pin, int sck_pin,
                        int mosi_pin, int miso_pin);

/**
 * @brief  Configure the APEX free-fall engine (Bank 4).
 *         Sets threshold register 0x5E and duration register 0x65,
 *         enables the free-fall detect feature, and routes it to INT1.
 *
 * @param[in] dev        Initialised device handle.
 * @param[in] threshold  Value for register 0x5E (LSB = 256 mg).
 * @param[in] duration   Value for register 0x65 (ODR cycle count).
 */
esp_err_t icm42688_configure_freefall(icm42688_dev_t *dev,
                                      uint8_t threshold,
                                      uint8_t duration);

/**
 * @brief  Enable INT1 output (pulsed, active-high, push-pull).
 *         Call after icm42688_configure_freefall().
 */
esp_err_t icm42688_enable_interrupt(icm42688_dev_t *dev);

/**
 * @brief  Read INT_STATUS and clear the interrupt latch.
 *         Call from the deferred STATUS task — never from the ISR itself.
 *
 * @param[in]  dev        Initialised device handle.
 * @param[out] status_out Raw INT_STATUS register value (may be NULL).
 */
esp_err_t icm42688_clear_interrupt(icm42688_dev_t *dev, uint8_t *status_out);

/* ─── Low-level Helpers (exposed for debugging) ───────────────────────── */

uint8_t   icm42688_read_reg(icm42688_dev_t *dev, uint8_t reg);
esp_err_t icm42688_write_reg(icm42688_dev_t *dev, uint8_t reg, uint8_t val);
esp_err_t icm42688_set_bank(icm42688_dev_t *dev, uint8_t bank);

