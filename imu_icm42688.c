/**
 * @file imu_icm42688.c
 * @brief AntigravityX — ICM-42688-P driver implementation.
 *
 * Key design decisions:
 *  - SPI is used over I2C for lower per-byte overhead during config.
 *  - All free-fall parameters are written to Bank 4 (APEX feature engine).
 *  - The driver does NOT live in IRAM; it is only called during init and
 *    from the low-priority STATUS task. The ISR never touches this code.
 */

#include "imu_icm42688.h"
#include "antigravityx_config.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

static const char *TAG = "ICM42688";

/* WHO_AM_I expected value for ICM-42688-P */
#define ICM_WHO_AM_I_REG    0x75
#define ICM_WHO_AM_I_VAL    0x47

/* SPI framing: read = bit 7 set, write = bit 7 clear */
#define SPI_READ_FLAG       0x80

/* ─── Low-level SPI helpers ───────────────────────────────────────────── */

/**
 * @brief Select the register bank. ICM-42688-P has 5 banks (0-4).
 */
esp_err_t icm42688_set_bank(icm42688_dev_t *dev, uint8_t bank)
{
    return icm42688_write_reg(dev, ICM_REG_BANK_SEL, bank);
}

/**
 * @brief Write one byte to a register in the current bank.
 */
esp_err_t icm42688_write_reg(icm42688_dev_t *dev, uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7F), val };   /* MSB clear = write */
    spi_transaction_t t = {
        .length    = 16,
        .tx_buffer = tx,
        .rx_buffer = NULL,
        .flags     = 0,
    };
    return spi_device_transmit(dev->spi_handle, &t);
}

/**
 * @brief Read one byte from a register in the current bank.
 */
uint8_t icm42688_read_reg(icm42688_dev_t *dev, uint8_t reg)
{
    uint8_t tx[2] = { (uint8_t)(reg | SPI_READ_FLAG), 0x00 };
    uint8_t rx[2] = { 0, 0 };
    spi_transaction_t t = {
        .length    = 16,
        .tx_buffer = tx,
        .rx_buffer = rx,
        .flags     = 0,
    };
    spi_device_transmit(dev->spi_handle, &t);
    return rx[1];   /* Second byte is the register's data */
}

/* ─── Public API ──────────────────────────────────────────────────────── */

esp_err_t icm42688_init(icm42688_dev_t *dev,
                        spi_host_device_t host,
                        int cs_pin, int sck_pin,
                        int mosi_pin, int miso_pin)
{
    esp_err_t ret;

    /* 1. Configure SPI bus */
    spi_bus_config_t bus_cfg = {
        .mosi_io_num     = mosi_pin,
        .miso_io_num     = miso_pin,
        .sclk_io_num     = sck_pin,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 64,
    };
    ret = spi_bus_initialize(host, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 2. Attach IMU as SPI device */
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = AGX_SPI_CLK_HZ,
        .mode           = 0,              /* CPOL=0, CPHA=0 */
        .spics_io_num   = cs_pin,
        .queue_size     = 4,
        .pre_cb         = NULL,
        .post_cb        = NULL,
    };
    ret = spi_bus_add_device(host, &dev_cfg, &dev->spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI device attach failed: %s", esp_err_to_name(ret));
        return ret;
    }
    dev->cs_pin = cs_pin;

    /* 3. Soft reset — clears all registers to POR state */
    icm42688_set_bank(dev, 0);
    icm42688_write_reg(dev, ICM_REG_DEVICE_CONFIG, 0x01);
    vTaskDelay(pdMS_TO_TICKS(2));    /* POR takes ≤ 1 ms per datasheet */

    /* 4. Verify WHO_AM_I */
    uint8_t who = icm42688_read_reg(dev, ICM_WHO_AM_I_REG);
    if (who != ICM_WHO_AM_I_VAL) {
        ESP_LOGE(TAG, "WHO_AM_I mismatch: got 0x%02X, expected 0x%02X",
                 who, ICM_WHO_AM_I_VAL);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "ICM-42688-P found (WHO_AM_I = 0x%02X)", who);

    /* 5. Power management: accel low-noise, gyro off (not needed for free-fall) */
    icm42688_write_reg(dev, ICM_REG_PWR_MGMT0,
                       (ICM_GYRO_OFF << 2) | ICM_ACCEL_LN_MODE);
    vTaskDelay(pdMS_TO_TICKS(1));    /* Mode change settling */

    /* 6. Accelerometer: 1600 Hz ODR, ±2 g FS
     *    Highest ODR = shortest window in which a free-fall event can be missed.
     *    Narrow FS = most sensitivity per LSB at the near-zero g values of free-fall. */
    icm42688_write_reg(dev, ICM_REG_ACCEL_CONFIG0,
                       ICM_ACCEL_FS_2G | ICM_ACCEL_ODR_1600HZ);

    /* 7. Accelerometer LPF: widest bandwidth (684 Hz) to minimise group delay.
     *    Higher BW = faster detection of the transition into free-fall. */
    icm42688_write_reg(dev, ICM_REG_ACCEL_CONFIG1, ICM_ACCEL_BW_684HZ);

    ESP_LOGI(TAG, "Accel configured: ODR=1600 Hz, FS=±2 g, BW=684 Hz");
    return ESP_OK;
}

esp_err_t icm42688_configure_freefall(icm42688_dev_t *dev,
                                      uint8_t threshold,
                                      uint8_t duration)
{
    esp_err_t ret;

    /* Switch to Bank 4 to access APEX feature engine */
    ret = icm42688_set_bank(dev, 4);
    if (ret != ESP_OK) return ret;

    /* Write threshold to register 0x5E
     * LSB = 256 mg.  threshold=0x08 → 2048 mg ≈ 0.2 g.
     * Values below ~1 g indicate free-fall; we use 0.2 g for margin. */
    ret = icm42688_write_reg(dev, ICM_B4_FF_THRESHOLD, threshold);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FF threshold write failed");
        icm42688_set_bank(dev, 0);
        return ret;
    }

    /* Write duration to register 0x65
     * Number of ODR cycles the sum-of-abs must stay below threshold.
     * At 1600 Hz: 1 cycle = 625 µs.  duration=2 → 1.25 ms hardware debounce.
     * This is the "minimum viable" setting per the design document —
     * increase only if false triggers occur in your mechanical environment. */
    ret = icm42688_write_reg(dev, ICM_B4_FF_DURATION, duration);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FF duration write failed");
        icm42688_set_bank(dev, 0);
        return ret;
    }

    /* Enable free-fall detect in APEX_CONFIG0 */
    uint8_t apex_cfg = icm42688_read_reg(dev, ICM_B4_APEX_CONFIG0);
    icm42688_write_reg(dev, ICM_B4_APEX_CONFIG0, apex_cfg | ICM_APEX_FF_EN);

    /* Route free-fall detect to INT1 (Bank 4 interrupt source register) */
    icm42688_write_reg(dev, ICM_B4_INT_SOURCE6, ICM_FF_INT1_EN);

    ESP_LOGI(TAG,
             "Free-fall engine: threshold=0x%02X (%u mg), duration=0x%02X (%u µs)",
             threshold,
             (unsigned)(threshold * 256),
             duration,
             (unsigned)(duration * 625));

    /* Return to Bank 0 */
    return icm42688_set_bank(dev, 0);
}

esp_err_t icm42688_enable_interrupt(icm42688_dev_t *dev)
{
    /* Bank 0: INT_CONFIG — pulsed (not latched), active-high, push-pull */
    icm42688_write_reg(dev, ICM_REG_INT_CONFIG, ICM_INT1_PULSED_AH_PP);

    /* INT_SOURCE0 — enable free-fall interrupt routed to INT1 */
    uint8_t src = icm42688_read_reg(dev, ICM_REG_INT_SOURCE0);
    icm42688_write_reg(dev, ICM_REG_INT_SOURCE0, src | ICM_FF_INT1_EN);

    ESP_LOGI(TAG, "INT1 enabled — free-fall → ESP32 GPIO %d", AGX_PIN_IMU_INT1);
    return ESP_OK;
}

esp_err_t icm42688_clear_interrupt(icm42688_dev_t *dev, uint8_t *status_out)
{
    /* Reading INT_STATUS self-clears the latch (pulsed mode, but explicit
     * read ensures any latched flag is consumed before next event). */
    uint8_t status = icm42688_read_reg(dev, ICM_REG_INT_STATUS);
    if (status_out) {
        *status_out = status;
    }
    return ESP_OK;
}

