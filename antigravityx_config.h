/**
 * @file antigravityx_config.h
 * @brief AntigravityX — Central configuration for all hardware constants,
 *        pin assignments, and IMU register addresses.
 *
 * Board: ESP32 (Espressif ESP32-WROOM-32 or equivalent)
 * IMU:   TDK InvenSense ICM-42688-P  (primary)
 *
 * Design principle from document:
 *   "The IMU must make the decision, and the ESP32 must only execute
 *    the trigger." — all threshold/debounce decisions happen in HW.
 */

#pragma once

#include <stdint.h>

/* ─── SPI Bus ─────────────────────────────────────────────────────────── */
#define AGX_SPI_HOST            SPI2_HOST
#define AGX_PIN_SCK             18
#define AGX_PIN_MOSI            23
#define AGX_PIN_MISO            19
#define AGX_PIN_CS_IMU          5
#define AGX_SPI_CLK_HZ          (10 * 1000 * 1000)   /* 10 MHz — well within ICM-42688-P 24 MHz max */

/* ─── IMU Interrupt → ESP32 ───────────────────────────────────────────── */
/* Route INT1 directly to this pin. Short dedicated trace, no shared function. */
#define AGX_PIN_IMU_INT1        4

/* ─── Actuator Output ─────────────────────────────────────────────────── */
/* Drives MOSFET gate for solenoid / micro-airbag / fan trigger.            */
/* Pre-charged rail must be continuously energised — ISR only closes switch. */
#define AGX_PIN_ACTUATOR        2
#define AGX_PIN_ACTUATOR_READY  15   /* Logic-high = pre-charge rail OK     */

/* ─── Status LED ──────────────────────────────────────────────────────── */
#define AGX_PIN_LED_ARMED       13   /* Solid = armed, waiting              */
#define AGX_PIN_LED_TRIGGERED   12   /* Blinks = actuator has fired         */

/* ─── ICM-42688-P Register Map (Bank 0) ──────────────────────────────── */
#define ICM_REG_DEVICE_CONFIG   0x11
#define ICM_REG_DRIVE_CONFIG    0x13
#define ICM_REG_INT_CONFIG      0x14
#define ICM_REG_INT_STATUS      0x2D
#define ICM_REG_PWR_MGMT0       0x4E
#define ICM_REG_GYRO_CONFIG0    0x4F
#define ICM_REG_ACCEL_CONFIG0   0x50   /* ODR + FS range selection          */
#define ICM_REG_ACCEL_CONFIG1   0x53   /* LPF bandwidth                     */
#define ICM_REG_INT_SOURCE0     0x65   /* Interrupt-source routing to INT1  */
#define ICM_REG_BANK_SEL        0x76   /* Register bank select              */

/* ─── ICM-42688-P Register Map (Bank 4 — APEX / free-fall) ───────────── */
/* Switch to bank 4 via BANK_SEL = 0x04 before accessing these.            */
#define ICM_B4_APEX_CONFIG0     0x40   /* Enable APEX engine                */
#define ICM_B4_APEX_CONFIG8     0x48   /* Free-fall config block start      */
#define ICM_B4_FF_THRESHOLD     0x5E   /* |ax|+|ay|+|az| threshold register  */
#define ICM_B4_FF_DURATION      0x65   /* Persistence duration (debounce)   */
#define ICM_B4_INT_SOURCE6      0x4D   /* Route ff_det to INT1 in bank 4    */

/* ─── ICM-42688-P Field Values ───────────────────────────────────────── */

/* ACCEL_CONFIG0 — ODR bits [3:0] */
#define ICM_ACCEL_ODR_1600HZ    0x05   /* Max ODR — minimum window to miss FF */
#define ICM_ACCEL_ODR_800HZ     0x06
#define ICM_ACCEL_ODR_400HZ     0x07

/* ACCEL_CONFIG0 — full-scale bits [7:5] */
#define ICM_ACCEL_FS_2G         0x60
#define ICM_ACCEL_FS_4G         0x40
#define ICM_ACCEL_FS_8G         0x20
#define ICM_ACCEL_FS_16G        0x00

/* ACCEL_CONFIG1 — LPF bandwidth — widest BW to minimise group delay */
#define ICM_ACCEL_BW_684HZ      0x00   /* BW = ODR/2.3 ≈ 684 Hz @ 1600 Hz */

/* PWR_MGMT0 — both sensors in low-noise mode */
#define ICM_ACCEL_LN_MODE       0x03
#define ICM_GYRO_OFF            0x00   /* Gyro not needed; save power       */

/* INT_CONFIG — INT1 pulsed, active-high, push-pull */
#define ICM_INT1_PULSED_AH_PP   0x00

/* INT_SOURCE0 — route free-fall detect to INT1 */
#define ICM_FF_INT1_EN          (1 << 5)

/* APEX_CONFIG0 — enable free-fall engine */
#define ICM_APEX_FF_EN          (1 << 4)

/* ─── Free-Fall Tuning ────────────────────────────────────────────────── */
/*
 * Threshold (0x5E): sum-of-absolute acceleration.
 * LSB = 256 mg.  0x08 → 8 × 256 mg = 2048 mg ≈ 0.2 g.
 * Tune down until false triggers appear on your vibration profile.
 */
#define AGX_FF_THRESHOLD_VAL    0x08

/*
 * Duration (0x65): number of ODR cycles the condition must persist.
 * At 1600 Hz, 1 cycle = 625 µs.  Start at 1 (minimum) and increase
 * until environment-induced jitter is rejected.
 * Never exceed ~8 ms worth of cycles (12–13 cycles at 1600 Hz).
 */
#define AGX_FF_DURATION_VAL     0x02   /* 2 cycles × 625 µs = 1.25 ms debounce */

/* ─── Timing ──────────────────────────────────────────────────────────── */
#define AGX_ACTUATOR_HOLD_MS    200    /* How long to keep actuator triggered */

/* ─── Task Config ─────────────────────────────────────────────────────── */
#define AGX_STATUS_TASK_PRIO    2
#define AGX_STATUS_TASK_STACK   2048

