/**
 * @file actuator.h
 * @brief AntigravityX — Actuator pre-charge and trigger management.
 *
 * The actuator system has three responsibilities:
 *
 *  1. PRE-CHARGE: Keep the boost converter / capacitor bank on the
 *     actuator rail continuously energised so the ISR's GPIO toggle
 *     is the ONLY remaining event on the critical path. The system
 *     must NEVER wait for a power rail to charge after free-fall is
 *     detected — every millisecond counts.
 *
 *  2. TRIGGER (ISR context): Exposed only as a GPIO pin address used
 *     directly by the IRAM-resident ISR via register write. Not a
 *     function call — see main.c ISR implementation.
 *
 *  3. RESET: After AGX_ACTUATOR_HOLD_MS, de-energise the actuator and
 *     re-arm the system (re-enable the GPIO interrupt).
 *
 * Supported actuator types (selectable at compile time):
 *   - MOSFET-driven solenoid (default)
 *   - Micro cold-gas thruster (same electrical interface)
 *   - Corner airbag initiator (same electrical interface)
 */

#pragma once

#include "esp_err.h"
#include <stdbool.h>

/* ─── Actuator State ──────────────────────────────────────────────────── */

typedef enum {
    AGX_ACT_STATE_UNINIT     = 0,
    AGX_ACT_STATE_PRECHARGING,   /* Rail energising, not ready */
    AGX_ACT_STATE_ARMED,         /* Ready — GPIO interrupt active */
    AGX_ACT_STATE_TRIGGERED,     /* Actuator fired, hold timer running */
    AGX_ACT_STATE_RESETTING,     /* Actuator released, re-arming */
} agx_actuator_state_t;

/* ─── Public API ──────────────────────────────────────────────────────── */

/**
 * @brief  Configure actuator GPIO pins and start pre-charge rail.
 *         Must complete before icm42688_enable_interrupt() is called.
 *         Blocks until the pre-charge READY pin asserts (or timeout).
 *
 * @return ESP_OK when rail is pre-charged and system is armed.
 *         ESP_ERR_TIMEOUT if ready signal does not appear within 500 ms.
 */
esp_err_t actuator_init(void);

/**
 * @brief  Called from the STATUS task after actuator has fired.
 *         Holds trigger for AGX_ACTUATOR_HOLD_MS, then de-energises
 *         and re-arms the GPIO interrupt.
 */
void actuator_hold_and_reset(void);

/**
 * @brief  Return current actuator state (for status task / logging).
 */
agx_actuator_state_t actuator_get_state(void);

/**
 * @brief  Emergency disarm — de-energise actuator immediately.
 *         Use for safe power-down sequence only.
 */
void actuator_disarm(void);

