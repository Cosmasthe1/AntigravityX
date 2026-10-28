# AntigravityX

> **Free-fall detection → actuator trigger in under 10 ms.**
> Hardware makes the decision. Firmware only executes.

---

## What This Is

AntigravityX is an ESP32 + ICM-42688-P firmware system that detects device free-fall and fires a physical actuator (solenoid, micro cold-gas thruster, or airbag initiator) fast enough to protect the device before impact.

A 1 m drop takes ~450 ms. Detection + actuation must complete well inside that window. This system targets **< 10 ms** from physical free-fall onset to actuator energise, with the dominant latency being the configurable hardware debounce in the IMU itself.

---

## Hardware Bill of Materials

| # | Part | Value / Part No. | Notes |
|---|------|-----------------|-------|
| 1 | MCU | ESP32-WROOM-32E | Dual-core, 240 MHz, 520 KB SRAM |
| 2 | IMU | TDK ICM-42688-P | SPI, hardware free-fall engine |
| 3 | MOSFET | Nexperia PMBT2369 or AO3400A | N-ch, low Qg, fast turn-on |
| 4 | Gate resistor | 10 Ω | Fast turn-on; NOT for EMI suppression |
| 5 | Flyback diode | SS14 Schottky | Across solenoid/actuator coil |
| 6 | Boost converter | TI TPS61023 or similar | Pre-charges capacitor bank to actuator Vcc |
| 7 | Storage cap | 100–470 µF, 10 V | Actuator rail reservoir |
| 8 | Pull-down (IMU INT1) | 100 kΩ | Holds INT1 defined when IMU de-asserts |
| 9 | SPI decoupling | 100 nF each | At each IC power pin |
| 10 | Actuator | Solenoid / CO₂ valve / airbag | Application-specific |

---

## Pin Assignment (ESP32)

| GPIO | Direction | Function |
|------|-----------|----------|
| 18 | OUT | SPI SCK → IMU |
| 23 | OUT | SPI MOSI → IMU |
| 19 | IN | SPI MISO ← IMU |
| 5 | OUT | SPI CS (active low) → IMU |
| **4** | **IN** | **IMU INT1 → ESP32 (direct, dedicated trace)** |
| **2** | **OUT** | **Actuator MOSFET gate (ISR fires this)** |
| 15 | IN | Actuator rail READY (from boost converter PGOOD) |
| 13 | OUT | LED: ARMED (solid = waiting) |
| 12 | OUT | LED: TRIGGERED (blinks after event) |

---

## Latency Budget

```
Physical free-fall onset
        │
        ▼  ← IMU at 1600 Hz ODR, 684 Hz BW
[ICM-42688-P senses |ax|+|ay|+|az| < threshold]
        │
        ▼  HW debounce: 2 × 625 µs = 1.25 ms   ← Register 0x65 = 0x02
[INT1 pin asserts (rising edge)]
        │
        ▼  trace propagation: nanoseconds
[ESP32 GPIO edge interrupt fires]
        │
        ▼  ISR entry (IRAM-resident, priority 3): < 1 µs
[freefall_isr_handler executes]
        │
        ▼  REG_WRITE(GPIO_OUT_W1TS_REG, ...): ~20 ns
[MOSFET gate rises]
        │
        ▼  gate charge + turn-on: < 1 µs
[ACTUATOR ENERGISED]

Total MCU + IMU latency: ~1.3 ms
Mechanical/chemical actuator response: application-specific
```

**The design document principle: "The IMU must make the decision, the ESP32 must only execute the trigger."**

---

## Tuning the Free-Fall Parameters

Both values live in `antigravityx_config.h`:

### `AGX_FF_THRESHOLD_VAL` (Register 0x5E)
Each LSB = 256 mg. The ICM-42688-P compares `|ax| + |ay| + |az|` against this.

| Value | Threshold | Notes |
|-------|-----------|-------|
| `0x04` | 1024 mg | Very sensitive; may false-trigger on fast wrist flick |
| `0x08` | 2048 mg | **Default** — good starting point |
| `0x10` | 4096 mg | Conservative; only catches clean vertical drops |

### `AGX_FF_DURATION_VAL` (Register 0x65)
Number of ODR cycles the condition must persist. At 1600 Hz, 1 cycle = 625 µs.

| Value | Duration | Notes |
|-------|----------|-------|
| `0x01` | 625 µs | Absolute minimum; may catch vibration noise |
| `0x02` | 1.25 ms | **Default** — rejects most hand motion |
| `0x08` | 5 ms | Conservative; safe margin but higher latency |
| `0x10` | 10 ms | Defeats the purpose — do not exceed this |

**Tuning procedure:** Start with defaults. Instrument the STATUS task to log event counts. Perform controlled drop tests alongside deliberate hand motions. Increase `DURATION_VAL` by 1 until false trigger rate reaches zero, then stop.

---

## Build & Flash

```bash
# Prerequisites: ESP-IDF v5.x installed and sourced
. $IDF_PATH/export.sh

# Configure
idf.py set-target esp32

# Build
idf.py build

# Flash and monitor
idf.py -p /dev/ttyUSB0 flash monitor
```

Expected boot log:
```
I AntigravityX: ╔══════════════════════════════════════╗
I AntigravityX: ║       AntigravityX  v1.0.0           ║
I AntigravityX: ║  Free-fall → actuator in < 10 ms     ║
I AntigravityX: ╚══════════════════════════════════════╝
I ACTUATOR:     Waiting for actuator rail pre-charge...
I ACTUATOR:     Actuator rail pre-charged and READY
I ICM42688:     ICM-42688-P found (WHO_AM_I = 0x47)
I ICM42688:     Accel configured: ODR=1600 Hz, FS=±2 g, BW=684 Hz
I ICM42688:     Free-fall engine: threshold=0x08 (2048 mg), duration=0x02 (1250 µs)
I ICM42688:     INT1 enabled — free-fall → ESP32 GPIO 4
I AntigravityX: ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
I AntigravityX:   SYSTEM ARMED — Latency budget:
I AntigravityX:   HW debounce : 1250 µs  (2 × 625 µs @ 1600 Hz)
I AntigravityX:   ISR + GPIO  : < 2 µs  (IRAM, direct reg write)
I AntigravityX:   Target total: < 10 ms
I AntigravityX: ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
I status_task:  System ARMED — waiting for next event
```

---

## File Structure

```
AntigravityX/
├── CMakeLists.txt            — ESP-IDF project root
├── sdkconfig.defaults        — CPU 240 MHz, FreeRTOS 1 kHz tick, IRAM ISR
├── README.md
└── main/
    ├── CMakeLists.txt        — Component sources
    ├── antigravityx_config.h — All pins, registers, thresholds
    ├── imu_icm42688.h/.c     — ICM-42688-P SPI driver + APEX FF config
    ├── actuator.h/.c         — Pre-charge, hold, re-arm logic
    └── main.c                — Boot sequence + IRAM ISR (critical path)
```

---

## Extending to BMI270

The Bosch BMI270 is a drop-in alternative. It also provides a hardware low-g interrupt on INT1/INT2 pins. To adapt:

1. Create `imu_bmi270.h/.c` with the same public API as `imu_icm42688.h`.
2. BMI270 uses I2C or SPI. For minimum latency, use SPI.
3. Free-fall parameters live in BMI270's feature configuration DMA block — write via the `bmi2_set_sensor_config()` function in Bosch's BSX library.
4. `main.c` and `actuator.c` are unchanged — they only depend on the GPIO interrupt and actuator pin, not on which IMU is fitted.

---

## Safety Notes

- This system drives a physical actuator. Always bench-test with actuator **disconnected** before field deployment.
- The pre-charge rail is continuously energised during operation. Include a hardware kill switch on the boost converter enable pin for safe handling.
- Airbag initiators / pyrotechnic squibs require additional safety interlocks beyond the scope of this firmware.

---

*AntigravityX — "the IMU decides, the ESP32 only executes."*
