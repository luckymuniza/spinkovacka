# spinkovacka — ESP32 motor control (DRV8251A)

ESP-IDF firmware for the **ESP32** (classic). All logic lives in a single file,
`main/spinkovacka.c`. The device drives a DC motor one direction through a
**DRV8251A** H-bridge, measures battery voltage and motor current, and handles a
power on/off button.

## Build

```sh
idf.py set-target esp32
idf.py build
idf.py -p <PORT> flash monitor
```

`main/CMakeLists.txt` requires: `esp_adc esp_timer esp_driver_gpio esp_driver_ledc`.

> "Include not found" diagnostics in the IDE before the first build are just
> missing ESP-IDF IntelliSense paths — they disappear after `idf.py build`
> generates `build/compile_commands.json`.

## Pinout

| Signal | GPIO | Dir | Notes |
|---|---|---|---|
| LED_BATT | 21 | OUT | 0 = on, 1 = off; boot = 1; battery-state indicator |
| LED_MODE | 19 | OUT | 0 = on, 1 = off; boot = 1; "mode" indicator |
| MOT_IN1 | 22 | PWM (LEDC) | 10 kHz, active-low; duty 0 % = brake, full duty = drive |
| MOT_IN2 | 23 | OUT | held at 1 |
| ON_OFF | 26 | OUT | 1 = keeps MCU power latched on; 0 = power off |
| ONOFF_BUTTON | 27 | IN | 0 = pressed; internal pull-up; 50 ms debounce |
| IPROPI | 32 | ADC1_CH4 | motor current from DRV8251A; ADC atten 12 dB |
| BAT_VOLT | 33 | ADC1_CH5 | battery voltage; ADC atten 0 dB (ref 1.1 V) |
| LIMIT_SW_1 | 35 | IN | 1 = active; **starts** the motor |
| LIMIT_SW_2 | 34 | IN | 1 = active; **stops** the motor |

> GPIO32–39 are input-only with no internal pull resistors. LIMIT_SW_1/2 are
> pulled externally (optical barrier + Schmitt trigger, clean signal) and are
> **active-high** (input 0 = barrier inactive).

## Behavior

- **Boot:** LED_BATT/LED_MODE off, motor braking (MOT_IN1=1, MOT_IN2=1), ON_OFF=1.
- **Motor:** LEDC 10 kHz, 10-bit, active-low (100 % duty ⇒ MOT_IN1=0 full drive,
  0 % duty ⇒ MOT_IN1=1 brake). Working point `MOTOR_PWM_DUTY_PCT = 50` unless
  duty interpolation is enabled (see below).
- **Cycle:** in STOPPED the state machine waits for a LIMIT_SW_1 activation edge
  (first start after boot fires immediately if LIMIT_SW_2 is not active and the
  battery is OK/LOW). In RUNNING, after a LIMIT_SW_2 edge it waits
  `LIMIT_SW2_BRAKE_DELAY_MS` and brakes. `g_cycle_count++` on each start.
- **Auto power-off:** if the motor sits idle in STOPPED for
  `MOTOR_IDLE_TIMEOUT_US` (10 min), the device powers itself off (`power_off()`).
- **Sampling (1 kHz via esp_timer):** BAT_VOLT (256-sample moving average) only
  while the motor is stopped; IPROPI (64-sample average) only while running.
  IPROPI is measured but not yet used.

### Battery voltage & LED_BATT

Voltage is computed from the calibrated ADC (line-fit, atten 0 dB / 1.1 V ref)
scaled by `BAT_VOLT_DIVIDER_RATIO = 22.8181` (resistor-divider ratio — **must be
verified/calibrated** against the real divider). Handled by a dedicated
`battery_check_task` (250 ms period):

- `BATTERY_OK` (`≥ 16 V`) — LED_BATT off.
- `BATTERY_LOW` (`12 V < V < 16 V`) — LED_BATT **blinks**.
- `BATTERY_CRITICAL` (`≤ 12 V`) — LED_BATT solid on, **motor will not start**.

`BATTERY_HYSTERESIS = 0.5 V` is defined for transitions (currently a constant only).

### Optional PWM duty interpolation by voltage

The `DO_DUTY_INTERPOLATION` macro (in `spinkovacka.c`) enables linear
interpolation of the working duty from `g_bat_volt`:
`VBAT_LOW (16 V) → DUTY_VBAT_LOW (80 %)`, `VBAT_HIGH (22 V) → DUTY_VBAT_HIGH (50 %)`,
clamped to the endpoints outside that range. Duty is chosen **once at cycle start**
from the last no-load measurement (Vbat is not sampled while running). If the macro
is undefined, the fixed `MOTOR_PWM_DUTY_PCT` is used.

### ONOFF_BUTTON

Release after 3–6 s ⇒ power off (ON_OFF=0, dead loop). Release after >6 s ⇒ "mode"
(LED_MODE on, `g_wifi_mode_active = true`); in mode the same logic applies (3–6 s
power off, >6 s return and turn LED_MODE off). 50 ms debounce at 10 ms sampling.

## Architecture

`app_main` → init GPIO / LEDC PWM / ADC + a 1 kHz `esp_timer` sampler + 3 FreeRTOS tasks:

- **motor_control_task** (prio 6) — STOPPED/RUNNING state machine, LIMIT_SW_1/2
  edges, `cycle_count`, idle auto power-off, duty selection.
- **onoff_button_task** (prio 5) — SWITCH, debounce, 3–6 s / >6 s logic,
  `power_off()`, mode + LED_MODE.
- **battery_check_task** (prio 4) — voltage measurement, battery state, LED_BATT.

`esp_timer` drives the 1 kHz sampling because the default FreeRTOS tick is 100 Hz
(a sub-10 ms `vTaskDelay` would not work). The callback runs in the esp_timer
**task** (not an ISR), so the blocking `adc_oneshot_read` is fine there.

## Open TODOs

- **"mode" function** (SWITCH >6 s): should later enable **BT/WiFi** with its own
  functionality — currently just the `g_wifi_mode_active` flag + LED_MODE.
- Decide whether the motor should be stopped during "mode" (it keeps running now).
- Calibrate `BAT_VOLT_DIVIDER_RATIO` to the real resistor divider.
- IPROPI (motor current) is measured but not yet used anywhere.
