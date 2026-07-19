# spinkovacka — ESP32 riadenie motora (DRV8251A)

ESP-IDF firmware pre **ESP32** (classic). Celá logika je v jednom súbore
`main/spinkovacka.c`. Zariadenie riadi DC motor cez H-mostík **DRV8251A** jedným
smerom, meria napätie batérie a prúd motora a obsluhuje zapínacie tlačidlo.

## Build

```
idf.py set-target esp32
idf.py build
idf.py -p <PORT> flash monitor
```

Diagnostiky v IDE typu „include not found" pred prvým buildom sú len chýbajúce
ESP-IDF cesty pre IntelliSense — po `idf.py build` (vygeneruje `build/compile_commands.json`) zmiznú.

`main/CMakeLists.txt` → `REQUIRES esp_adc esp_timer esp_driver_gpio esp_driver_ledc`.

## Piny

| Signál | GPIO | Smer | Poznámka |
|---|---|---|---|
| LED_BATT | 21 | OUT | 0 = svieti, 1 = zhasnutá; boot = 1; indikuje stav batérie |
| LED_MODE | 19 | OUT | 0 = svieti, 1 = zhasnutá; boot = 1; indikuje „mode" |
| MOT_IN1 | 22 | PWM (LEDC) | 10 kHz, invertovaný výstup; duty 0 % = brzda, plný duty = pohon |
| MOT_IN2 | 23 | OUT | trvalo 1 |
| ON_OFF | 26 | OUT | 1 = drží napájanie MCU zopnuté; 0 = vypnutie |
| ONOFF_BUTTON | 27 | IN | 0 = stlačené; interný pull-up; debounce 50 ms |
| IPROPI | 32 | ADC1_CH4 | prúd motora z DRV8251A; ADC atten 12 dB |
| BAT_VOLT | 33 | ADC1_CH5 | napätie batérie; ADC atten 0 dB (ref 1.1 V) |
| LIMIT_SW_1 | 35 | IN | 1 = aktívny; **spúšťa** motor |
| LIMIT_SW_2 | 34 | IN | 1 = aktívny; **zastavuje** motor |

Pozn.: GPIO32–39 sú input-only bez interných pull rezistorov — LIMIT_SW_1/2 majú
pull riešený externe (optická závora + Schmitt, čistý signál). Pozor: oproti staršej
verzii sú LIMIT_SW_1/2 na iných pinoch a sú **aktívne v HIGH** (vstup 0 = závora
neaktívna).

## Správanie

- **Boot:** LED_BATT/LED_MODE zhasnuté, motor brzda (MOT_IN1=1, MOT_IN2=1), ON_OFF=1.
- **Motor:** LEDC 10 kHz, 10-bit, aktívny v LOW (duty 100 % ⇒ MOT_IN1=0 plný pohon,
  duty 0 % ⇒ MOT_IN1=1 brzda). Pracovný bod `MOTOR_PWM_DUTY_PCT = 50` (ak nie je
  zapnutá interpolácia, viď nižšie).
- **Cyklus:** v STOPPED sa čaká na aktivačnú hranu LIMIT_SW_1 (prvý štart po boote
  hneď, ak nie je aktívny LIMIT_SW_2 a batéria je OK/LOW). V RUNNING po hrane
  LIMIT_SW_2 počká `LIMIT_SW2_BRAKE_DELAY_MS` (15 ms) a zabrzdí. Pri štarte
  `g_cycle_count++`.
- **Auto-vypnutie:** ak motor v STOPPED nečinne stojí `MOTOR_IDLE_TIMEOUT_US`
  (10 min), zariadenie sa samo vypne (`power_off()`).
- **Merania (1 kHz cez esp_timer):** BAT_VOLT (kĺzavý priemer 256) len keď motor
  stojí; IPROPI (priemer 64) len keď beží — IPROPI zatiaľ len meranie, nevyužité.

### Napätie batérie a LED_BATT

Napätie sa počíta z kalibrovaného ADC (line-fitting, atten 0 dB / ref 1.1 V)
prenásobeného `BAT_VOLT_DIVIDER_RATIO = 22.8181` (pomer odporového deliča —
**treba overiť/kalibrovať** na reálny delič). Obsluhuje samostatný
`battery_check_task` (perióda 250 ms) so stavmi:

- `BATTERY_OK` (`≥ 16 V`) — LED_BATT zhasnutá.
- `BATTERY_LOW` (`12 V < V < 16 V`) — LED_BATT **bliká**.
- `BATTERY_CRITICAL` (`≤ 12 V`) — LED_BATT trvalo svieti, **motor sa nespustí**.

`BATTERY_HYSTERESIS = 0.5 V` je definovaná pre prechody (zatiaľ len konštanta).

### Interpolácia PWM duty podľa napätia (voliteľné)

Makro `DO_DUTY_INTERPOLATION` (v `spinkovacka.c`) zapína lineárnu interpoláciu
pracovného duty podľa `g_bat_volt`: `VBAT_LOW (16 V) → DUTY_VBAT_LOW (80 %)`,
`VBAT_HIGH (22 V) → DUTY_VBAT_HIGH (50 %)`, mimo rozsahu sa oreže na krajné
hodnoty. Duty sa zvolí **raz pri štarte cyklu** z posledného merania naprázdno
(Vbat sa počas behu nevzorkuje). Ak makro nie je definované, použije sa fixný
`MOTOR_PWM_DUTY_PCT`.

### Tlačidlo ONOFF_BUTTON

Uvoľnenie po 3–6 s ⇒ vypnutie (ON_OFF=0, dead loop); uvoľnenie po >6 s ⇒ „mode"
(LED_MODE svieti, `g_wifi_mode_active = true`), v mode tá istá logika (3–6 s
vypnutie, >6 s návrat a zhasnutie LED_MODE). Debounce 50 ms pri vzorkovaní 10 ms.

## Architektúra

`app_main` → init GPIO / LEDC PWM / ADC + `esp_timer` 1 kHz vzorkovač + 3 FreeRTOS tasky:
- **motor_control_task** (prio 6) — stavový automat STOPPED/RUNNING, hrany
  LIMIT_SW_1/2, `cycle_count`, auto-vypnutie po nečinnosti, voľba duty.
- **onoff_button_task** (prio 5) — SWITCH, debounce, logika 3–6 s / >6 s,
  `power_off()`, mode + LED_MODE.
- **battery_check_task** (prio 4) — meranie napätia, stav batérie a LED_BATT.

`esp_timer` sa používa pre 1 kHz vzorkovanie preto, že default FreeRTOS tick je 100 Hz
(sub-10 ms `vTaskDelay` by nefungoval). Callback beží v esp_timer **tasku** (nie ISR),
takže blokujúci `adc_oneshot_read` je tam v poriadku.

## Otvorené TODO

- **„mode" funkcia** (SWITCH >6 s): má neskôr zapnúť **BT/WiFi** a mať vlastnú
  funkcionalitu — zatiaľ len flag `g_wifi_mode_active` + LED_MODE.
- Zvážiť, či má byť motor počas „mode" zastavený (teraz beží ďalej).
- Kalibrovať `BAT_VOLT_DIVIDER_RATIO` na reálny odporový delič.
- IPROPI (prúd motora) sa meria, ale zatiaľ sa nikde nevyužíva.
