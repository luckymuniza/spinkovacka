/*
 * esp0 - riadenie motora s DRV8251A, meranie napatia baterie a prudu.
 *
 *  - Po boote: LED1/LED2 zhasnute (1), motor v brzde (MOT_IN1=1, MOT_IN2=1),
 *    ON_OFF=1 (drzi napajanie MCU zopnute).
 *  - Motor: PWM 1 kHz na MOT_IN1, aktivny v LOW (duty 100% => MOT_IN1=0 plny pohon,
 *    duty 0% => MOT_IN1=1 brzda). Motor PWM Duty = MOTOR_PWM_DUTY_PCT (50 %).
 *    MOT_IN2 je trvalo 1. Alebo Motor PWM Duty = korekcia na Vbat.
 *  - Cyklus: v STOPPED sa caka na aktivacnu hranu LIMIT_SW_1 (prvy start po boote
 *    sa spusti hned, pokial nie je aktivny LIMIT_SW_2). Po starte cycle_count++.
 *    V RUNNING sa po aktivacii LIMIT_SW_2 pocka LIMIT_SW2_BRAKE_DELAY_MS a zabrzdi.
 *  - Merania: BAT_VOLT (klzavy priemer 256) len ked motor stoji; IPROPI
 *    (klzavy priemer 64) len ked motor bezi. Vzorkovanie 1 kHz cez esp_timer.
 *  - LED1: svieti (0) ked BAT_VOLT <= 16 V, zhasne az pri >= 16 V.
 *    Ked BAT_VOLT <= 12 V, motor sa nespusti.
 *  - Tlacidlo SWITCH: uvolnenie po 3-6 s => vypnutie (ON_OFF=0, dead loop),
 *    uvolnenie po >6 s => "mode" funkcia (LED2 svieti; neskor BT/WiFi), v ktorej
 *    plati ta ista logika (3-6 s vypnutie, >6 s navrat a zhasnutie LED2).
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_timer.h"
#include "esp_log.h"

/* ----------------------------- Piny ------------------------------------- */
#define LED_BATT_GPIO           21       /*LED pin pre zobrazenie stavu baterie */
#define LED_MODE_GPIO           19       /*LED pin pre zobrazenie stavu - vypinanie, mod BT/WIFI */
#define MOT_IN1_GPIO            22       /* PWM vystup (LEDC) */
#define MOT_IN2_GPIO            23       /* riadenie motora 2. vstup, stale = 1*/
#define ONOFF_GPIO              26       /* vystup - zapnutie napajania*/
#define ONOFF_BUTTON_GPIO       27       /* tlacitko , zapnutie, vypnutie, zmena modu*/
#define IPROPI_GPIO             32       /* ADC1_CH4 */
#define BAT_VOLT_GPIO           33       /* ADC1_CH5 */
#define LIMIT_SW_1_GPIO         35       //sw1 - spusta motor
#define LIMIT_SW_2_GPIO         34       //sw2 - vypina motor

//ADC pouzite kanaly
#define IPROPI_ADC_CH      ADC_CHANNEL_4
#define BAT_VOLT_ADC_CH    ADC_CHANNEL_5

/* --------------------------- Konstanty ---------------------------------- */
#define MOTOR_PWM_FREQ_HZ         10000   /* frekvencia PWM preriadenie motora */
#define MOTOR_PWM_DUTY_PCT        50      /* pracovny duty pri behu motora ak nie je pouzita interpolacia */
#define LIMIT_SW2_BRAKE_DELAY_MS  300     /*[ms] cas pre dobeh motora, po aktivacii LIMIT_SW2 sa zabrzdi motor po tomto case */
//PWM driver duty = 0... 1023, (10bit)
#define LEDC_MODE                 LEDC_LOW_SPEED_MODE
#define LEDC_TIMER                LEDC_TIMER_0
#define LEDC_CHANNEL              LEDC_CHANNEL_0
#define LEDC_DUTY_RES             LEDC_TIMER_10_BIT
#define LEDC_DUTY_MAX             ((1 << 10) - 1)   /* 1023 */


#define BAT_VOLT_DIVIDER_RATIO    22.8181f   /* Vbat/Vadc pomer odporoveho delica R5,R6 (R5+R6)/R6 */
#define IPROPI_FULL_SCALE_V       3.3f     /* IPROPI je priamo napatie 0..3.3 V */

#define BATTERY_LOW_LEVEL_V        16.0f  //napatie ked zacne blikat led
#define BATTERY_CRITICAL_LEVEL_V   12.0f  //napatie ked sa uz nespusti motor
#define BATTERY_HYSTERESIS         0.5f   //hysterezie pre prechod z vybiteho do nabiteho stavu (neni implemntovane)
//korekcia PWM koli poklesu VBat
#define DO_DUTY_INTERPOLATION           /* robi interpolaciu PWM duty ak sa znizuje napatie baterie -> zvacsuje duty*/
//dva body na priamke pr einterpolaciu [VBAT_LOW, DUTY_VBAT_LOW], [VBAT_HIGH, DUTY_VBAT_HIGH]
#define VBAT_LOW                16.0f
#define VBAT_HIGH               22.0f
#define DUTY_VBAT_LOW           80      /* 80 % duty pri VBAT_LOW */
#define DUTY_VBAT_HIGH          50      /* 50 % duty pri VBAT_HIGH */


//velkost bufera pre ADC
#define BAT_AVG_BUFF_BIT      8    //(1<<8) = 256 vzoriek
#define IPROPI_AVG_BUFF_BIT   6    //(1<<6) = 64 vzoriek
#define BAT_AVG_BUFF_SIZE      (1<<BAT_AVG_BUFF_BIT)
#define IPROPI_AVG_BUFF_SIZE   (1<<IPROPI_AVG_BUFF_BIT)
#define ADC_SAMPLE_PERIOD_US      1000 /* [us]   1ms => 1 kHz */


#define ONOFF_BUTTON_DEBOUNCE_MS    50  // doba po ktoru musi byt onoff buton v nezmenenom stave aby sa akceptovala hodnota
#define BUTTON_CYCLE_TIME_MS        10      //[ms] perioda s akou sa kontroluje stlaceni tlacitka
#define MOTOR_CONTROL_CYCLE_TIME_MS 10      //[ms] perioda s akou sa aktualizuje stav motora, zisti sa stav limit_sw
#define BATTERY_CHECK_CYCLE_TIME_MS 250     //[ms] perioda s akou sa kontroluje stav baterie

#define SWITCHING_OFF_HOLD_US     3000000     /*[us] 3-6 s => vypnutie */
#define CHANGE_MODE_HOLD_US       6000000     /*[us] >6 s zmena WIFI/BT modu  */

#define MOTOR_IDLE_TIMEOUT_US      (10 * 60 * 1000000ULL)   //[us] po 10min sa vypne spinkovacka automaticky


static const char *TAG = "Spinkovac";

/*  Stav v akom sa nachadza motor  */
typedef enum {
    MOTOR_STOPPED = 0,
    MOTOR_RUNNING
} motor_state_t;

/*   stav baterie */
typedef enum {
    BATTERY_OK = 0,
    BATTERY_LOW,
    BATTERY_CRITICAL,
    BATTERY_NOT_INITIALIZED
} battery_state_t;

/* stav ledky pre zobrazenie stavu baterie
 * zhasnuta:  Vbat > BATTERY_LOW_LEVEL_V
 * blika   :  BATTERY_CRITICAL_LEVEL_V < Vbat < BATTERY_LOW_LEVEL_V
 * svieti  :  BATTERY_CRITICAL_LEVEL_V > Vbat, motor uz nespusti
 */
 typedef enum {
    LED_BATT_OFF,
    LED_BATT_ON,
    LED_BATT_BLINK
}battery_led_state_t;

static volatile motor_state_t g_motor_state = MOTOR_STOPPED;    // stav motora
static volatile uint16_t g_battery_volt_raw = 0;                //cista hodnota s ADC, spriemerovana
static volatile uint16_t g_ipropi_volt_raw = 0;                 //cista hodnota s ADc, sprimrovana
static volatile battery_state_t g_battery_state = BATTERY_NOT_INITIALIZED;  //stav baterie

static volatile float         g_bat_volt    = 0.0f;   /* [V], napatie baterie, po prepocitani pomeru delica */
static volatile float         g_ipropi_volt = 0.0f;   /* V, priemer 64,  nepouzite */
static volatile uint32_t      g_cycle_count = 0;      // TODO, pocet cyklov spinkovacky ale len pri napajani, neuklada sa
static volatile bool          g_wifi_mode_active = false;  /*TODO, ci je alebo nie je aktivny BT/WIFI */

//ADC driver objects
static adc_oneshot_unit_handle_t s_adc = NULL;
static adc_cali_handle_t         s_bat_adc_cali = NULL;
static adc_cali_handle_t         s_ipropi_adc_cali = NULL;



/* --------------------------- Klzavy priemer ----------------------------- */
typedef struct {
    uint16_t *buf;       //pointer na bufer kde su ulozene posledne hodnoty
    int       size_bit;  //velkost bufera   = 8 => 1<<8 = 256 vzoriek
    int       idx;       // index na hodnotu ktora sa vymaza a nahradi novou v bufery
    int       buff_filed; //na zaciatku kym sa naplni bufer hodnotami tak sa inkrementuje az po BUFF_SIZE
    uint32_t  sum;       // suma hodnot v bufferi
} meaavg_t;

static uint16_t s_bat_buf[BAT_AVG_BUFF_SIZE];        //bufer pre VBAT
static uint16_t s_ipropi_buf[IPROPI_AVG_BUFF_SIZE];  //bufer pr IPROPI
//struct. pre vypocet klzavy priemr VBAT
static meaavg_t s_battery_avg    = { .buf = s_bat_buf,    .size_bit = BAT_AVG_BUFF_BIT, .buff_filed = 0, .sum = 0 };
//struct. pre IPROPI
static meaavg_t s_ipropi_avg = { .buf = s_ipropi_buf, .size_bit = IPROPI_AVG_BUFF_BIT, .buff_filed = 0, .sum = 0 };


/* aktualizacia bufera pre vypocet plavajuceho priemeru pre IPROPI aj VBAT
 * kruhovy FIFO bufer, ulozena je len vysledna sum, priemer sa pocita len ked treba
 */

static void meaavg_push(meaavg_t *m, uint16_t val)
{
    if (m->buff_filed < (1<<m->size_bit)) {
        m->buff_filed++;
    } else {
        m->sum -= m->buf[m->idx];   /* najstarsia vzorka ide prec */
    }
    m->sum += val;
    m->buf[m->idx] = val;
    m->idx = (m->idx + 1) % (1<<m->size_bit);
}

//vrati celkovu sumu z bufera
static uint32_t meaavg_get_sum(const meaavg_t *m)
{
    if (m->buff_filed == (1<<m->size_bit)) {
        return m->sum;
    }
    else {
        //pokial neni plny bufer, na zaciatku, vracia 0
        return 0;
    }
}


/*
 * **********************************************************************
 * ------------------------------- Motor ----------------------------------
 * ***********************************************************************
 */

/*
 * Nastavy DUTY pre PWM motora - na jednom pine
 */
static void motor_set_duty_percent(int duty_percent)
{
    uint32_t duty = ((uint32_t)duty_percent * LEDC_DUTY_MAX) / 100;
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}


/* Brzda: duty 0 % -> (invertovany vystup) MOT_IN1 = 1
 * vola sa ked treba motor zabrzdit
 */
static void motor_brake(void) {
    motor_set_duty_percent(0);
}

#ifdef DO_DUTY_INTERPOLATION
/* Linearna interpolacia duty [%] podla napatia baterie:
 *   VBAT_LOW  -> DUTY_VBAT_LOW
 *   VBAT_HIGH -> DUTY_VBAT_HIGH
 * Mimo rozsahu sa orezava na krajne duty hodnoty.
 */
static int motor_duty_from_vbat(float vbat)
{
    float d = (float)DUTY_VBAT_LOW +
              (vbat - VBAT_LOW) *
              (float)(DUTY_VBAT_HIGH - DUTY_VBAT_LOW) /
              (VBAT_HIGH - VBAT_LOW);

    int duty = (int)(d + 0.5f);   /* zaokruhlenie */

    /* clamp medzi obe duty hodnoty (funguje aj pri klesajucej krivke) */
    int lo = (DUTY_VBAT_LOW < DUTY_VBAT_HIGH) ? DUTY_VBAT_LOW : DUTY_VBAT_HIGH;
    int hi = (DUTY_VBAT_LOW < DUTY_VBAT_HIGH) ? DUTY_VBAT_HIGH : DUTY_VBAT_LOW;
    if (duty < lo) duty = lo;
    if (duty > hi) duty = hi;
    return duty;
}
#endif


/* Spustenie motora
 * duty fixne alebo interpolacia podla Vbat
 */
static void motor_run(void) {
#ifdef DO_DUTY_INTERPOLATION
    motor_set_duty_percent(motor_duty_from_vbat(g_bat_volt));
#else
    motor_set_duty_percent(MOTOR_PWM_DUTY_PCT);
#endif
}


/* Inicializacia PWM pre riadenie motora, len jeden vstupny pin je PWM ten druhy je 1
 * ak su obidva 1 motor brzdi - skratovane je vinutie
 */

static void motor_pwm_init(void)
{
    ledc_timer_config_t tcfg = {
        .speed_mode      = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num       = LEDC_TIMER,
        .freq_hz         = MOTOR_PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&tcfg));

    ledc_channel_config_t ccfg = {
        .gpio_num   = MOT_IN1_GPIO,
        .speed_mode = LEDC_MODE,
        .channel    = LEDC_CHANNEL,
        .timer_sel  = LEDC_TIMER,
        .duty       = 0,            /* 0 -> po inverzii MOT_IN1 = 1 (brzda) */
        .hpoint     = 0,
        .flags.output_invert = 1,   /* aktivny v LOW: 100% => MOT_IN1=0 */
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ccfg));
    motor_brake();
}






/*
 * ***********************************************************************
 * -------------------------------- GPIO ----------------------------------
 * ************************************************************************
 */
static void gpio_init_all(void)
{
    /* Digitalne vystupy: LED1, LED2, MOT_IN2, ON_OFF (MOT_IN1 nastavi LEDC PWM controler) */
    gpio_config_t out = {
        .pin_bit_mask = (1ULL << LED_BATT_GPIO) | (1ULL << LED_MODE_GPIO) |
                        (1ULL << MOT_IN2_GPIO) | (1ULL << ONOFF_GPIO),
        .mode         = GPIO_MODE_INPUT_OUTPUT, //aj input aby sa dal precitat stav pinu (koli led blikaniu)
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&out));

    /* vychodzie stavy */
    gpio_set_level(LED_BATT_GPIO, 1);      /* LED zhasnuta */
    gpio_set_level(LED_MODE_GPIO, 1);      /* LED zhasnuta */
    gpio_set_level(MOT_IN2_GPIO, 1);   /* trvalo 1 */
    gpio_set_level(ONOFF_GPIO, 1);    /* drzi napajanie zopnute */

    /* SWITCH: vstup s internym pull-up (stlacene = 0) */
    gpio_config_t sw = {
        .pin_bit_mask = (1ULL << ONOFF_BUTTON_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&sw));

    /* LIMIT_SW_1/2: vstupy, cisty signal (opticka zavora + Schmitt),
     * netreba interne pull up. */
    gpio_config_t lim = {
        .pin_bit_mask = (1ULL << LIMIT_SW_1_GPIO) | (1ULL << LIMIT_SW_2_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&lim));
}

/*vrati stav LiMIT_SW 1 a 2e
 * 1 - aktivnych , 0 neaktivny
 * ak je vstup 0 -> je opticka zavora neaktivna
 */
static inline bool limit_sw1_active(void) {
    return gpio_get_level(LIMIT_SW_1_GPIO) == 1;
}
static inline bool limit_sw2_active(void) {
    return gpio_get_level(LIMIT_SW_2_GPIO) == 1;
}




 /*
 * ************************************************************************
 * -------------------------------- ADC -----------------------------------
 * ************************************************************************
 * VBAT referencia je 1.1V
 * TODO IPROPI Vref = 1.1/0.25 = 4.4V
 *
 *
 */
static void adc_init(void)
{
    //pouzije sa ADC_unit_1 prevodnik
    adc_oneshot_unit_init_cfg_t ucfg = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&ucfg, &s_adc));

    adc_oneshot_chan_cfg_t ccfg = {
        .atten    = ADC_ATTEN_DB_0,          /* bez utlmu do 1.1V */
        .bitwidth = ADC_BITWIDTH_12,/* 12-bit */
    };
    //kanal pre VBAT, nastavenie
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, BAT_VOLT_ADC_CH, &ccfg));
    /* kalibracna schema pre VBAT kanal (Line Fitting - ESP32 classic) */
    adc_cali_line_fitting_config_t bat_cali_cfg = {
        .unit_id  = ADC_UNIT_1,
        .atten    = ccfg.atten,
        .bitwidth = ccfg.bitwidth,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&bat_cali_cfg, &s_bat_adc_cali));


    //kanal pre IPROPI
    ccfg.atten = ADC_ATTEN_DB_12;        //Vref*4
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, IPROPI_ADC_CH, &ccfg));
    /* kalibracna schema pre IPROPI kanal (Line Fitting - ESP32 classic) */
    adc_cali_line_fitting_config_t ipropi_cali_cfg = {
        .unit_id  = ADC_UNIT_1,
        .atten    = ccfg.atten,
        .bitwidth = ccfg.bitwidth,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&ipropi_cali_cfg, &s_ipropi_adc_cali));
}



/* 1 kHz frevencia vzorkovania, ak je aktivny rezim MOTOR_STOPPED meria BAT_VOLT, v MOTOR_RUNNING meria IPROPI. */
static void adc_timer_cb(void *arg)
{
    int raw = 0;
    if (g_motor_state == MOTOR_STOPPED) {
        //meranie VBAT
        if (adc_oneshot_read(s_adc, BAT_VOLT_ADC_CH, &raw) == ESP_OK) {
            meaavg_push(&s_battery_avg, (uint16_t)raw); //buff update
            //update Bat. volt, vypocet priemru
            g_battery_volt_raw = meaavg_get_sum(&s_battery_avg) >> s_battery_avg.size_bit;
        }
    } else {
        //meranie IPROPI
        if (adc_oneshot_read(s_adc, IPROPI_ADC_CH, &raw) == ESP_OK) {
            meaavg_push(&s_ipropi_avg, (uint16_t)raw);
            g_ipropi_volt_raw = meaavg_get_sum(&s_ipropi_avg) >> s_ipropi_avg.size_bit;
        }
    }
}

//pouzije timer na vzorkovanie ADC je ADC_SAMPLE_PERIOD_US (1kHz) , vola adc_timer_cb
static void adc_sampler_start(void)
{
    const esp_timer_create_args_t targs = {
        .callback = adc_timer_cb,
        .name     = "adc_sampler",
    };
    esp_timer_handle_t th;
    ESP_ERROR_CHECK(esp_timer_create(&targs, &th));
    ESP_ERROR_CHECK(esp_timer_start_periodic(th, ADC_SAMPLE_PERIOD_US));
}



/*
 * ************************************************************************
 * ----------------------- SWITCHING OFF -----------------------------------
 * *************************************************************************
 * vypone sa tlacitkom ak sa podrzi viac ako 3s ale menej ako 6s, zablikaju led a moze sa uvolnit tlacitko
 * alebo po zabrzdeni motora po urcitej dobe - obsluha je v riadeni motora
 * - vypne hlavny spinaci tranzistor
 */
static void power_off(void)
{
    motor_brake();
    gpio_set_level(ONOFF_GPIO, 0);   /* rozopne napajaci tranzistor */
    ESP_LOGI(TAG, "power off");
    for (;;) {
        vTaskDelay(portMAX_DELAY);    /* MCU coskoro strati napajanie */
    }
}


/*
 * ************************************************************************
 * ----------------------- BATTERY CHECK TASK -----------------------------
 * *************************************************************************
 * kontroluje stav baterie kazdych BATTERY_CHECK_CYCLE_TIME_MS
 * prpocita raw hodnotu na [V], podla napatia nastavy stav baterie g_battery_state a led_state
 * zapne, vypne LED
 */

static void battery_check_task(void *arg)
{

    static battery_led_state_t led_state = LED_BATT_OFF;
    static bool blink =false;

    for (;;) {
        int raw_mv = 0;
        //knvertuje raw hodnotu spriemerovanu s ADC na V, pouzije kalibracne hodnoty
        adc_cali_raw_to_voltage(s_bat_adc_cali, (int)g_battery_volt_raw, &raw_mv);
        float v = ((float)raw_mv / 1000.0f) * BAT_VOLT_DIVIDER_RATIO;
        g_bat_volt = v;

        ESP_LOGI(TAG, "Ubat = %.2f V (raw=%u)", v, (unsigned)g_battery_volt_raw);

        //podla napatia nastavy stav baterie g_battery_state
        if (v >= BATTERY_LOW_LEVEL_V) {
            g_battery_state = BATTERY_OK;
            led_state = LED_BATT_OFF;
        } else if (v > BATTERY_CRITICAL_LEVEL_V) {
            g_battery_state = BATTERY_LOW;
            led_state = LED_BATT_BLINK;
        } else {
            g_battery_state = BATTERY_CRITICAL;
            led_state = LED_BATT_ON;
         }

        //update stavvej ledky baterie
        switch (led_state){
            case LED_BATT_OFF:
                gpio_set_level(LED_BATT_GPIO, 1);   /* 1 = nesvieti */
            break;
            case LED_BATT_ON:
                gpio_set_level(LED_BATT_GPIO, 0);   /* 0 = svieti */
            break;
            case LED_BATT_BLINK:
                //blika s periodov 2*BATTERY_CHECK_CYCLE_TIME_MS (v polperiode volania tohto task-u)
                blink = !blink;
                gpio_set_level(LED_BATT_GPIO, blink ? 0 : 1);
            break;
        }
        //uloha sa opakuje kazdych BATTERY_CHECK_CYCLE_TIME_MS
        vTaskDelay(pdMS_TO_TICKS(BATTERY_CHECK_CYCLE_TIME_MS));
    }
}





/*
 * ************************************************************************
 * ----------------------- MOTOR CONTROL TASK -----------------------------
 * ************************************************************************
 * motor sa spusti ak sa aktiviuje LIMIT_SW_1 a bateria je v stave OK, alebo LOW
 * vypne sa ked sa aktivuje LIMIT_SW_2, ale pocka este LIMIT_SW2_BRAKE_DELAY_MS, potom sa vypne
 * snima sa hrana signalu.
 * Po spusteni spinkovaca sa motor automaticky spusti. Naspusti sa len v pripade ze je aktivny LIMIT_SW_2.
  */

static void motor_control_task(void *arg)
{
    bool first_start = true; //prvy start motora po spusteni
    bool prev_limit_sw1 = limit_sw1_active();
    bool prev_limit_sw2 = limit_sw2_active();
    //cas motora v stave necinnosti
    uint64_t t_motor_idle = esp_timer_get_time();

    motor_brake();
    g_motor_state = MOTOR_STOPPED;

    for (;;) {
        //stav LIMIT_SW1,2
        bool sw1 = limit_sw1_active();
        bool sw2 = limit_sw2_active();
        /* SW_edge = 1 ak je hrana not active -> active */
        bool sw1_edge = (!prev_limit_sw1 && sw1);
        bool sw2_edge = (!prev_limit_sw2 && sw2);


        if (g_motor_state == MOTOR_STOPPED) {
            // MOTOR_STOPPED state
            battery_state_t bs = g_battery_state;
            bool battery_ok = (bs == BATTERY_OK || bs == BATTERY_LOW);
            bool do_start;

            if (first_start) {
                /* Prvy start po boote: spusti hned, pokial nie je aktivny SW2 */
                if (!sw2 && battery_ok) {
                    do_start = true;
                }
                else do_start = false;
                //Ale hned nie je uplne skontrolovana bateria (kym sa zaplni bufer), tak este pocka kym sa zmeria
                //napatie a bud bat OK, lebo inak by sa moto nespustil, kedze po prvom starte uz caka na hranu.
                if (battery_ok) {
                    first_start = false;
                }

            } else {
                /* Dalsie starty: aktivacna hrana SW1 + bat OK */
                do_start = (sw1_edge && battery_ok);
            }

            if (do_start) {
                motor_run();
                g_motor_state = MOTOR_RUNNING;
                g_cycle_count++;
                ESP_LOGI(TAG, "motor start, cyklus %u", (unsigned)g_cycle_count);
            }
            else {
                //kontroluje ci vyprsal cas pre vypnutie, ak ano vypne
                if (esp_timer_get_time() - t_motor_idle >= MOTOR_IDLE_TIMEOUT_US ) {
                    gpio_set_level(LED_MODE_GPIO, !gpio_get_level(LED_MODE_GPIO));
                    power_off();
                }
            }
        }

        else {
            // MOTOR_RUNNING state
            if (sw2_edge) {
                vTaskDelay(pdMS_TO_TICKS(LIMIT_SW2_BRAKE_DELAY_MS));   //pocka chvilu aby vyslo zo zaberu
                motor_brake();
                g_motor_state = MOTOR_STOPPED;
                ESP_LOGI(TAG, "motor brzda (LIMIT_SW_2)");
                t_motor_idle = esp_timer_get_time();
            }
        }

        prev_limit_sw1 = sw1;
        prev_limit_sw2 = sw2;

        vTaskDelay(pdMS_TO_TICKS(MOTOR_CONTROL_CYCLE_TIME_MS));
    }
}





/*
 * ************************************************************************
 * -------------------------ONOFF SWITCH CHCK TASK-----------------------
 * ************************************************************************
*/

/*
 Debounce 50 ms pri vzorkovani 10 ms => 5 stabilnych vzoriek.
 return value:
 1 - stlacene
 0 - uvolnene
 */
static bool onoff_button_debounced(void)
{
    static bool stable = false;
    static int  cnt = 0;
    bool raw = (gpio_get_level(ONOFF_BUTTON_GPIO) == 0);   /* stlacene = 0 */
    if (raw != stable) {
        cnt++;
        if (cnt >= (ONOFF_BUTTON_DEBOUNCE_MS / BUTTON_CYCLE_TIME_MS)) {
            stable = raw;
            cnt = 0;
        }
    } else {
        cnt = 0;
    }
    return stable;
}


/* stav tlacitka ON_OFF
 * ak sa stlaci ked je vsetko vypnute, privedie sa napatie na hlavny spinaci tranzistor a privedie sa napatie na MCU
 * ten potom nastavy ONOFF_GPIO a to uz drzi tranzistor zopnuty.
 * Ak sa tlacitko drzi viac ako SWITCHING_OFF_HOLD_MS (3s) a menej ako CHANGE_MODE_HOLD_MS (6s) vsetko sa vypne
 * ak sa drzi viac ako CHANGE_MODE_HOLD_MS prepne sa rezim BT/WIFi
 * TODO BT/WIFI
 */
static void onoff_button_task(void *arg)
{
    bool pressed = false;
    uint64_t t_press = esp_timer_get_time();
    uint64_t dur_us;

    /* led_cnt: ked je > 1 ma zablikat led, signalizujuce ze po uvolneni tl. sa spinkovac vypne.,
     ked je 1 prepne stav led, pre signalizaciu zmeny modu, zmeni mod po pusteni tl.
     ked je 0 uz nic. */
    int led_cnt = 0;
    int led_sub_cnt = 0;

    for (;;) {
        bool now_pressed = onoff_button_debounced();

        if (!pressed && now_pressed) {
            //prave stlacene tlacitko
            pressed = true;
            t_press =  esp_timer_get_time();

            //init led cnt -> 3x blikne, led sa musi vratt do stavu ako bola predtym- tak parny pocet 7-1 = 6/2=3
            //lebo mophol a nemusel byt aktivny BT/WIFI rezim a tato ledka blika....
            led_cnt = 7;
            led_sub_cnt = 0; // pocita periodu blikania 2*(15*10ms)

        } else if (pressed && !now_pressed) {
            //prave uvolnene tlacitko
            pressed = false;
            //ako dlho bolo stlacene
            dur_us = (esp_timer_get_time() - t_press);

            if (!g_wifi_mode_active) {
                //ak je aktivny normalny mod
                if (dur_us >= CHANGE_MODE_HOLD_US ) {
                    /* >6 s: vstup do mode rezimu */
                    g_wifi_mode_active = true;
                    gpio_set_level(LED_MODE_GPIO, 0);   /* LED2 svieti */
                    ESP_LOGI(TAG, "BT/WiFi mode ON");
                    /* TODO: zapnut BT/WiFi a spustit BT/WIFI mode funkcionalitu */


                } else if (dur_us >= SWITCHING_OFF_HOLD_US) {
                    //vypnut vsetko
                    power_off();                    /* nevrati sa */
                }
            }
            else {
                //ak je aktivny wifi mod
                if (dur_us >= CHANGE_MODE_HOLD_US ) {
                    /* >6 s: navrat z mode rezimu */
                    g_wifi_mode_active = false;
                    gpio_set_level(LED_MODE_GPIO, 1);   /* LED2 zhasne */
                    //TODO: vypnut BT/WIFI


                    ESP_LOGI(TAG, "BT/WiFi mode OFF");
                } else if (dur_us >= SWITCHING_OFF_HOLD_US) {
                    power_off();                    /* nevrati sa */
                }
            }
        }
        else if (pressed) {
            /* len koli ledkam je toto tu
            * bliknu ledky pre vypnutie
            */
            dur_us = (esp_timer_get_time() - t_press);
            if (dur_us >= SWITCHING_OFF_HOLD_US && led_cnt > 1) {
                if (led_sub_cnt == 0) {
                    gpio_set_level(LED_MODE_GPIO, !gpio_get_level(LED_MODE_GPIO));
                    led_cnt--;
                    led_sub_cnt = 15; //2* 15*10ms perioda blikania
                }
                led_sub_cnt--;
            }
            else if (dur_us >= CHANGE_MODE_HOLD_US && led_cnt == 1){
                gpio_set_level(LED_MODE_GPIO, !gpio_get_level(LED_MODE_GPIO));
                led_cnt = 0;
            }

        }
        vTaskDelay(pdMS_TO_TICKS(BUTTON_CYCLE_TIME_MS));
    }
}

/*
 * ************************************************************************
 * ------------------------------ APP MAIN --------------------------------
 * ************************************************************************
*/
void app_main(void)
{
    //inicializacia
    gpio_init_all();
    motor_pwm_init();
    adc_init();
    adc_sampler_start();

    //spustni taskov
    xTaskCreate(motor_control_task, "control", 4096, NULL, 6, NULL);
    xTaskCreate(onoff_button_task,  "button",  4096, NULL, 5, NULL);
    xTaskCreate(battery_check_task,  "battery",  4096, NULL, 4, NULL);
    ESP_LOGI(TAG, "start");
}
