#include "jdesp.h"

#include "driver/ledc.h"
#include "hal/ledc_ll.h"
#include "hal/gpio_ll.h"
#include "esp_rom_gpio.h"

#include "esp_private/periph_ctrl.h"

#define LEDC_TIMER_DIV_NUM_MAX (0x3FFFF)

typedef struct timer_info {
    uint8_t tim_num;
    uint8_t bits;
    uint32_t div;
    uint32_t period;
} timer_info_t;

typedef struct channel_info {
    uint8_t ch_num;
    uint8_t pin;
    timer_info_t timer;
} channel_info_t;

static timer_info_t timers[LEDC_TIMER_MAX];
static channel_info_t channels[LEDC_CHANNEL_MAX];

uint8_t cpu_mhz = APB_CLK_FREQ / 1000000;

static void apply_config(timer_info_t *t) {
    ledc_timer_t tim = t->tim_num;

    ledc_ll_set_clock_divider(&LEDC, LEDC_LOW_SPEED_MODE, tim, t->div);
#if !defined(CONFIG_IDF_TARGET_ESP32C3) && !defined(CONFIG_IDF_TARGET_ESP32S3)
    ledc_ll_set_clock_source(&LEDC, LEDC_LOW_SPEED_MODE, tim, LEDC_APB_CLK);
#endif
    ledc_ll_set_duty_resolution(&LEDC, LEDC_LOW_SPEED_MODE, tim, t->bits);
    ledc_ll_ls_timer_update(&LEDC, LEDC_LOW_SPEED_MODE, tim);

    ledc_ll_timer_rst(&LEDC, LEDC_LOW_SPEED_MODE, tim);
    ledc_ll_ls_timer_update(&LEDC, LEDC_LOW_SPEED_MODE, tim);
}

static void init(void) {
    if (timers[1].tim_num == 1)
        return;
    for (int i = 0; i < LEDC_TIMER_MAX; ++i) {
        timers[i].tim_num = i;
        timers[i].period = 0x00;
    }
    for (int i = 0; i < LEDC_CHANNEL_MAX; ++i) {
        channels[i].ch_num = i;
        channels[i].pin = 0xff;
    }
    periph_module_enable(PERIPH_LEDC_MODULE);
    ledc_ll_set_slow_clk_sel(&LEDC, LEDC_SLOW_CLK_APB);
}

uint8_t jd_pwm_init(uint8_t pin, uint32_t period, uint32_t duty, uint8_t prescaler) {
    init();

    uint32_t period_cycles = period * prescaler;
    timer_info_t *t = NULL;
    channel_info_t *ch = NULL;

    for (int bits = LEDC_TIMER_BIT_MAX - 1; bits >= 4; bits--) {
        uint32_t div = (period_cycles << 8) >> bits;
        if (div < 256)
            continue;
        if (div > LEDC_TIMER_DIV_NUM_MAX) {
            DMESG("! PWM too fast");
            hw_panic();
        }

        for (unsigned i = 0; i < LEDC_TIMER_MAX; ++i) {
            t = &timers[i];
            if (i > 0 && t->period == period && t->div == div) // reuse after second timer (first is reserved?)
                break;
            t = NULL;
        }

        if (t == NULL)
            for (unsigned i = 0; i < LEDC_TIMER_MAX; ++i) {
                t = &timers[i];
                if (t->period == 0x00) {
                    break;
                }
                t = NULL;
            }

        if (t == NULL) {
            DMESG("! out of LEDC timers");

            hw_panic();
        }

        for (unsigned i = 0; i < LEDC_CHANNEL_MAX; ++i) {
            ch = &channels[i];
            if (ch->pin == pin)
                break;
            ch = NULL;
        }

        if (ch == NULL)
            for (unsigned i = 0; i < LEDC_CHANNEL_MAX; ++i) {
                ch = &channels[i];
                if (ch->pin == 0xff) {
                    break;
                }
                ch = NULL;
            }

        if (ch == NULL) {
            DMESG("! out of LEDC channels");

            hw_panic();
        }

        t->div = div;
        t->bits = bits;
        t->period = period;

        // DMESG("! === TIMER ===");
        // DMESG("! TIMER num: %d", t->tim_num);
        // DMESG("! TIMER period: %lu", t->period);
        // DMESG("! TIMER div: %lu", t->div);
        // DMESG("! === TIMER ===");

        ch->pin = pin;
        ch->timer = *t;

        // DMESG("! === CHANNEL ===");
        // DMESG("! CHANNEL num: %d", ch->ch_num);
        // DMESG("! CHANNEL pin: %d", ch->pin);
        // DMESG("! CHANNEL timer num: %d", (&ch->timer)->tim_num);
        // DMESG("! CHANNEL timer period: %lu", (&ch->timer)->period);
        // DMESG("! === CHANNEL ===");

        apply_config(t);

        break;
    }

    int ch_num = ch->ch_num; // we don't bind timers to channels 1-1
    int tim = t->tim_num;
    int pwm_id = ch->ch_num + 1;

    jd_pwm_set_duty(pwm_id, duty);

    ledc_ll_bind_channel_timer(&LEDC, LEDC_LOW_SPEED_MODE, ch_num, tim);
    ledc_ll_ls_channel_update(&LEDC, LEDC_LOW_SPEED_MODE, ch_num);

    jd_pwm_enable(pwm_id, 1);

    return pwm_id;
}

void jd_pwm_set_duty(uint8_t pwm_id, uint32_t duty) {
    JD_ASSERT(pwm_id > 0);
    JD_ASSERT(pwm_id <= LEDC_CHANNEL_MAX);

    channel_info_t *ch = &channels[pwm_id - 1];
    timer_info_t *t = &ch->timer;

    int ch_num = ch->ch_num;

    uint32_t max = 1 << t->bits;
    duty = max * duty / t->period;
    if (duty >= max)
        duty = max - 1;

    ledc_ll_set_duty_int_part(&LEDC, LEDC_LOW_SPEED_MODE, ch_num, duty);
    ledc_ll_set_duty_direction(&LEDC, LEDC_LOW_SPEED_MODE, ch_num, 1);
    ledc_ll_set_duty_num(&LEDC, LEDC_LOW_SPEED_MODE, ch_num, 0);
    ledc_ll_set_duty_cycle(&LEDC, LEDC_LOW_SPEED_MODE, ch_num, 0);
    ledc_ll_set_duty_scale(&LEDC, LEDC_LOW_SPEED_MODE, ch_num, 0);
    ledc_ll_ls_channel_update(&LEDC, LEDC_LOW_SPEED_MODE, ch_num);

    ledc_ll_set_sig_out_en(&LEDC, LEDC_LOW_SPEED_MODE, ch_num, true);
    ledc_ll_set_duty_start(&LEDC, LEDC_LOW_SPEED_MODE, ch_num, true);
    ledc_ll_ls_channel_update(&LEDC, LEDC_LOW_SPEED_MODE, ch_num);
}

void jd_pwm_enable(uint8_t pwm_id, bool enabled) {
    JD_ASSERT(pwm_id > 0);
    JD_ASSERT(pwm_id <= LEDC_CHANNEL_MAX);

    channel_info_t *ch = &channels[pwm_id - 1];

    int ch_num = ch->ch_num;

    pin_setup_output(ch->pin);
    if (enabled) {
        bool output_invert = false;
        esp_rom_gpio_connect_out_signal(
            ch->pin, ledc_periph_signal[LEDC_LOW_SPEED_MODE].sig_out0_idx + ch_num, output_invert, 0);
    }
}

void pin_set(int pin, int v) {
    if ((uint8_t)pin != NO_PIN)
        gpio_set_level(pin, v);
}

void pin_setup_output(int pin) {
    if ((uint8_t)pin != NO_PIN) {
        gpio_ll_iomux_func_sel(GPIO_PIN_MUX_REG[pin], PIN_FUNC_GPIO);
        CHK(gpio_set_direction(pin, GPIO_MODE_OUTPUT));
        esp_rom_gpio_connect_out_signal(pin, SIG_GPIO_OUT_IDX, false, false);
    }
}

int pin_get(int pin) {
    if ((uint8_t)pin == NO_PIN)
        return -1;
    return gpio_get_level(pin);
}

void pin_setup_input(int pin, int pull) {
    if ((uint8_t)pin == NO_PIN)
        return;
    gpio_ll_iomux_func_sel(GPIO_PIN_MUX_REG[pin], PIN_FUNC_GPIO);
    pin_set_pull(pin, pull);
    CHK(gpio_set_direction(pin, GPIO_MODE_INPUT));
}

void pin_set_pull(int pin, int pull) {
    if ((uint8_t)pin == NO_PIN)
        return;

    if (pull < 0) {
        gpio_pulldown_en(pin);
        gpio_pullup_dis(pin);
    } else if (pull > 0) {
        gpio_pullup_en(pin);
        gpio_pulldown_dis(pin);
    } else {
        gpio_pullup_dis(pin);
        gpio_pulldown_dis(pin);
    }
}

void pin_setup_analog_input(int pin) {
    if ((uint8_t)pin == NO_PIN)
        return;
    gpio_ll_iomux_func_sel(GPIO_PIN_MUX_REG[pin], PIN_FUNC_GPIO);
    pin_set_pull(pin, PIN_PULL_NONE);
    CHK(gpio_set_direction(pin, GPIO_MODE_DISABLE));
}

void pwr_enter_no_sleep(void) {}
void pwr_enter_tim(void) {}
void pwr_leave_tim(void) {}

void pwr_enter_pll(void) {}
void pwr_leave_pll(void) {}

void power_pin_enable(int en) {}
