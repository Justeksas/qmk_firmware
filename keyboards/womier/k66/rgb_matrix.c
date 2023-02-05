// Copyright 2021 https://github.com/fightforlife
// SPDX-License-Identifier: GPL-2.0-or-later

#include "rgb_matrix.h"
#include "k66.h"

/* Womier K66 v1
    ROWS RGB PWM
    SS8050 transistors NPN driven high
    base      - GPIO (PWM Row Pin)
    collector - LED RGB PWM row pins Cathode
    emitter   - GND

    GND     GPIO
    (E)     (B)
     |  NPN  |
     |_______|
         |
         |
        (C)
        LED

    COLS key / led
    SS8850 transistors PNP driven low
    base      - GPIO (col PIN)
    collector - LED RGB common Anode
    emitter   - VDD

        LED
        (C)
         |
         |
      _______
     |  NPN  |
     |       |
    (B)     (E)
    GPIO    VDD
*/
static uint8_t chan_row_order[LED_MATRIX_ROWS_HW] = {0}; // track the channel row order for PWM
static uint8_t current_key_col = 0; // current key col, scanner
extern matrix_row_t raw_matrix[MATRIX_ROWS]; //raw values
/*PWM ticks for a full period, PWM colors from 0-255*/
static const uint32_t periodticks = 256;
/* PWM Timer clock in Hz. Can be quite low, since we are manually advancing timer after successfull scan. Here 0,25s */
static const uint32_t freq = periodticks*4;
static const pin_t led_row_pins[LED_MATRIX_ROWS_HW] = LED_MATRIX_ROW_PINS; // We expect a RGB order here
static const pin_t led_col_pins[LED_MATRIX_COLS] = LED_MATRIX_COL_PINS;
RGB led_state[MATRIX_LED_TOTAL]; // led state buffer



void rgb_ch_ctrl(void) {
    /* Enable PWM function, IOs and select the PWM modes for the LED column pins */
    for(uint8_t i = 0; i < LED_MATRIX_ROWS_HW; i++) {
        switch(led_row_pins[i]) {
            // Intentional fall-through for the PWM B-pin mapping, not available on SN32F26XX
            case A0:
                chan_row_order[i] = 0;
                break;
            case A1:
                chan_row_order[i] = 1;
                break;
            case A2:
                chan_row_order[i] = 2;
                break;
            case A3:
                chan_row_order[i] = 3;
                break;
            case A4:
                chan_row_order[i] = 4;
                break;
            case A5:
                chan_row_order[i] = 5;
                break;
            case A6:
                chan_row_order[i] = 6;
                break;
            case A7:
                chan_row_order[i] = 7;
                break;
            case A8:
                chan_row_order[i] = 8;
                break;
            case A9:
                chan_row_order[i] = 9;
                break;
            case A10:
                chan_row_order[i] = 10;
                break;
            case A11:
                chan_row_order[i] = 11;
                break;
            case A12:
                chan_row_order[i] = 12;
                break;
            case A13:
                chan_row_order[i] = 13;
                break;
            case A14:
                chan_row_order[i] = 14;
                break;
            case A15:
                chan_row_order[i] = 15;
                break;
            case D0:
                chan_row_order[i] = 16;
                break;
            case D1:
                chan_row_order[i] = 17;
                break;
            case D2:
                chan_row_order[i] = 18;
                break;
            case D3:
                chan_row_order[i] = 19;
                break;
            case D4:
                chan_row_order[i] = 20;
                break;
            case D5:
                chan_row_order[i] = 21;
                break;
            case D8:
                chan_row_order[i] = 22;
                break;
        }
    }
}

void rgb_callback(PWMDriver *pwmp);

/* PWM configuration structure. We use timer CT16B1 with 23 channels. */
static const PWMConfig pwmcfg = {
    freq,         /* PWM clock frequency. */
    periodticks,  /* PWM period (in ticks) 1S (1/10kHz=0.1mS 0.1ms*10000 ticks=1S) */
    rgb_callback, /* led Callback */
    .channels =
        {
/* Default all channels to disabled - Channels will be configured durring init */
#ifdef ACTIVATE_PWM_CHAN_0
            [0] = {.mode = PWM_OUTPUT_ACTIVE_HIGH},
#else
            [0]  = {.mode = PWM_OUTPUT_DISABLED},
#endif
#ifdef ACTIVATE_PWM_CHAN_1
            [1] = {.mode = PWM_OUTPUT_ACTIVE_HIGH},
#else
            [1]  = {.mode = PWM_OUTPUT_DISABLED},
#endif
#ifdef ACTIVATE_PWM_CHAN_2
            [2] = {.mode = PWM_OUTPUT_ACTIVE_HIGH},
#else
            [2]  = {.mode = PWM_OUTPUT_DISABLED},
#endif
#ifdef ACTIVATE_PWM_CHAN_3
            [3] = {.mode = PWM_OUTPUT_ACTIVE_HIGH},
#else
            [3]  = {.mode = PWM_OUTPUT_DISABLED},
#endif
#ifdef ACTIVATE_PWM_CHAN_4
            [4] = {.mode = PWM_OUTPUT_ACTIVE_HIGH},
#else
            [4]  = {.mode = PWM_OUTPUT_DISABLED},
#endif
#ifdef ACTIVATE_PWM_CHAN_5
            [5] = {.mode = PWM_OUTPUT_ACTIVE_HIGH},
#else
            [5]  = {.mode = PWM_OUTPUT_DISABLED},
#endif
#ifdef ACTIVATE_PWM_CHAN_6
            [6] = {.mode = PWM_OUTPUT_ACTIVE_HIGH},
#else
            [6]  = {.mode = PWM_OUTPUT_DISABLED},
#endif
#ifdef ACTIVATE_PWM_CHAN_7
            [7] = {.mode = PWM_OUTPUT_ACTIVE_HIGH},
#else
            [7]  = {.mode = PWM_OUTPUT_DISABLED},
#endif
#ifdef ACTIVATE_PWM_CHAN_8
            [8] = {.mode = PWM_OUTPUT_ACTIVE_HIGH},
#else
            [8]  = {.mode = PWM_OUTPUT_DISABLED},
#endif
#ifdef ACTIVATE_PWM_CHAN_9
            [9] = {.mode = PWM_OUTPUT_ACTIVE_HIGH},
#else
            [9]  = {.mode = PWM_OUTPUT_DISABLED},
#endif
#ifdef ACTIVATE_PWM_CHAN_10
            [10] = {.mode = PWM_OUTPUT_ACTIVE_HIGH},
#else
            [10] = {.mode = PWM_OUTPUT_DISABLED},
#endif
#ifdef ACTIVATE_PWM_CHAN_11
            [11] = {.mode = PWM_OUTPUT_ACTIVE_HIGH},
#else
            [11] = {.mode = PWM_OUTPUT_DISABLED},
#endif
#ifdef ACTIVATE_PWM_CHAN_12
            [12] = {.mode = PWM_OUTPUT_ACTIVE_HIGH},
#else
            [12] = {.mode = PWM_OUTPUT_DISABLED},
#endif
#ifdef ACTIVATE_PWM_CHAN_13
            [13] = {.mode = PWM_OUTPUT_ACTIVE_HIGH},
#else
            [13] = {.mode = PWM_OUTPUT_DISABLED},
#endif
#ifdef ACTIVATE_PWM_CHAN_14
            [14] = {.mode = PWM_OUTPUT_ACTIVE_HIGH},
#else
            [14] = {.mode = PWM_OUTPUT_DISABLED},
#endif
#ifdef ACTIVATE_PWM_CHAN_15
            [15] = {.mode = PWM_OUTPUT_ACTIVE_HIGH},
#else
            [15] = {.mode = PWM_OUTPUT_DISABLED},
#endif
#ifdef ACTIVATE_PWM_CHAN_16
            [16] = {.mode = PWM_OUTPUT_ACTIVE_HIGH},
#else
            [16] = {.mode = PWM_OUTPUT_DISABLED},
#endif
#ifdef ACTIVATE_PWM_CHAN_17
            [17] = {.mode = PWM_OUTPUT_ACTIVE_HIGH},
#else
            [17] = {.mode = PWM_OUTPUT_DISABLED},
#endif
#ifdef ACTIVATE_PWM_CHAN_18
            [18] = {.mode = PWM_OUTPUT_ACTIVE_HIGH},
#else
            [18] = {.mode = PWM_OUTPUT_DISABLED},
#endif
#ifdef ACTIVATE_PWM_CHAN_19
            [19] = {.mode = PWM_OUTPUT_ACTIVE_HIGH},
#else
            [19] = {.mode = PWM_OUTPUT_DISABLED},
#endif /* Channel 20 is a dummy channel in 26x .*/
            [20] = {.mode = PWM_OUTPUT_DISABLED},
#ifdef ACTIVATE_PWM_CHAN_21
            [21] = {.mode = PWM_OUTPUT_ACTIVE_HIGH},
#else
            [21] = {.mode = PWM_OUTPUT_DISABLED},
#endif
#ifdef ACTIVATE_PWM_CHAN_22
            [22] = {.mode = PWM_OUTPUT_ACTIVE_HIGH},
#else
            [22] = {.mode = PWM_OUTPUT_DISABLED},
#endif
        },
    0 /* HW dependent part.*/
};


void shared_matrix_rgb_enable(void) {
    pwmEnablePeriodicNotification(&PWMD1);
}

void shared_matrix_rgb_disable_pwm(void) {
    // Disable PWM outputs on row HW pins
    for(uint8_t y = 0; y < LED_MATRIX_ROWS_HW; y++) {
        pwmDisableChannel(&PWMD1,chan_row_order[y]);
    }
}

void shared_matrix_rgb_disable_leds(void) {
    // Disable LED outputs on RGB channel pins
    for (uint8_t x = 0; x < LED_MATRIX_COLS; x++) {
        setPinOutput(led_col_pins[x]);
        writePinHigh(led_col_pins[x]);
    }
}
void update_pwm_channels(PWMDriver *pwmp) {
    for(uint8_t current_key_row = 0; current_key_row < MATRIX_ROWS; current_key_row++) {
        #if(DIODE_DIRECTION == COL2ROW)
            matrix_scan_keys(raw_matrix, current_key_row);
        #endif
        uint8_t led_index = g_led_config.matrix_co[current_key_row][current_key_col];
        // Update matching RBG channel PWM configuration
        if (led_state[led_index].g != 0) pwmEnableChannelI(pwmp,chan_row_order[(current_key_row*3)+0],led_state[led_index].g);
        if (led_state[led_index].b != 0) pwmEnableChannelI(pwmp,chan_row_order[(current_key_row*3)+1],led_state[led_index].b);
        if (led_state[led_index].r != 0) pwmEnableChannelI(pwmp,chan_row_order[(current_key_row*3)+2],led_state[led_index].r);
        // setting Anode high if needed
        if (led_state[led_index].r != 0 || led_state[led_index].g != 0 || led_state[led_index].b != 0) 
        {
            setPinOutput(led_col_pins[current_key_col]);
            writePinLow(led_col_pins[current_key_col]);
        }
    }
}

void rgb_callback(PWMDriver *pwmp) {
    // Disable the interrupt
    pwmDisablePeriodicNotification(pwmp);
    // Advance to the next key col
    current_key_col++;
    if(current_key_col >= MATRIX_COLS) current_key_col = 0;
    chSysLockFromISR();
    // Disable LED output before scanning the key matrix
    shared_matrix_rgb_disable_leds();
    shared_matrix_rgb_disable_pwm();
    #if(DIODE_DIRECTION == ROW2COL)
        // Scan the key matrix column
        matrix_scan_keys(raw_matrix,current_key_col);
    #endif
    update_pwm_channels(pwmp);
    chSysUnlockFromISR();
    // Advance the timer to just before the wrap-around, that will start a new PWM cycle
    pwm_lld_change_counter(pwmp, 0xFFFF);
    // Enable the interrupt
    pwmEnablePeriodicNotification(pwmp);
}

void init(void) {
    init_underglow(UNDERGLOW_I2C_ADR);
    for (uint8_t x = 0; x < LED_MATRIX_COLS; x++) {
        setPinOutput(led_col_pins[x]);
        writePinHigh(led_col_pins[x]);
    }
    // Determine which PWM channels we need to control
    rgb_ch_ctrl();
    pwmStart(&PWMD1, &pwmcfg);
    shared_matrix_rgb_enable();
}

void set_color(int index, uint8_t r, uint8_t g, uint8_t b) {
     if(index<MATRIX_LED_TOTAL) {     
        led_state[index].r = r;
        led_state[index].g = g;
        led_state[index].b = b;
     }else{
         // user I2C underglow when LED not in key matrix
         set_i2c_color_direct(index, r, g, b);
     }
}

void set_color_all(uint8_t r, uint8_t g, uint8_t b) {
    for (int i=0; i<DRIVER_LED_TOTAL; i++) {
        set_color(i, r, g, b);
    }
}


static void flush(void) {
}

const rgb_matrix_driver_t rgb_matrix_driver = {
    .init          = init,
    .flush         = flush,
    .set_color     = set_color,
    .set_color_all = set_color_all,
};
