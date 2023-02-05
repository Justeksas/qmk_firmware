#include "rgb_matrix.h"
#include "rgb_matrix_sn32f268f.h"

#if !defined(RGB_MATRIX_HUE_STEP)
#    define RGB_MATRIX_HUE_STEP 8
#endif

#if !defined(RGB_MATRIX_SAT_STEP)
#    define RGB_MATRIX_SAT_STEP 16
#endif

#if !defined(RGB_MATRIX_VAL_STEP)
#    define RGB_MATRIX_VAL_STEP 16
#endif

#if !defined(RGB_MATRIX_SPD_STEP)
#    define RGB_MATRIX_SPD_STEP 16
#endif

#if !defined(RGB_MATRIX_MAXIMUM_BRIGHTNESS) || RGB_MATRIX_MAXIMUM_BRIGHTNESS > UINT8_MAX
#    undef RGB_MATRIX_MAXIMUM_BRIGHTNESS
#    define RGB_MATRIX_MAXIMUM_BRIGHTNESS UINT8_MAX
#endif

#define ROWS_PER_HAND (MATRIX_ROWS)

#if !defined(MATRIX_IO_DELAY)
#    define MATRIX_IO_DELAY 30
#endif

#if !defined(PWM_OUTPUT_ACTIVE_LEVEL)
#    define PWM_OUTPUT_ACTIVE_LEVEL PWM_OUTPUT_ACTIVE_LOW
#endif

#define RGB_OUTPUT_ACTIVE_HIGH PWM_OUTPUT_ACTIVE_HIGH
#define RGB_OUTPUT_ACTIVE_LOW PWM_OUTPUT_ACTIVE_LOW

#if !defined(RGB_OUTPUT_ACTIVE_LEVEL)
#    define RGB_OUTPUT_ACTIVE_LEVEL RGB_OUTPUT_ACTIVE_HIGH
#endif

#define HARDWARE_PWM 0
#define SOFTWARE_PWM 1
#if !defined(SN32_PWM_CONTROL)
#    define SN32_PWM_CONTROL HARDWARE_PWM
#endif

/*
    Default configuration example

    COLS key / led
    SS8050 transistors NPN driven low
    base      - GPIO
    collector - LED Col pins
    emitter   - VDD

    VDD     GPIO
    (E)     (B)
     |  PNP  |
     |_______|
         |
         |
        (C)
        LED

    ROWS RGB
    SS8550 transistors PNP driven high
    base      - GPIO
    collector - LED RGB row pins
    emitter   - GND

        LED
        (C)
         |
         |
      _______
     |  NPN  |
     |       |
    (B)     (E)
    GPIO    GND
*/
#if (DIODE_DIRECTION == COL2ROW)
static uint8_t chan_col_order[LED_MATRIX_COLS] = {0}; // track the channel col order
static uint8_t current_row                     = 0;   // LED row scan counter
static uint8_t current_key_row                 = 0;   // key row scan counter
#    if (SN32_PWM_CONTROL == SOFTWARE_PWM)
static uint8_t led_duty_cycle[LED_MATRIX_COLS] = {0}; // track the channel duty cycle
#    endif
#elif (DIODE_DIRECTION == ROW2COL)
/* make sure to `#define MATRIX_UNSELECT_DRIVE_HIGH` in this configuration*/
static uint8_t      chan_row_order[LED_MATRIX_ROWS_HW] = {0}; // track the channel row order
static uint8_t      current_key_col                    = 0;   // key col scan counter
static uint8_t      last_key_col                       = 0;   // key col scan counter
static matrix_row_t row_shifter                        = MATRIX_ROW_SHIFTER;
#    if (SN32_PWM_CONTROL == SOFTWARE_PWM)
static uint8_t      led_duty_cycle[LED_MATRIX_ROWS_HW] = {0}; // track the channel duty cycle
#    endif
#endif
extern matrix_row_t   raw_matrix[MATRIX_ROWS];                  // raw values
extern matrix_row_t   matrix[MATRIX_ROWS];                      // debounced values
static matrix_row_t   shared_matrix[MATRIX_ROWS];               // scan values
static volatile bool  matrix_locked                    = false; // matrix update check
static volatile bool  matrix_scanned                   = false;
static const uint32_t periodticks                      = RGB_MATRIX_MAXIMUM_BRIGHTNESS;
static const uint32_t freq                             = 3670016; //value calculated by dexter93
static const pin_t    led_row_pins[LED_MATRIX_ROWS_HW] = LED_MATRIX_ROW_PINS; // We expect a R,B,G order here
static const pin_t    led_col_pins[LED_MATRIX_COLS]    = LED_MATRIX_COL_PINS;
static RGB            led_state[MATRIX_LED_TOTAL];     // led state buffer
//static RGB            led_state_buf[RGB_MATRIX_LED_COUNT]; // led state buffer, not used in SN32F268F because of RAM constraints
#ifdef UNDERGLOW_RBG                                       // handle underglow with flipped B,G channels
static const uint8_t underglow_leds[UNDERGLOW_LEDS] = UNDERGLOW_IDX;
#endif

void matrix_output_unselect_delay(uint8_t line, bool key_pressed) {
    for (int i = 0; i < TIME_US2I(MATRIX_IO_DELAY); ++i) {
        __asm__ volatile("" ::: "memory");
    }
}
bool matrix_available(void) {
    return matrix_scanned;
}

static void rgb_ch_ctrl(void) {
    /* Enable PWM function, IOs and select the PWM modes for the LED pins */
#if (DIODE_DIRECTION == COL2ROW)
    for (uint8_t i = 0; i < LED_MATRIX_COLS; i++) {
        switch(led_col_pins[i]) {
            // Intentional fall-through for the PWM B-pin mapping, not available on SN32F26XX
            case A0:
                chan_col_order[i] = 0;
                break;
            case A1:
                chan_col_order[i] = 1;
                break;
            case A2:
                chan_col_order[i] = 2;
                break;
            case A3:
                chan_col_order[i] = 3;
                break;
            case A4:
                chan_col_order[i] = 4;
                break;
            case A5:
                chan_col_order[i] = 5;
                break;
            case A6:
                chan_col_order[i] = 6;
                break;
            case A7:
                chan_col_order[i] = 7;
                break;
            case A8:
                chan_col_order[i] = 8;
                break;
            case A9:
                chan_col_order[i] = 9;
                break;
            case A10:
                chan_col_order[i] = 10;
                break;
            case A11:
                chan_col_order[i] = 11;
                break;
            case A12:
                chan_col_order[i] = 12;
                break;
            case A13:
                chan_col_order[i] = 13;
                break;
            case A14:
                chan_col_order[i] = 14;
                break;
            case A15:
                chan_col_order[i] = 15;
                break;
            case D0:
                chan_col_order[i] = 16;
                break;
            case D1:
                chan_col_order[i] = 17;
                break;
            case D2:
                chan_col_order[i] = 18;
                break;
            case D3:
                chan_col_order[i] = 19;
                break;
            case D4:
                chan_col_order[i] = 20;
                break;
            case D5:
                chan_col_order[i] = 21;
                break;
            case D8:
                chan_col_order[i] = 22;
                break;
        }
    }
#elif (DIODE_DIRECTION == ROW2COL)
    for (uint8_t i = 0; i < LED_MATRIX_ROWS_HW; i++) {
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
#endif
}

static void rgb_callback(PWMDriver *pwmp);

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



static void shared_matrix_rgb_enable(void) {
    //pwmcfg.callback = rgb_callback; Not used in SN32F268F since PWM channels are hardcoded to save RAM
    pwmEnablePeriodicNotification(&PWMD1);
}

#if (DIODE_DIRECTION == COL2ROW)

static void shared_matrix_rgb_disable_output(void) {
    // Disable PWM outputs on column pins
    for (uint8_t y = 0; y < LED_MATRIX_COLS; y++) {
#    if (SN32_PWM_CONTROL == HARDWARE_PWM)
        pwmDisableChannel(&PWMD1, chan_col_order[y]);
#    elif (SN32_PWM_CONTROL == SOFTWARE_PWM)
        setPinInput(led_col_pins[y]);
#    endif
    }
    // Disable LED outputs on RGB channel pins
    for (uint8_t x = 0; x < LED_MATRIX_ROWS_HW; x++) {
#    if (RGB_OUTPUT_ACTIVE_LEVEL == RGB_OUTPUT_ACTIVE_HIGH)
        writePinLow(led_row_pins[x]);
#    elif (RGB_OUTPUT_ACTIVE_LEVEL == RGB_OUTPUT_ACTIVE_LOW)
        writePinHigh(led_row_pins[x]);
#    endif
    }
}

static void update_pwm_channels(PWMDriver *pwmp) {
    // Advance to the next LED RGB channels
    current_row++;
    /* Check if counter has wrapped around, reset before the next pass */
    if (current_row == LED_MATRIX_ROWS_HW) current_row = 0;
    uint8_t last_key_row = current_key_row;
    // Advance to the next key matrix row
#    if (SN32_PWM_CONTROL == HARDWARE_PWM)
    if (current_row % LED_MATRIX_ROW_CHANNELS == 2) current_key_row++;
#    elif (SN32_PWM_CONTROL == SOFTWARE_PWM)
    if (current_row % LED_MATRIX_ROW_CHANNELS == 0) current_key_row++;
#    endif
    /* Check if counter has wrapped around, reset before the next pass */
    if (current_key_row == LED_MATRIX_ROWS) current_key_row = 0;
    // Disable LED output before scanning the key matrix
    shared_matrix_rgb_disable_output();
    // Scan the key matrix row
    static uint8_t first_scanned_row;
    if (!matrix_scanned) {
        if (!matrix_locked) {
            matrix_locked     = true;
            first_scanned_row = current_key_row;
        } else {
            if ((last_key_row != current_key_row) && (current_key_row == first_scanned_row)) {
                matrix_locked  = false;
                matrix_scanned = true;
            }
        }
        if (matrix_locked) {
            matrix_read_cols_on_row(shared_matrix, current_key_row);
        }
    }
    bool enable_pwm_output = false;
    for (uint8_t current_key_col = 0; current_key_col < LED_MATRIX_COLS; current_key_col++) {
        uint8_t led_index = g_led_config.matrix_co[current_key_row][current_key_col];
#    if (SN32_PWM_CONTROL == SOFTWARE_PWM)
        if (led_index > RGB_MATRIX_LED_COUNT) continue;
#    endif
        // Check if we need to enable RGB output
        if (led_state[led_index].b != 0) enable_pwm_output |= true;
        if (led_state[led_index].g != 0) enable_pwm_output |= true;
        if (led_state[led_index].r != 0) enable_pwm_output |= true;
            // Update matching RGB channel PWM configuration
#    if (SN32_PWM_CONTROL == HARDWARE_PWM)
        switch (current_row % LED_MATRIX_ROW_CHANNELS) {
            case 0:
                pwmEnableChannel(pwmp, chan_col_order[current_key_col], led_state[led_index].b);
                break;
            case 1:
                pwmEnableChannel(pwmp, chan_col_order[current_key_col], led_state[led_index].g);
                break;
            case 2:
                pwmEnableChannel(pwmp, chan_col_order[current_key_col], led_state[led_index].r);
                break;
            default:;
        }
#    elif (SN32_PWM_CONTROL == SOFTWARE_PWM)
        switch (current_row % LED_MATRIX_ROW_CHANNELS) {
            case 0:
                led_duty_cycle[current_key_col] = led_state[led_index].r;
#        if defined(EVISION_BOTCHED_RED_CHANNEL) // some keyboards have a 151k resistor value tied to the R channel instead of a 10k, as the rest.
                /* Boost the output for that channel maximizing the current draw by disabling other sinks */
#            if (RGB_OUTPUT_ACTIVE_LEVEL == RGB_OUTPUT_ACTIVE_HIGH)
                writePinLow(led_row_pins[current_row + 1]);
                writePinLow(led_row_pins[current_row + 2]);
#            elif (RGB_OUTPUT_ACTIVE_LEVEL == RGB_OUTPUT_ACTIVE_LOW)
                writePinHigh(led_row_pins[current_row + 1]);
                writePinHigh(led_row_pins[current_row + 2]);
#            endif
#        endif
                break;
            case 1:
                led_duty_cycle[current_key_col] = led_state[led_index].b;
                break;
            case 2:
                led_duty_cycle[current_key_col] = led_state[led_index].g;
                break;
            default:;
        }
#    endif
    }
    // Enable RGB output
    if (enable_pwm_output) {
#    if (RGB_OUTPUT_ACTIVE_LEVEL == RGB_OUTPUT_ACTIVE_HIGH)
        writePinHigh(led_row_pins[current_row]);
#    elif (RGB_OUTPUT_ACTIVE_LEVEL == RGB_OUTPUT_ACTIVE_LOW)
        writePinLow(led_row_pins[current_row]);
#    endif
    }
}
#elif (DIODE_DIRECTION == ROW2COL)

static void shared_matrix_rgb_disable_output(void) {
    // Disable LED outputs on RGB channel pins
    for (uint8_t x = 0; x < LED_MATRIX_COLS; x++) {
        // Unselect all columns before scanning the key matrix
#    if (RGB_OUTPUT_ACTIVE_LEVEL == RGB_OUTPUT_ACTIVE_LOW || defined(MATRIX_UNSELECT_DRIVE_HIGH))
        writePinHigh(led_col_pins[x]);
#    elif (RGB_OUTPUT_ACTIVE_LEVEL == RGB_OUTPUT_ACTIVE_HIGH)
        writePinLow(led_col_pins[x]);
#    endif
    }
}

static void update_pwm_channels(PWMDriver *pwmp) {
    // Disable LED output before scanning the key matrix
    shared_matrix_rgb_disable_output();

    // Scan the key matrix column
    static uint8_t first_scanned_col;
    if (!matrix_scanned) {
        if (!matrix_locked) {
            matrix_locked     = true;
            first_scanned_col = current_key_col;
        } else {
            if ((last_key_col != current_key_col) && (current_key_col == first_scanned_col)) {
                matrix_locked  = false;
                matrix_scanned = true;
            }
        }
        if (matrix_locked) {
            matrix_read_rows_on_col(shared_matrix, current_key_col, row_shifter);
        }
    }
#    if ((RGB_OUTPUT_ACTIVE_LEVEL == RGB_OUTPUT_ACTIVE_HIGH) && defined(MATRIX_UNSELECT_DRIVE_HIGH))
    // Disable all RGB columns before turning on PWM in case matrix read unselect high
    for (uint8_t x = 0; x < LED_MATRIX_COLS; x++) {
        writePinLow(led_col_pins[x]);
    }
#    endif
    /* Advance to the next LED RGB channel and get ready for the next pass */
    last_key_col = current_key_col;
    current_key_col++;
    row_shifter <<= 1;
    /* Check if counter has wrapped around, reset before the next pass */
    if (current_key_col == LED_MATRIX_COLS) {
        current_key_col = 0;
        row_shifter     = MATRIX_ROW_SHIFTER;
    }
    bool enable_pwm_output = false;
    for (uint8_t current_key_row = 0; current_key_row < MATRIX_ROWS; current_key_row++) {
        uint8_t led_index = g_led_config.matrix_co[current_key_row][current_key_col];
        if (led_index > RGB_MATRIX_LED_COUNT) continue;
        uint8_t led_row_id = (current_key_row * LED_MATRIX_ROW_CHANNELS);
        // Check if we need to enable RGB output
        if (led_state[led_index].b != 0) enable_pwm_output |= true;
        if (led_state[led_index].g != 0) enable_pwm_output |= true;
        if (led_state[led_index].r != 0) enable_pwm_output |= true;
            // Update matching RGB channel PWM configuration
#    if (SN32_PWM_CONTROL == HARDWARE_PWM)
        pwmEnableChannelI(pwmp, chan_row_order[(led_row_id + 0)], led_state[led_index].r);
        pwmEnableChannelI(pwmp, chan_row_order[(led_row_id + 1)], led_state[led_index].b);
        pwmEnableChannelI(pwmp, chan_row_order[(led_row_id + 2)], led_state[led_index].g);
#    elif (SN32_PWM_CONTROL == SOFTWARE_PWM)
        led_duty_cycle[(led_row_id + 0)] = led_state[led_index].r;
        led_duty_cycle[(led_row_id + 1)] = led_state[led_index].b;
        led_duty_cycle[(led_row_id + 2)] = led_state[led_index].g;
#    endif
    }
    // Enable RGB output
    if (enable_pwm_output) {
#    if (RGB_OUTPUT_ACTIVE_LEVEL == RGB_OUTPUT_ACTIVE_HIGH)
        writePinHigh(led_col_pins[current_key_col]);
#    elif (RGB_OUTPUT_ACTIVE_LEVEL == RGB_OUTPUT_ACTIVE_LOW)
        writePinLow(led_col_pins[current_key_col]);
#    endif
    }
}
#endif // DIODE_DIRECTION == ROW2COL

static void rgb_callback(PWMDriver *pwmp) {
    // Disable the interrupt
    pwmDisablePeriodicNotification(pwmp);
#if ((SN32_PWM_CONTROL == SOFTWARE_PWM) && (DIODE_DIRECTION == COL2ROW))
    for (uint8_t pwm_cnt = 0; pwm_cnt < (LED_MATRIX_COLS * RGB_MATRIX_HUE_STEP); pwm_cnt++) {
        uint8_t pwm_index = (pwm_cnt % LED_MATRIX_COLS);
        if (((uint16_t)(pwmp->ct->TC) < ((uint16_t)(led_duty_cycle[pwm_index] + periodticks))) && (led_duty_cycle[pwm_index] > 0)) {
            setPinOutput(led_col_pins[pwm_index]);
#    if (PWM_OUTPUT_ACTIVE_LEVEL == PWM_OUTPUT_ACTIVE_LOW)
            writePinLow(led_col_pins[pwm_index]);
        } else {
            setPinInputHigh(led_col_pins[pwm_index]);
#    elif (PWM_OUTPUT_ACTIVE_LEVEL == PWM_OUTPUT_ACTIVE_HIGH)
            writePinHigh(led_col_pins[pwm_index]);
        } else {
            setPinInputLow(led_col_pins[pwm_index]);
#    endif
        }
    }
#elif ((SN32_PWM_CONTROL == SOFTWARE_PWM) && (DIODE_DIRECTION == ROW2COL))
    for (uint8_t pwm_cnt = 0; pwm_cnt < (LED_MATRIX_ROWS_HW * RGB_MATRIX_HUE_STEP); pwm_cnt++) {
        uint8_t pwm_index = (pwm_cnt % LED_MATRIX_ROWS_HW);
        if (((uint16_t)(pwmp->ct->TC) < ((uint16_t)(led_duty_cycle[pwm_index] + periodticks))) && (led_duty_cycle[pwm_index] > 0)) {
#    if (PWM_OUTPUT_ACTIVE_LEVEL == PWM_OUTPUT_ACTIVE_LOW)
            writePinLow(led_row_pins[pwm_index]);
        } else {
            writePinHigh(led_row_pins[pwm_index]);
#    elif (PWM_OUTPUT_ACTIVE_LEVEL == PWM_OUTPUT_ACTIVE_HIGH)
            writePinHigh(led_row_pins[pwm_index]);
        } else {
            writePinLow(led_row_pins[pwm_index]);
#    endif
        }
    }
#endif
    // Scan the rgb and key matrix
    update_pwm_channels(pwmp);
    chSysLockFromISR();
    // Advance the timer to just before the wrap-around, that will start a new PWM cycle
    pwm_lld_change_counter(pwmp, 0xFFFF);
    // Enable the interrupt
    pwmEnablePeriodicNotificationI(pwmp);
    chSysUnlockFromISR();
}

void SN32F268F_init(void) {
    for (uint8_t x = 0; x < LED_MATRIX_ROWS_HW; x++) {
        setPinOutput(led_row_pins[x]);
        writePinLow(led_row_pins[x]);
    }
    // Determine which PWM channels we need to control
    rgb_ch_ctrl(); //no parameter since PWM channels are hardcoded on SN32F268F
    // initialize matrix state: all keys off
    for (uint8_t i = 0; i < MATRIX_ROWS; i++) {
        shared_matrix[i] = 0;
    }
    pwmStart(&PWMD1, &pwmcfg);
    shared_matrix_rgb_enable();
}

void SN32F268F_flush(void) {
    // memcpy(led_state, led_state_buf, sizeof(RGB) * RGB_MATRIX_LED_COUNT); Not used since two buffers do not fit SN32F268F RAM
}

void SN32F268F_set_color(int index, uint8_t r, uint8_t g, uint8_t b) {
#ifdef UNDERGLOW_RBG
    bool flip_gb = false;
    for (uint8_t led_id = 0; led_id < UNDERGLOW_LEDS; led_id++) {
        if (underglow_leds[led_id] == index) {
            flip_gb = true;
        }
    }
    if (flip_gb) {
        led_state[index].r = r;
        led_state[index].b = g;
        led_state[index].g = b;
    } else {
#endif // UNDERGLOW_RBG
        led_state[index].r = r;
        led_state[index].b = b;
        led_state[index].g = g;
#ifdef UNDERGLOW_RBG
    }
#endif // UNDERGLOW_RBG
}

void SN32F268F_set_color_all(uint8_t r, uint8_t g, uint8_t b) {
    for (int i = 0; i < RGB_MATRIX_LED_COUNT; i++) {
        SN32F268F_set_color(i, r, g, b);
    }
}

bool matrix_scan_custom(matrix_row_t current_matrix[]) {
    if (!matrix_scanned) return false; // Nothing to process until we have the matrix scanned

    bool changed = memcmp(raw_matrix, shared_matrix, sizeof(shared_matrix)) != 0;
    if (changed) memcpy(raw_matrix, shared_matrix, sizeof(shared_matrix));

    matrix_scanned = false;

    return changed;
}