// Copyright 2021 https://github.com/fightforlife
// SPDX-License-Identifier: GPL-2.0-or-later

#include QMK_KEYBOARD_H

// Each layer gets a name for readability, which is then used in the keymap matrix below.
// The underscores don't mean anything - you can have a layer called STUFF or any other name.
// Layer names don't all need to be of the same length, obviously, and you can also skip them
// entirely and just use numbers.
enum layer_names {
    _BASE,
    _FN,
};

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
    /*  COL:       0         1         2         3         4         5         6        7        8         9         10        11        12        13      */
    [_BASE] = { {  KC_ESC,   KC_1,     KC_2,     KC_3,     KC_4,     KC_5,     KC_6,    KC_7,    KC_8,     KC_9,     KC_0,     KC_MINS,  KC_EQL,   KC_BSPC  },
                {  KC_TAB,   KC_Q,     KC_W,     KC_E,     KC_R,     KC_T,     KC_Y,    KC_U,    KC_I,     KC_O,     KC_P,     KC_LBRC,  KC_RBRC,  KC_BSLS  },
                {  KC_CAPS,  KC_A,     KC_S,     KC_D,     KC_F,     KC_G,     KC_H,    KC_J,    KC_K,     KC_L,     KC_SCLN,  KC_QUOT,  KC_ENT,   KC_DEL   },
                {  KC_LSFT,  KC_Z,     KC_X,     KC_C,     KC_V,     KC_B,     KC_N,    KC_M,    KC_COMM,  KC_DOT,   KC_SLSH,  KC_RSFT,  KC_UP,    KC_GRV   },
                {  KC_LCTL,  KC_LGUI,  KC_LALT,  KC_NO,    KC_NO,    KC_SPC,   KC_NO,   KC_NO,   KC_RALT,  MO(_FN),  KC_RCTL,  KC_LEFT,  KC_DOWN,  KC_RIGHT }
              },
    [_FN]   = { {  KC_SLEP,  KC_F1,    KC_F2,    KC_F3,    KC_F4,    KC_F5,    KC_F6,   KC_F7,   KC_F8,    KC_F9,    KC_F10,   KC_F11,   KC_F12,   KC_TRNS  },
                {  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS, KC_TRNS, KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS  },
                {  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS, KC_TRNS, RGB_VAI,  RGB_SPI,  RGB_HUI,  RGB_SAI,  KC_TRNS,  KC_PGUP  },
                {  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS, RGB_VAD, RGB_SPD,  RGB_HUD,  RGB_SAD,  KC_TRNS,  KC_VOLU,  KC_PGDN  },
                {  RGB_MOD,  KC_TRNS,  KC_TRNS,  KC_NO,    KC_NO,    RGB_TOG,  KC_NO,   KC_NO,   RESET,    KC_TRNS,  KC_MPLY,  KC_MPRV,  KC_VOLD,  KC_MNXT  }
             }
};



void rgb_matrix_indicators_advanced_user(uint8_t led_min, uint8_t led_max) {
    // show Capslock
    if (IS_HOST_LED_ON(USB_LED_CAPS_LOCK)) {
        rgb_matrix_set_color(28, 255,255,255);
    }

    //show special keys in second layer
    if (get_highest_layer(layer_state) > 0 ) {
        uint8_t layer = get_highest_layer(layer_state);

        for (uint8_t row = 0; row < MATRIX_ROWS; ++row) {
            for (uint8_t col = 0; col < MATRIX_COLS; ++col) {
                uint8_t index = g_led_config.matrix_co[row][col];

                if (index >= led_min && index <= led_max && index != NO_LED && keymap_key_to_keycode(layer, (keypos_t){col,row}) > KC_TRNS) {
                    rgb_matrix_set_color(index, RGB_WHITE);
                }
            }
        }
    }
}
