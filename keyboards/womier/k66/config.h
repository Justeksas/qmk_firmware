// Copyright 2021 https://github.com/fightforlife
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

#include "config_common.h"


/* USB Device descriptor parameter HID\VID
*/
#define VENDOR_ID       0xFEED
#define PRODUCT_ID      0x7698
#define DEVICE_VER      0x0001

#define MANUFACTURER    Womier
#define PRODUCT         K66
#define DESCRIPTION     K66 v1 Keyboard

#define MATRIX_ROWS 5
#define MATRIX_COLS 14
#define DIODE_DIRECTION ROW2COL
#define MATRIX_ROW_PINS {B1, B2, B3, B4, B5}
#define MATRIX_COL_PINS {D1, D2, D3, D4, D5, D6, D7, D8, C3, C4, C5, C6, C7, C8}

//Features
#define USB_POLLING_INTERVAL_MS 1           /* 1000 Hz */
#define FORCE_NKRO                          /* default NRKO enabled */
#define DEBOUNCE 3                          /* Debounce time in ms */
#define MATRIX_KEY_SAMPLE_DELAY 0           /* introduced with 100 there: https://github.com/SonixQMK/qmk_firmware/issues/157,
#undef LOCKING_SUPPORT_ENABLE
#undef LOCKING_RESYNC_ENABLE                                                not needed on Womier K66, faster scanning */
#define LAYER_STATE_8BIT

// RGB Matrix Effects
#define ENABLE_RGB_MATRIX_ALPHAS_MODS
#define ENABLE_RGB_MATRIX_GRADIENT_UP_DOWN
#define ENABLE_RGB_MATRIX_GRADIENT_LEFT_RIGHT
#define ENABLE_RGB_MATRIX_BREATHING
#define ENABLE_RGB_MATRIX_BAND_SAT
#define ENABLE_RGB_MATRIX_BAND_VAL
#define ENABLE_RGB_MATRIX_BAND_PINWHEEL_SAT
#define ENABLE_RGB_MATRIX_BAND_PINWHEEL_VAL
#define ENABLE_RGB_MATRIX_BAND_SPIRAL_SAT
#define ENABLE_RGB_MATRIX_BAND_SPIRAL_VAL
#define ENABLE_RGB_MATRIX_CYCLE_ALL
#define ENABLE_RGB_MATRIX_CYCLE_LEFT_RIGHT
#define ENABLE_RGB_MATRIX_CYCLE_UP_DOWN
#define ENABLE_RGB_MATRIX_RAINBOW_MOVING_CHEVRON
#define ENABLE_RGB_MATRIX_CYCLE_OUT_IN
#define ENABLE_RGB_MATRIX_CYCLE_OUT_IN_DUAL
#define ENABLE_RGB_MATRIX_CYCLE_PINWHEEL
#define ENABLE_RGB_MATRIX_CYCLE_SPIRAL
#define ENABLE_RGB_MATRIX_DUAL_BEACON
#define ENABLE_RGB_MATRIX_RAINBOW_BEACON
#define ENABLE_RGB_MATRIX_RAINBOW_PINWHEELS
//#define ENABLE_RGB_MATRIX_RAINDROPS  //Space Problems
//#define ENABLE_RGB_MATRIX_JELLYBEAN_RAINDROPS //Space Problems
#define ENABLE_RGB_MATRIX_HUE_BREATHING
#define ENABLE_RGB_MATRIX_HUE_PENDULUM
#define ENABLE_RGB_MATRIX_HUE_WAVE
#define ENABLE_RGB_MATRIX_PIXEL_RAIN
#define ENABLE_RGB_MATRIX_PIXEL_FRACTAL

//Framebuffer Effects  Space problems
//#define RGB_MATRIX_FRAMEBUFFER_EFFECTS
//#define ENABLE_RGB_MATRIX_TYPING_HEATMAP
//#define ENABLE_RGB_MATRIX_DIGITAL_RAIN

//keypress Effects
#define RGB_MATRIX_KEYPRESSES
#define ENABLE_RGB_MATRIX_SOLID_REACTIVE_SIMPLE
#define ENABLE_RGB_MATRIX_SOLID_REACTIVE
#define ENABLE_RGB_MATRIX_SOLID_REACTIVE_WIDE
#define ENABLE_RGB_MATRIX_SOLID_REACTIVE_MULTIWIDE
#define ENABLE_RGB_MATRIX_SOLID_REACTIVE_CROSS
#define ENABLE_RGB_MATRIX_SOLID_REACTIVE_MULTICROSS
#define ENABLE_RGB_MATRIX_SOLID_REACTIVE_NEXUS
#define ENABLE_RGB_MATRIX_SOLID_REACTIVE_MULTINEXUS
#define ENABLE_RGB_MATRIX_SPLASH
#define ENABLE_RGB_MATRIX_MULTISPLASH
#define ENABLE_RGB_MATRIX_SOLID_SPLASH
#define ENABLE_RGB_MATRIX_SOLID_MULTISPLASH

//RGB Settings
#define RGB_DISABLE_WHEN_USB_SUSPENDED /* rgb off when power off / suspend */
#define RGB_MATRIX_LED_PROCESS_LIMIT 1 // 1 for temporary fix for flickering LED indicators (like Capslock or layer), tested.
#define RGB_MATRIX_LED_FLUSH_LIMIT 16 //16ms is 60HZ, not needed since we are not flusing at all.

#include "config_led.h"
