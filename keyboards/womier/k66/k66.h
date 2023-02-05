// Copyright 2021 https://github.com/fightforlife
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

#include "quantum.h"
#include <stdint.h>
#include <stdbool.h>

void init(void);
void set_color(int index, uint8_t r, uint8_t g, uint8_t b);
void set_color_all(uint8_t r, uint8_t g, uint8_t b);
void matrix_scan_keys(matrix_row_t raw_matrix[], uint8_t current);
void init_underglow(uint8_t devid);
void set_i2c_color_direct(uint8_t index, uint8_t r, uint8_t g, uint8_t b);