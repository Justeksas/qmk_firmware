#pragma once

#include <stdint.h>
#include <stdbool.h>

void SN32F268F_init(void);
void SN32F268F_flush(void);
void SN32F268F_set_color(int index, uint8_t r, uint8_t g, uint8_t b);
void SN32F268F_set_color_all(uint8_t r, uint8_t g, uint8_t b);