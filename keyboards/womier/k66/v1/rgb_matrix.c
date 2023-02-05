#include "rgb_matrix.h"
#include "drivers/rgb_matrix_sn32f268f.c"
#include "drivers/rgb_matrix_sled1734x.c"


void init(void) {
    SN32F268F_init();
    SLED1734X_init(UNDERGLOW_I2C_ADR);
}

void set_color(int index, uint8_t r, uint8_t g, uint8_t b) {
     //if key matrix LEDs
     if(index < MATRIX_LED_TOTAL) 
     {     
        SN32F268F_set_color(index, r, g, b);
     }
     //if underglow LEDs
     else if (index < RGB_MATRIX_LED_COUNT)
     {
        SLED1734X_set_color(index, r, g, b);
     }
}

void set_color_all(uint8_t r, uint8_t g, uint8_t b) {
    for (int i=0; i<RGB_MATRIX_LED_COUNT; i++) {
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
