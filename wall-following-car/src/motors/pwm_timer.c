#include "pwm_timer.h"
#include <avr/io.h>

void pwm_timer2_init(void) {
    // Timer2: 8-bit Fast PWM (mode 3), prescaler 64
    // OC2A drives M1_EN (PB3)
    TCCR2A = (1 << COM2A1) | (1 << WGM21) | (1 << WGM20);
    TCCR2B = (1 << CS22);
    OCR2A  = 0;
}

void pwm_set_M1(uint8_t duty) {
    OCR2A = duty;
}

void pwm_set_M2(uint8_t duty) {
    OCR1B = duty;
}
