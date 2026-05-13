#include "encoders.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include "../utils/timing.h"

#define ENC_RIGHT_BIT   PD2
#define ENC_LEFT_BIT    PD3
#define DEBOUNCE_TIME   500UL  // microseconds

volatile unsigned long leftTicks       = 0;
volatile unsigned long rightTicks      = 0;
volatile unsigned long lastLeftMicros  = 0;
volatile unsigned long lastRightMicros = 0;

ISR(INT0_vect) {  // ENCODER_RIGHT (PD2)
    uint32_t now = myMicros();
    if (now - lastRightMicros > DEBOUNCE_TIME) {
        rightTicks++;
        lastRightMicros = now;
    }
}

ISR(INT1_vect) {  // ENCODER_LEFT (PD3)
    uint32_t now = myMicros();
    if (now - lastLeftMicros > DEBOUNCE_TIME) {
        leftTicks++;
        lastLeftMicros = now;
    }
}

void encoders_init(void) {
    // ISC01:0 = 01 (CHANGE), ISC11:0 = 01 (CHANGE)
    EICRA = (1 << ISC10) | (1 << ISC00);
    EIMSK = (1 << INT0)  | (1 << INT1);
}

volatile unsigned long* encoders_get_right_ticks_ptr(void) {
    return (volatile unsigned long*)&rightTicks;
}

volatile unsigned long* encoders_get_left_ticks_ptr(void) {
    return (volatile unsigned long*)&leftTicks;
}

void encoders_reset_ticks(void) {
    uint8_t sreg = SREG;
    cli();
    leftTicks  = 0;
    rightTicks = 0;
    SREG = sreg;
}
