#include "timing.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// Timer1 free-running counter for microsecond tracking
// Ticks at 4 us, overflow every 1024 us with 64 prescaler on 16 MHz
volatile uint32_t timer1_ovf_count = 0;

ISR(TIMER1_OVF_vect) {
    timer1_ovf_count++;
}

void timing_init(void) {
    // Timer1: 8-bit Fast PWM (mode 5), prescaler 64
    // OC1B can be used for PWM (M2_EN)
    TCCR1A = (1 << COM1B1) | (1 << WGM10);
    TCCR1B = (1 << WGM12)  | (1 << CS11) | (1 << CS10);
    OCR1B  = 0;
    TIMSK1 = (1 << TOIE1);  // Enable Timer1 overflow interrupt
}

uint32_t myMicros(void) {
    uint32_t ovf;
    uint8_t  cnt;
    uint8_t  sreg = SREG;
    cli();
    ovf = timer1_ovf_count;
    cnt = TCNT1L;
    if ((TIFR1 & (1 << TOV1)) && (cnt < 255)) {
        ovf++;
    }
    SREG = sreg;
    return ((ovf << 8) + cnt) * 4UL;
}

void myDelayMs(uint32_t ms) {
    while (ms--) _delay_ms(1);
}
