#include "ultrasonic.h"
#include <avr/io.h>
#include <util/delay.h>
#include "../utils/timing.h"

#define SPEED_OF_SOUND 0.0343f

// Pin definitions
#define TRIG_FRONT_BIT  PD5
#define ECHO_FRONT_BIT  PD4
#define TRIG_RIGHT_BIT  PD7
#define ECHO_RIGHT_BIT  PD6
#define TRIG_LEFT_BIT   PC0
#define ECHO_LEFT_BIT   PC1

const UltrasonicSensor SENSOR_FRONT = { &PORTD, &PIND, TRIG_FRONT_BIT, ECHO_FRONT_BIT };
const UltrasonicSensor SENSOR_RIGHT = { &PORTD, &PIND, TRIG_RIGHT_BIT, ECHO_RIGHT_BIT };
const UltrasonicSensor SENSOR_LEFT  = { &PORTC, &PINC, TRIG_LEFT_BIT,  ECHO_LEFT_BIT  };

/**
 * @brief pulseIn replacement using myMicros
 * @param pinReg pointer to PIN register
 * @param bitMask bit position
 * @param timeout_us timeout in microseconds
 * @return pulse width in microseconds (0 if timeout)
 */
static uint32_t myPulseIn(volatile uint8_t* pinReg, uint8_t bitMask, uint32_t timeout_us) {
    uint32_t start = myMicros();

    // 1) wait for any prior pulse to end (echo LOW)
    while (*pinReg & bitMask) {
        if (myMicros() - start > timeout_us) return 0;
    }
    // 2) wait for pulse to begin (echo HIGH)
    while (!(*pinReg & bitMask)) {
        if (myMicros() - start > timeout_us) return 0;
    }
    uint32_t pulseStart = myMicros();
    // 3) wait for pulse to end (echo LOW)
    while (*pinReg & bitMask) {
        if (myMicros() - pulseStart > timeout_us) return 0;
    }
    return myMicros() - pulseStart;
}

float getDistance(const UltrasonicSensor* s) {
    *(s->trigPort) &= ~(1 << s->trigBit);
    _delay_us(2);
    *(s->trigPort) |=  (1 << s->trigBit);
    _delay_us(10);
    *(s->trigPort) &= ~(1 << s->trigBit);

    uint32_t duration = myPulseIn(s->echoPin, (1 << s->echoBit), 30000UL);
    if (duration == 0) return 0.0f;
    return duration * SPEED_OF_SOUND / 2.0f;
}
