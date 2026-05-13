#include "motor.h"
#include "pwm_timer.h"
#include <avr/io.h>

// Motor 1 pin definitions
#define M1_EN_BIT     PB3
#define M1_IN1_BIT    PB4
#define M1_IN2_BIT    PB5

// Motor 2 pin definitions
#define M2_EN_BIT     PB2
#define M2_IN1_BIT    PB1
#define M2_IN2_BIT    PB0

void runMotor(int motorIndex, int speed, bool forward) {
    uint8_t s8 = (speed < 0) ? 0 : (speed > 255 ? 255 : (uint8_t)speed);

    if (motorIndex == 1) {
        if (s8 == 0) {
            pwm_set_M1(0);
            PORTB &= ~((1 << M1_IN1_BIT) | (1 << M1_IN2_BIT));
        } else if (forward) {
            PORTB |=  (1 << M1_IN1_BIT);
            PORTB &= ~(1 << M1_IN2_BIT);
            pwm_set_M1(s8);
        } else {
            PORTB &= ~(1 << M1_IN1_BIT);
            PORTB |=  (1 << M1_IN2_BIT);
            pwm_set_M1(s8);
        }
    } else if (motorIndex == 2) {
        if (s8 == 0) {
            pwm_set_M2(0);
            PORTB &= ~((1 << M2_IN1_BIT) | (1 << M2_IN2_BIT));
        } else if (forward) {
            PORTB |=  (1 << M2_IN1_BIT);
            PORTB &= ~(1 << M2_IN2_BIT);
            pwm_set_M2(s8);
        } else {
            PORTB &= ~(1 << M2_IN1_BIT);
            PORTB |=  (1 << M2_IN2_BIT);
            pwm_set_M2(s8);
        }
    }
}

void stopMotors(void) {
    runMotor(1, 0, true);
    runMotor(2, 0, true);
}
