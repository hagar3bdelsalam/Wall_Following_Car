#ifndef PWM_TIMER_H
#define PWM_TIMER_H

#include <stdint.h>

/**
 * @brief Initialize Timer2 for Motor 1 PWM (OC2A on PB3)
 * Mode: 8-bit Fast PWM, Prescaler: 64
 */
void pwm_timer2_init(void);

/**
 * @brief Set PWM duty cycle for Motor 1
 * @param duty 0-255 (0% to 100%)
 */
void pwm_set_M1(uint8_t duty);

/**
 * @brief Set PWM duty cycle for Motor 2 (already initialized by timing_init)
 * @param duty 0-255 (0% to 100%)
 */
void pwm_set_M2(uint8_t duty);

#endif // PWM_TIMER_H
