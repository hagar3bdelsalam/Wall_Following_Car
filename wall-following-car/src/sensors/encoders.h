#ifndef ENCODERS_H
#define ENCODERS_H

#include <stdint.h>

/**
 * @brief Initialize encoder interrupts (INT0 for right, INT1 for left)
 * Detects on CHANGE, debounces with 500 us hysteresis
 */
void encoders_init(void);

/**
 * @brief Get tick count for right encoder
 * @return number of ticks since last reset
 */
volatile unsigned long* encoders_get_right_ticks_ptr(void);

/**
 * @brief Get tick count for left encoder
 * @return number of ticks since last reset
 */
volatile unsigned long* encoders_get_left_ticks_ptr(void);

/**
 * @brief Reset both encoder tick counters
 */
void encoders_reset_ticks(void);

#endif // ENCODERS_H
