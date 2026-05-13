#ifndef TIMING_H
#define TIMING_H

#include <stdint.h>

/**
 * @brief Initialize the timing subsystem (Timer1 for microsecond tracking)
 */
void timing_init(void);

/**
 * @brief Return current time in microseconds
 * Timer1 overflows every 1024 us (with 64 prescaler on 16 MHz clock)
 * @return microseconds since startup
 */
uint32_t myMicros(void);

/**
 * @brief Delay in milliseconds (busy-wait via _delay_ms)
 * @param ms milliseconds to delay
 */
void myDelayMs(uint32_t ms);

#endif // TIMING_H
