#ifndef SYSTEM_INIT_H
#define SYSTEM_INIT_H

/**
 * @brief Initialize all system hardware (GPIO, timers, interrupts, UART)
 * Must be called first in setup()
 */
void system_init(void);

#endif // SYSTEM_INIT_H
