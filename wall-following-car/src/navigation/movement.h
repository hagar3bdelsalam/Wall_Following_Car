#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Movement state and parameters
 */
typedef struct {
    int baseSpeed;
    int turnSpeedRight;
    int turnSpeedLeft;
    int turnTicksRight;
    int turnTicksLeft;
    int backSpeed;
    uint32_t backTimeMs;
    float leftTrim;
    float rightTrim;
    bool debugEnabled;
} MovementConfig;

/**
 * @brief Initialize movement configuration with defaults
 * @param mc pointer to movement config structure
 */
void movement_init(MovementConfig* mc);

/**
 * @brief Move forward for specified duration
 * @param mc pointer to movement config
 * @param seconds duration in seconds
 */
void goForward(const MovementConfig* mc, int seconds);

/**
 * @brief Move backward for specified duration
 * @param mc pointer to movement config
 * @param ms duration in milliseconds
 */
void goBackwardMs(const MovementConfig* mc, uint32_t ms);

/**
 * @brief Right pivot turn (left wheel drives, right wheel dead)
 * Uses encoder ticks to determine rotation
 * @param mc pointer to movement config
 */
void turnRight90(const MovementConfig* mc);

/**
 * @brief Left pivot turn (right wheel drives, left wheel dead)
 * Uses encoder ticks to determine rotation
 * @param mc pointer to movement config
 */
void turnLeft90(const MovementConfig* mc);

#endif // MOVEMENT_H
