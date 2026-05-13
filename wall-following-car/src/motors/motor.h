#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include <stdbool.h>

#define MAX_SPEED   200
#define MIN_SPEED   0

/**
 * @brief Run a motor at specified speed and direction
 * @param motorIndex 1 for motor 1 (left), 2 for motor 2 (right)
 * @param speed PWM speed (0-255), clamped internally
 * @param forward true for forward, false for backward
 */
void runMotor(int motorIndex, int speed, bool forward);

/**
 * @brief Stop all motors
 */
void stopMotors(void);

#endif // MOTOR_H
