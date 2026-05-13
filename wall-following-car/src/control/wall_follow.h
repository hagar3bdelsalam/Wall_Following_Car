#ifndef WALL_FOLLOW_H
#define WALL_FOLLOW_H

#include <stdint.h>
#include <stdbool.h>
#include "pid.h"

#define TARGET_CENTER        15.0f
#define LEFT_WALL_THRESHOLD  40.0f
#define RIGHT_WALL_THRESHOLD 40.0f

/**
 * @brief Wall following controller state
 */
typedef struct {
    PidController pid;
    float lastDistR;
    float lastDistL;
    int   baseSpeed;
    float leftTrim;
    float rightTrim;
} WallFollower;

/**
 * @brief Initialize wall follower with default parameters
 * @param wf pointer to wall follower structure
 * @param baseSpeed base PWM speed (0-255)
 * @param kp PID proportional gain
 * @param ki PID integral gain
 * @param kd PID derivative gain
 */
void wall_follower_init(WallFollower* wf, int baseSpeed, float kp, float ki, float kd);

/**
 * @brief Reset wall follower state
 * @param wf pointer to wall follower structure
 */
void wall_follower_reset(WallFollower* wf);

/**
 * @brief Compute motor speeds to maintain center between walls
 * @param wf pointer to wall follower structure
 * @param distLeft distance to left wall (cm)
 * @param distRight distance to right wall (cm)
 * @param outLeftSpeed pointer to store left motor speed
 * @param outRightSpeed pointer to store right motor speed
 * @return error value used for control
 */
float wall_follower_compute(WallFollower* wf, float distLeft, float distRight,
                           int* outLeftSpeed, int* outRightSpeed);

#endif // WALL_FOLLOW_H
