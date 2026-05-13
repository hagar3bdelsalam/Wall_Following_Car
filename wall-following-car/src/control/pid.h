#ifndef PID_H
#define PID_H

#include <stdint.h>

/**
 * @brief PID controller state
 */
typedef struct {
    float Kp;               // Proportional gain
    float Ki;               // Integral gain
    float Kd;               // Derivative gain
    float previous_error;   // Last error for derivative term
    float integral;         // Accumulated integral
} PidController;

/**
 * @brief Initialize PID controller with default gains
 * @param pid pointer to controller structure
 * @param kp proportional gain
 * @param ki integral gain
 * @param kd derivative gain
 */
void pid_init(PidController* pid, float kp, float ki, float kd);

/**
 * @brief Reset PID state (clears integral and previous error)
 * @param pid pointer to controller structure
 */
void pid_reset(PidController* pid);

/**
 * @brief Compute PID correction
 * @param pid pointer to controller structure
 * @param error current error value
 * @return correction value (typically added/subtracted from speed)
 */
float pid_compute(PidController* pid, float error);

/**
 * @brief Clamp value between min and max
 * @param value input value
 * @param min minimum
 * @param max maximum
 * @return clamped value
 */
int pid_clamp_speed(int value, int min, int max);

#endif // PID_H
