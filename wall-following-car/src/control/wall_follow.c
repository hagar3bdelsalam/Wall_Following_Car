#include "wall_follow.h"
#include "pid.h"

#define MAX_SPEED 200
#define MIN_SPEED 0

void wall_follower_init(WallFollower* wf, int baseSpeed, float kp, float ki, float kd) {
    pid_init(&wf->pid, kp, ki, kd);
    wf->baseSpeed = baseSpeed;
    wf->leftTrim = 0.9f;
    wf->rightTrim = 1.0f;
    wf->lastDistR = -1.0f;
    wf->lastDistL = -1.0f;
}

void wall_follower_reset(WallFollower* wf) {
    pid_reset(&wf->pid);
    wf->lastDistR = -1.0f;
    wf->lastDistL = -1.0f;
}

float wall_follower_compute(WallFollower* wf, float distLeft, float distRight,
                           int* outLeftSpeed, int* outRightSpeed) {
    // Update cached distances
    if (distLeft > 0) wf->lastDistL = distLeft;
    if (distRight > 0) wf->lastDistR = distRight;

    // Calculate error for centering
    float error = 0.0f;
    bool leftValid  = (distLeft > 0 && distLeft < 45.0f);
    bool rightValid = (distRight > 0 && distRight < 45.0f);

    if (leftValid && rightValid) {
        error = (distLeft - distRight) / 2.0f;
    } else if (rightValid) {
        error = (TARGET_CENTER - distRight);
    } else if (leftValid) {
        error = -(TARGET_CENTER - distLeft);
    }

    // PID control
    float correction = pid_compute(&wf->pid, error);

    int leftMotorSpeed  = (int)(wf->baseSpeed * wf->leftTrim)  - (int)correction;
    int rightMotorSpeed = (int)(wf->baseSpeed * wf->rightTrim) + (int)correction;

    leftMotorSpeed  = pid_clamp_speed(leftMotorSpeed,  MIN_SPEED, MAX_SPEED);
    rightMotorSpeed = pid_clamp_speed(rightMotorSpeed, MIN_SPEED, MAX_SPEED);

    *outLeftSpeed = leftMotorSpeed;
    *outRightSpeed = rightMotorSpeed;

    return error;
}
