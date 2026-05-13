#include "pid.h"

void pid_init(PidController* pid, float kp, float ki, float kd) {
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->previous_error = 0.0f;
    pid->integral = 0.0f;
}

void pid_reset(PidController* pid) {
    pid->previous_error = 0.0f;
    pid->integral = 0.0f;
}

float pid_compute(PidController* pid, float error) {
    pid->integral += error;
    if (pid->integral >  100.0f) pid->integral =  100.0f;
    if (pid->integral < -100.0f) pid->integral = -100.0f;
    
    float derivative = error - pid->previous_error;
    float correction = (pid->Kp * error) + (pid->Ki * pid->integral) + (pid->Kd * derivative);
    pid->previous_error = error;
    
    return correction;
}

int pid_clamp_speed(int value, int min, int max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}
