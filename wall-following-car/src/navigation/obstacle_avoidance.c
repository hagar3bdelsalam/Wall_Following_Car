#include "obstacle_avoidance.h"
#include "../motors/motor.h"
#include "../communication/uart.h"

static bool deadReverseTrialUsed = false;

void obstacle_avoidance_init(ObstacleAvoidanceConfig* oac) {
    oac->stopDistance        = 30;
    oac->resetDeadDistance   = 45;
    oac->deadDistance        = 15;
    oac->rightWallThreshold  = 40.0f;
    oac->leftWallThreshold   = 40.0f;
    oac->debugEnabled        = true;
    deadReverseTrialUsed     = false;
}

bool handle_front_obstacle(const ObstacleAvoidanceConfig* oac,
                          float distFront, float distLeft, float distRight,
                          const MovementConfig* mc) {
    if (distFront <= 0 || distFront >= oac->stopDistance) {
        return false;  // No obstacle
    }

    float freshRight = distRight;
    float freshLeft  = distLeft;
    float freshFront = distFront;

    bool rightOpen = (freshRight <= 0 || freshRight > oac->rightWallThreshold);
    bool leftOpen  = (freshLeft  <= 0 || freshLeft  > oac->leftWallThreshold);

    bool inDeadRange = (freshFront > 0 && freshFront < oac->deadDistance);

    if (rightOpen) {
        if (oac->debugEnabled) {
            uart_print_str("OBSTACLE: Right is open. Turning RIGHT. ");
            uart_print_str("Front="); uart_print_float(freshFront, 1);
            uart_print_str(" L="); uart_print_float(freshLeft, 1);
            uart_print_str(" R="); uart_print_float(freshRight, 1);
            uart_println("");
        }
        turnRight90(mc);
        return true;
    } else if (leftOpen) {
        if (oac->debugEnabled) {
            uart_print_str("OBSTACLE: Right closed, Left is open. Turning LEFT. ");
            uart_print_str("Front="); uart_print_float(freshFront, 1);
            uart_print_str(" L="); uart_print_float(freshLeft, 1);
            uart_print_str(" R="); uart_print_float(freshRight, 1);
            uart_println("");
        }
        turnLeft90(mc);
        return true;
    } else if (inDeadRange && !deadReverseTrialUsed) {
        if (oac->debugEnabled) {
            uart_print_str("DEAD-END: Both sides closed, distance=");
            uart_print_float(freshFront, 1);
            uart_println(" cm. Reversing...");
        }
        goBackwardMs(mc, mc->backTimeMs);
        deadReverseTrialUsed = true;
        return true;
    } else {
        stopMotors();
        return true;
    }
}

void obstacle_avoidance_update_dead_state(bool frontCloseEnough) {
    if (!frontCloseEnough) {
        deadReverseTrialUsed = false;
    }
}

bool obstacle_avoidance_dead_reverse_used(void) {
    return deadReverseTrialUsed;
}

void obstacle_avoidance_set_dead_reverse_used(bool used) {
    deadReverseTrialUsed = used;
}
