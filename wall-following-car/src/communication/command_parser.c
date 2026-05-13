#include "command_parser.h"
#include "uart.h"
#include "../utils/timing.h"
#include <avr/interrupt.h>

void robot_state_init(RobotState* state, WallFollower* wf, MovementConfig* mc,
                      ObstacleAvoidanceConfig* oac) {
    state->isRunning = false;
    state->warmupDone = false;
    state->turnCount = 0;
    state->wallFollower = wf;
    state->movement = mc;
    state->obstacleAvoidance = oac;
}

bool process_command(RobotState* state, char cmd) {
    if (cmd == '1' || cmd == 'S' || cmd == 's') {
        state->isRunning = true;
        state->warmupDone = false;
        state->turnCount = 0;
        wall_follower_reset(state->wallFollower);
        encoders_reset_ticks();
        obstacle_avoidance_set_dead_reverse_used(false);
        uart_println(">>> STARTED");
        return true;
    }
    else if (cmd == '0' || cmd == 'X' || cmd == 'x') {
        state->isRunning = false;
        stopMotors();
        uart_println(">>> STOPPED");
        return true;
    }
    else if (cmd == 'R') {
        turnRight90(state->movement);
        record_turn(state, 'R');
        return true;
    }
    else if (cmd == 'L') {
        turnLeft90(state->movement);
        record_turn(state, 'L');
        return true;
    }
    else if (cmd == 'F') {
        int seconds = (int)uart_parse_long();
        goForward(state->movement, seconds);
        return true;
    }
    else if (cmd == 'B') {
        uint32_t ms = (uint32_t)uart_parse_long();
        goBackwardMs(state->movement, ms);
        return true;
    }
    else if (cmd == 'V') {
        state->movement->baseSpeed = (int)uart_parse_long();
        uart_print_str("Base speed set to ");
        uart_print_long(state->movement->baseSpeed);
        uart_println("");
        return true;
    }
    else if (cmd == 'P') {
        state->wallFollower->pid.Kp = uart_parse_float();
        uart_print_str("Kp set to ");
        uart_print_float(state->wallFollower->pid.Kp, 2);
        uart_println("");
        return true;
    }
    else if (cmd == 'D') {
        state->wallFollower->pid.Kd = uart_parse_float();
        uart_print_str("Kd set to ");
        uart_print_float(state->wallFollower->pid.Kd, 2);
        uart_println("");
        return true;
    }
    else if (cmd == 'O') {
        state->obstacleAvoidance->stopDistance = (int)uart_parse_long();
        uart_print_str("Front stop distance set to ");
        uart_print_long(state->obstacleAvoidance->stopDistance);
        uart_println(" cm");
        return true;
    }
    else if (cmd == '?') {
        print_settings(state);
        return true;
    }
    return false;
}

void record_turn(RobotState* state, char dir) {
    if (state->turnCount < 64) {
        state->turnSequence[state->turnCount++] = dir;
    }
}

void print_dead_end_report(const RobotState* state) {
    uart_print_str("Turns: ");
    uart_print_ulong(state->turnCount);
    uart_print_str("\r\nSequence: ");
    for (uint8_t i = 0; i < state->turnCount; i++) {
        if (i) uart_print_str(", ");
        uart_write((uint8_t)state->turnSequence[i]);
    }
    uart_print_str("\r\n");
}

void print_settings(const RobotState* state) {
    uart_println("\n=== ROBOT SETTINGS ===");
    uart_print_str("Base Speed: ");
    uart_print_long(state->movement->baseSpeed);
    uart_println("");
    
    uart_print_str("Kp: ");
    uart_print_float(state->wallFollower->pid.Kp, 2);
    uart_println("");
    
    uart_print_str("Kd: ");
    uart_print_float(state->wallFollower->pid.Kd, 2);
    uart_println("");
    
    uart_print_str("Stop Distance: ");
    uart_print_long(state->obstacleAvoidance->stopDistance);
    uart_println(" cm");
    
    uart_print_str("Turn Ticks (R): ");
    uart_print_long(state->movement->turnTicksRight);
    uart_print_str(", (L): ");
    uart_print_long(state->movement->turnTicksLeft);
    uart_println("");
}
