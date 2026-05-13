#include "robot.h"
#include "system/system_init.h"
#include "utils/timing.h"
#include "motors/pwm_timer.h"
#include "sensors/ultrasonic.h"
#include "sensors/encoders.h"
#include "motors/motor.h"
#include "communication/uart.h"
#include "communication/command_parser.h"
#include <avr/interrupt.h>

// Global configuration structures
static WallFollower wallFollower;
static MovementConfig movementConfig;
static ObstacleAvoidanceConfig obstacleAvoidanceConfig;
static RobotState robotState;

static bool debugEnabled = true;

void robot_init(void) {
    cli();
    
    // Initialize hardware subsystems
    system_init();
    timing_init();
    pwm_timer2_init();
    encoders_init();
    uart_init();
    
    sei();
    
    // Initialize control structures
    wall_follower_init(&wallFollower, 110, 0.35f, 0.0f, 3.5f);
    movement_init(&movementConfig);
    obstacle_avoidance_init(&obstacleAvoidanceConfig);
    robot_state_init(&robotState, &wallFollower, &movementConfig, &obstacleAvoidanceConfig);
    
    // Print startup message
    if (debugEnabled) {
        uart_println(" MASTER CAR SCRIPT READY (LL - MODULAR) ");
        uart_println("'1' = Run Continuous PID | '0' = Stop");
        uart_println("'R' = Run Right Sequence | 'L' = Run Left Sequence");
        uart_println("'F3' = Move Forward 3s | 'B500' = Reverse 500ms");
        uart_println("'V100' = Base speed | 'O30' = Front stop distance (cm)");
        uart_println("'P2.5' = Set Kp | 'D15' = Set Kd");
        uart_println("'?' = Settings");
    }
}

static void warmupSensors(void) {
    if (debugEnabled) uart_println("Warming up sensors...");
    for (int i = 0; i < 20; i++) {
        float r = getDistance(&SENSOR_RIGHT);
        float l = getDistance(&SENSOR_LEFT);
        if (r > 0) wallFollower.lastDistR = r;
        if (l > 0) wallFollower.lastDistL = l;
        myDelayMs(10);
    }
    if (debugEnabled) uart_println("Sensors ready.");
}

void robot_loop(void) {
    // 1. Handle serial commands
    if (uart_available()) {
        char cmd = (char)uart_read_blocking();
        process_command(&robotState, cmd);
    }

    // Stop if not running
    if (!robotState.isRunning) {
        stopMotors();
        myDelayMs(100);
        return;
    }

    // Warmup on first run
    if (!robotState.warmupDone) {
        warmupSensors();
        robotState.warmupDone = true;
    }

    // 2. Read sensors
    float distRight = getDistance(&SENSOR_RIGHT);
    float distLeft  = getDistance(&SENSOR_LEFT);
    float distFront = getDistance(&SENSOR_FRONT);

    // 3. Update dead-end state
    bool frontCloseEnough = (distFront > 0 && distFront < obstacleAvoidanceConfig.resetDeadDistance);
    obstacle_avoidance_update_dead_state(frontCloseEnough);

    // 4. Handle front obstacle avoidance
    if (distFront > 0 && distFront < obstacleAvoidanceConfig.stopDistance) {
        handle_front_obstacle(&obstacleAvoidanceConfig, distFront, distLeft, distRight,
                            &movementConfig);
        wall_follower_reset(&wallFollower);
    } else {
        // 5. Wall following PID control
        int leftMotorSpeed, rightMotorSpeed;
        float error = wall_follower_compute(&wallFollower, distLeft, distRight,
                                           &leftMotorSpeed, &rightMotorSpeed);

        // Debug output
        if (debugEnabled) {
            uart_print_str("F: ");        uart_print_float(distFront, 2);
            uart_print_str(" | L: ");     uart_print_float(distLeft,  2);
            uart_print_str(" | R: ");     uart_print_float(distRight, 2);
            uart_print_str(" | Err: ");   uart_print_float(error,     2);
            uart_print_str(" | L_PWM: "); uart_print_long(leftMotorSpeed);
            uart_print_str(" | R_PWM: "); uart_print_long(rightMotorSpeed);
            uart_println("");
        }

        // 6. Drive motors
        runMotor(1, leftMotorSpeed,  true);
        runMotor(2, rightMotorSpeed, true);
    }

    myDelayMs(50);
}
