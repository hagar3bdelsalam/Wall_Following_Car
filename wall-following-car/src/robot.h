#ifndef ROBOT_H
#define ROBOT_H

#include "control/wall_follow.h"
#include "navigation/movement.h"
#include "navigation/obstacle_avoidance.h"
#include "communication/command_parser.h"

/**
 * @brief Initialize the entire robot system
 */
void robot_init(void);

/**
 * @brief Main robot control loop
 * Call repeatedly from loop() function
 */
void robot_loop(void);

#endif // ROBOT_H
