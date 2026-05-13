#ifndef COMMAND_PARSER_H
#define COMMAND_PARSER_H

#include <stdint.h>
#include <stdbool.h>
#include "../control/wall_follow.h"
#include "../navigation/movement.h"
#include "../navigation/obstacle_avoidance.h"

/**
 * @brief Global robot state for command handling
 */
typedef struct {
    bool isRunning;
    bool warmupDone;
    uint8_t turnCount;
    char turnSequence[64];
    
    WallFollower* wallFollower;
    MovementConfig* movement;
    ObstacleAvoidanceConfig* obstacleAvoidance;
} RobotState;

/**
 * @brief Initialize robot state
 * @param state pointer to robot state structure
 * @param wf pointer to wall follower config
 * @param mc pointer to movement config
 * @param oac pointer to obstacle avoidance config
 */
void robot_state_init(RobotState* state, WallFollower* wf, MovementConfig* mc,
                      ObstacleAvoidanceConfig* oac);

/**
 * @brief Process a serial command character
 * @param state pointer to robot state
 * @param cmd command character
 * @return true if command was processed, false otherwise
 */
bool process_command(RobotState* state, char cmd);

/**
 * @brief Record a turn in the sequence log
 * @param state pointer to robot state
 * @param dir 'L' for left, 'R' for right
 */
void record_turn(RobotState* state, char dir);

/**
 * @brief Print dead-end report (always prints, DEBUG independent)
 * @param state pointer to robot state
 */
void print_dead_end_report(const RobotState* state);

/**
 * @brief Print current settings/parameters
 * @param state pointer to robot state
 */
void print_settings(const RobotState* state);

#endif // COMMAND_PARSER_H
