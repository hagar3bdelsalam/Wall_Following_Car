#ifndef OBSTACLE_AVOIDANCE_H
#define OBSTACLE_AVOIDANCE_H

#include <stdint.h>
#include <stdbool.h>
#include "../navigation/movement.h"

/**
 * @brief Obstacle avoidance configuration
 */
typedef struct {
    int stopDistance;           // Front distance threshold (cm) to trigger avoidance
    int resetDeadDistance;      // Front distance to reset dead-end handling
    int deadDistance;           // Stricter dead-end detection threshold
    float rightWallThreshold;   // Right sensor threshold (cm)
    float leftWallThreshold;    // Left sensor threshold (cm)
    bool debugEnabled;
} ObstacleAvoidanceConfig;

/**
 * @brief Initialize obstacle avoidance configuration
 * @param oac pointer to config
 */
void obstacle_avoidance_init(ObstacleAvoidanceConfig* oac);

/**
 * @brief Handle front obstacle detection and determine turn direction
 * @param oac pointer to config
 * @param distFront front distance (cm)
 * @param distLeft left distance (cm)
 * @param distRight right distance (cm)
 * @param mc movement config for executing turns
 * @return true if turn was executed, false otherwise
 */
bool handle_front_obstacle(const ObstacleAvoidanceConfig* oac, 
                          float distFront, float distLeft, float distRight,
                          const MovementConfig* mc);

/**
 * @brief Track dead-end state
 * Used internally to manage dead-end reverse attempts
 */
void obstacle_avoidance_update_dead_state(bool frontCloseEnough);

/**
 * @brief Check if dead-end reverse has been used
 */
bool obstacle_avoidance_dead_reverse_used(void);

/**
 * @brief Mark dead-end reverse as used
 */
void obstacle_avoidance_set_dead_reverse_used(bool used);

#endif // OBSTACLE_AVOIDANCE_H
