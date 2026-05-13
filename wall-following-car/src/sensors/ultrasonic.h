#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdint.h>

/**
 * @brief Sensor descriptor for ultrasonic measurement
 */
typedef struct {
    volatile uint8_t* trigPort;
    volatile uint8_t* echoPin;
    uint8_t           trigBit;
    uint8_t           echoBit;
} UltrasonicSensor;

/**
 * @brief Sensor instances for front/right/left
 */
extern const UltrasonicSensor SENSOR_FRONT;
extern const UltrasonicSensor SENSOR_RIGHT;
extern const UltrasonicSensor SENSOR_LEFT;

/**
 * @brief Measure distance using ultrasonic sensor
 * @param s pointer to sensor descriptor
 * @return distance in cm (0 if timeout)
 */
float getDistance(const UltrasonicSensor* s);

#endif // ULTRASONIC_H
