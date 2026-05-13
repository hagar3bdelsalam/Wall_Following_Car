#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include "src/robot.h"

void setup() {
    robot_init();
}

void loop() {
    robot_loop();
}
