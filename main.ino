#include <Arduino.h>
#include "Robot.h"

Robot *robot;

void setup() {
    

    robot = Robot::getRobot();
}

void loop() {
    robot->loop();
}
