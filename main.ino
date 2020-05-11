#include <Arduino.h>
#include "Robot.h"

Robot *robot;

void setup() {
    Serial.begin(115200);

    robot = Robot::getRobot();
}

void loop() {
    robot->loop();
}
