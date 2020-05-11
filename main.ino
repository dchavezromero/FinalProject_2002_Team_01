#include <Arduino.h>
#include "Robot.h"

Robot *robot;

void setup() {
    Serial.begin(115200);
    while(!Serial) {}

    Serial.println("I'm going crazy");

    robot = Robot::getRobot();

    Serial.println("After constructor");
}

void loop() {
    robot->loop();
}
