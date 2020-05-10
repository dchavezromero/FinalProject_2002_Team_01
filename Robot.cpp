#include "Robot.h"

Robot* Robot::instance = 0;

Robot::Robot() {}

Robot *Robot::getRobot() {
    if(!instance)
        instance = new Robot();

    return instance;
}

bool Robot::loop() {
    updateSensors();

    return runStateMachine();
}

void updateSensors() {
    //Read from and update sensor values, such as SharpIR, gyro, accel, and IR receiver
}

bool runStateMachine() {
    switch(currentState) {
        case STARTUP:
            if(/*C button pressed*/) {
                currentState++;
            }
            break;
        case WALL_FOLLOW:
            if(/*Line detected*/) {
                currentState++;
            }
            break;
        case TURN_LEFT_90:
            if(/*Done turning*/) {
                currentState++;
            }
            break;
        case LINE_FOLLOW:
            if(/*Received IR signal*/) {
                currentState++;
            }
            break;
        case TURN_LEFT_90_2:
            //Same as TURN_LEFT_90
            if(/*Done turning*/) {
                currentState++;
            }
            break;
        case DRIVE_UP_RAMP:
            if(/*Sitting on flat ground after driving up ramp*/) {
                currentState++;
            }
            break;
        case SPIN_360:
            if(/*Spun 360 degrees*/) {
                currentState++;
            }
            break;
        case STOP:
            return true;
            break;
        default:
            break;
    }

    return false;
}
