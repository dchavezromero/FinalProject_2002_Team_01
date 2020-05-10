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
            speed->setTargetSpeeds(0, 0);

            //If we detect an IR signal
            if(proxSensors.readBasicFront()) {
                timer.Start(1000);
                currentState++;
            }
            break;
        case WAIT_1S:
            speed->setTargetSpeeds(0, 0);
            if(timer->CheckExpired()) {
                currentState++;
            }
            break;
        case WALL_FOLLOW:
            speed->setTargetSpeeds(ir->getLeftEffort(), ir->getRightEffort());

            if(line.Detect() || proxSensors.readBasicFront()) {
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

/*
 * ISR for timing. Basically, raise a flag on overflow. Timer4 is set up to run with a pre-scaler
 * of 1024 and TOP is set to 249. Clock is 16 MHz, so interval is dT = (1024 * 250) / 16 MHz = 16 ms.
 */
ISR(TIMER4_OVF_vect)
{
    Robot *robot = Robot::getInstance();

    //Capture a "snapshot" of the encoder counts for later processing
    robot->countsLeft = encoders.getCountsLeft();
    robot->countsRight = encoders.getCountsRight();

    robot->readyToPID = true;
}
