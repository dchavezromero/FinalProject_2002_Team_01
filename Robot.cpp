#include "Robot.h"

Robot* Robot::instance = 0;

Robot::Robot() {
    timer = new EventTimer();
    ir = new SharpIR();
    speed = new SpeedControl(); //TODO: remove once PID is fully implemented
    pid = new PID();

    //TODO: Init() classes that have Inits
}

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
            pid->setSpeedTargets(0, 0);

            //If we detect an IR signal
            if(proxSensors.readBasicFront()) {
                timer.Start(1000);
                currentState++;
            }
            break;
        case WAIT_1S:
            pid->setSpeedTargets(0, 0);
            if(timer->CheckExpired()) {
                currentState++;
            }
            break;
        case WALL_FOLLOW:
            pid->setSpeedTargets(ir->getLeftEffort(), ir->getRightEffort());

            if(line.Detect() || proxSensors.readBasicFront()) {
                resetEncoderOffset();
                currentState++;
            }
            break;
        case TURN_LEFT_90:
            pid->setSpeedTargets(PIVOT_SPEED, -PIVOT_SPEED);

            if(getDegreesTurned() > 90) {
                //TODO: Make sure we're updating speed targets so that the pivot speeds get overwritten when trying to acquire the line
                if(line.Align(pid->getLineLeftEffort(), pid->getLineRightEffort())) {
                    currentState++;
                }
            }
            break;
        case LINE_FOLLOW:
            pid->linePID();

            speed->setTargetSpeeds(pid->getLineLeftEffort(), pid->getLineRightEffort());

            if(proxSensors.readBasicFront()) {
                resetEncoderOffset();
                currentState++;
            }
            break;
        case TURN_RIGHT_90:
            pid->setSpeedTargets(-PIVOT_SPEED, PIVOT_SPEED);

            if(getDegreesTurned() < -90) {
                currentState++;
            }
            break;
        case DRIVE_UP_RAMP:
            if(filter->getCurrentAngle())
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

void Robot::resetEncoderOffset() {
    countsLeftOffset = countsLeft;
    countsRightOffset = countsRight;
}

float Robot::getDegreesTurned() {
    int dLeft = currentLeft - countsLeftOffset; //How much we've turned since the last reset
    int dRight = currentRight - countsRightOffset;

    int avgTurned = abs((-dLeft + dRight)/2);

    //Calculate what percentage of the circle we've spun, then multiply by 360 to calculate degrees turned
    float degreesTurned = (PIVOT_CIRCUMFERENCE / (avgTurned * TICKS_TO_CM)) * 360;

    return degreesTurned;
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
