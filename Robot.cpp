#include "Robot.h"

Robot* Robot::instance = 0;

Robot::Robot() {
    timer = new EventTimer();
    ir = new SharpIR((uint8_t) A6);
    line = new LineFollowing();
    pid = new PID(ir, line);
    filter = new ComplementaryFilter();

    //TODO: Init() classes that have Inits
    line->Init();
    filter->Init();
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

void Robot::updateSensors() {
    //Read from and update sensor values, such as SharpIR, gyro, accel, and IR receiver
}

bool Robot::runStateMachine() {
    switch(currentState) {
        case STARTUP:
            pid->setSpeedTargets(0, 0);

            //If we detect an IR signal
            if(proxSensors.readBasicFront()) {
                timer->Start(1000);
                incrementState();
            }
            break;
        case WAIT_1S:
            pid->setSpeedTargets(0, 0);
            if(timer->CheckExpired()) {
                incrementState();
            }
            break;
        case WALL_FOLLOW:
            pid->setSpeedTargets(pid->getLeftWallEffort(), pid->getRightWallEffort());

            if(line->Detect() || proxSensors.readBasicFront()) {
                resetEncoderOffset();
                incrementState();
            }
            break;
        case TURN_LEFT_90:
            pid->setSpeedTargets(PIVOT_SPEED, -PIVOT_SPEED);

            if(getDegreesTurned() > 90) {
                //TODO: Make sure we're updating speed targets so that the pivot speeds get overwritten when trying to acquire the line
                if(line->Align(pid->getLeftLineEffort(), pid->getRightLineEffort())) {
                    incrementState();
                }
            }
            break;
        case LINE_FOLLOW:
            pid->calcLinePID();

            pid->setSpeedTargets(pid->getLeftLineEffort(), pid->getRightLineEffort());

            if(proxSensors.readBasicFront()) {
                resetEncoderOffset();
                incrementState();
            }
            break;
        case TURN_RIGHT_90:
            pid->setSpeedTargets(-PIVOT_SPEED, PIVOT_SPEED);

            if(getDegreesTurned() < -90) {
                incrementState();
            }
            break;
        case DRIVE_UP_RAMP:
            if(filter->getCurrentAngle() > 10 && !onRamp) {
                onRamp = true;
            }

            if(onRamp && filter->getCurrentAngle() < 10) {
                resetEncoderOffset();
                incrementState();
            }
            break;
        case SPIN_360:
            pid->setSpeedTargets(PIVOT_SPEED, -PIVOT_SPEED);

            if(getDegreesTurned() > 360) {
                incrementState();
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

void Robot::incrementState() {
    currentState = (StateMachine) ((int) currentState + 1);
}

void Robot::resetEncoderOffset() {
    countsLeftOffset = countsLeft;
    countsRightOffset = countsRight;
}

float Robot::getDegreesTurned() {
    int dLeft = countsLeft - countsLeftOffset; //How much we've turned since the last reset
    int dRight = countsRight - countsRightOffset;

    int avgTurned = abs((-dLeft + dRight)/2); //TODO: Make not abs()

    //Calculate what percentage of the circle we've spun, then multiply by 360 to calculate degrees turned
    float degreesTurned = (PIVOT_CIRCUMFERENCE / (avgTurned * TICKS_TO_CM)) * 360;

    return degreesTurned;
}

Zumo32U4Encoders Robot::getEncoders() {
    return encoders;
}

/*
 * ISR for timing. Basically, raise a flag on overflow. Timer4 is set up to run with a pre-scaler
 * of 1024 and TOP is set to 249. Clock is 16 MHz, so interval is dT = (1024 * 250) / 16 MHz = 16 ms.
 */
ISR(TIMER4_OVF_vect)
{
    Robot *robot = Robot::getRobot();
    Zumo32U4Encoders encoders = robot->getEncoders();

    //Capture a "snapshot" of the encoder counts for later processing
    robot->countsLeft = encoders.getCountsLeft();
    robot->countsRight = encoders.getCountsRight();

    robot->readyToPID = true;
}
