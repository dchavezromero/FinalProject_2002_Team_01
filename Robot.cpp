#include "Robot.h"

Robot* Robot::instance = 0;

Robot::Robot() {
    //Serial.begin(115200);

    timer = new EventTimer();
    ir = new SharpIR((uint8_t) A6);
    line = new LineFollowing();
    pid = new PID(ir, line);
    filter = new ComplementaryFilter();

    //TODO: Init() classes that have Inits
    line->Init(pid);
    filter->Init();

    lcd.clear();
    lcd.print("STARTUP");

    setupEncoderTimer();
}

Robot *Robot::getRobot() {
    if(!instance)
        instance = new Robot();

    return instance;
}

bool Robot::loop() {
    updateSensors();

    ir->getDistance();

    line->update();


    if(readyToSpeedPID) {
        pid->calcSpeedPID(countsLeft, countsRight);

        readyToSpeedPID = false;
    }

    if(readyToWallPID) {
        pid->calcWallPID();
        
        readyToWallPID = false;
    }

    filter->CalcAngle();

    motors.setSpeeds(pid->getLeftSpeedEffort(), pid->getRightSpeedEffort());

    return runStateMachine();
}

void Robot::updateSensors() {
    //Read from and update sensor values, such as SharpIR, gyro, accel, and IR receiver
}

bool Robot::runStateMachine() {
    switch(currentState) {
        case STARTUP:
            //Serial.println(filter->getCurrentAngle());
            //lcd.clear();
            //lcd.print(filter->getCurrentAngle());

            pid->setSpeedTargets(0, 0);

            //If we detect an IR signal
            if(line->detectIR()) {
                timer->Start(1000);
                incrementState();

                lcd.clear();
                lcd.print("WAIT_1S");
            }
            break;
        case WAIT_1S:
            pid->setSpeedTargets(0, 0);
            if(timer->CheckExpired()) {
                lcd.clear();
                lcd.print("WALL_FOLLOW");
                incrementState();
                resetEncoderOffset();
            }
            break;
        case WALL_FOLLOW:
            pid->setSpeedTargets(pid->getLeftWallEffort(), pid->getRightWallEffort());

            /*if(line->detectLine()) {
                resetEncoderOffset();
                incrementState();
            }*/

            //line->detectLine();

            //pid->setSpeedTargets(PIVOT_SPEED, PIVOT_SPEED);
            if(/*getDegreesTurned() > 90*/ false) {
                lcd.clear();
                lcd.print("YAY");
                pid->setSpeedTargets(0, 0);
            }

            break;
        case TURN_LEFT_90:
            pid->setSpeedTargets(PIVOT_SPEED, -PIVOT_SPEED);

            if(getDegreesTurned() > 90) {
                //TODO: Make sure we're updating speed targets so that the pivot speeds get overwritten when trying to acquire the line
                if(line->doAlign(pid->getLeftLineEffort(), pid->getRightLineEffort())) {
                    //incrementState();
                }
            }
            break;
        case LINE_FOLLOW:
            pid->calcLinePID(pid->getLeftLineEffort(), pid->getRightLineEffort(), pid->getCurrentBaseSpeed());

            pid->setSpeedTargets(pid->getLeftLineEffort(), pid->getRightLineEffort());

            if(line->detectIR()) {
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
            pid->setSpeedTargets(PIVOT_SPEED, PIVOT_SPEED);

            if(filter->getCurrentAngle() < -20 && !onRamp) {
                lcd.clear();
                lcd.print("ON RAMP");
                onRamp = true;
            }

            if(onRamp && abs(filter->getCurrentAngle()) < 10) {
                lcd.clear();
                lcd.print("SPINNING");
                resetEncoderOffset();
                incrementState();
            }
            break;
        case SPIN_360:
            pid->setSpeedTargets(PIVOT_SPEED, -PIVOT_SPEED);

            Serial.println(getDegreesTurned());

            if(getDegreesTurned() > 360) {
                pid->setSpeedTargets(0, 0);
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

    float cmTraveled = (avgTurned / TICKS_TO_CM);
    double pctCircle = cmTraveled / PIVOT_CIRCUMFERENCE;
    float degreesTurned = pctCircle * 360;

    return degreesTurned;
}

Zumo32U4Encoders Robot::getEncoders() {
    return encoders;
}

void Robot::setupEncoderTimer() {
    noInterrupts();
    TCCR4A = 0x00; //disable some functionality -- no need to worry about this
    TCCR4B = 0x0C; //sets the prescaler -- look in the handout for values
    TCCR4C = 0x04; //toggles pin 6 at one-half the timer frequency
    TCCR4D = 0x00; //normal mode

    OCR4C = 0X6C;  //TOP goes in OCR4C //I picked 107?
    TIMSK4 = 0x04; //enable overflow interrupt
    interrupts();
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

    robot->readyToSpeedPID = true;
}
