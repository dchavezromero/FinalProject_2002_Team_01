#include "Robot.h"

//Stores the single instance of the robot
Robot* Robot::instance = 0;

/*
 * Constructor for the robot. The Robot class is the only class who can call this, as the
 * constructor is private. This is so that only one instance of the robot can be created, and so that
 * anybody can get an instance of the robot through the getRobot() method.
*/
Robot::Robot() {
    //Initialize all of the different tools we will need to traverse the course.

    //The timer is used to delay between states, so that the robot does not start
    //the moment the start button is pressed.
    timer = new EventTimer();

    //Create the IR and line sensor classes
    ir = new SharpIR((uint8_t) A6);
    line = new LineFollowing();

    //Create the PID manager, and give it pointers to the IR and line sensors
    pid = new PID(ir, line);

    //Create the filter to determine pitch angle
    filter = new ComplementaryFilter();

    //Initialize the line follower and filter
    line->Init(pid);
    filter->Init();

    lcd.clear();
    lcd.print("STARTUP");

    //Start the timer that fires interrupts to read encoder values.
    setupEncoderTimer();
}


/*
 * Singleton design pattern. This getRobot() method is public, and will return
 * a pointer to the only instance of the robot. This is so the encoder ISR can
 * set values within the robot class, instead of setting global values.
*/
Robot *Robot::getRobot() {
    if(!instance)
        instance = new Robot();

    return instance;
}


/*
 * The robot loop. Call this repeatedly to have the robot update sensors, PIDs,
 * and the state machine.
*/
bool Robot::loop() {
    //Make sure that the event timer is updated within the getDistance method. Will
    //update the readyToWallPID flag, so it is important to call this method before
    //attempting to run the wall PID
    ir->getDistance();

    //Update the line sensor
    line->update();

    //If the encoder ISR has fired
    if(readyToSpeedPID) {
        //Update the speed PID
        pid->calcSpeedPID(countsLeft, countsRight);

        //Update the line following PID
        pid->calcLinePID(0);

        //Reset the readyToSpeedPID flag so that we do not run these again until
        //we have new encoder values
        readyToSpeedPID = false;
    }

    //If the IR sensor has updated
    if(readyToWallPID) {
        //Update the wall following PID
        pid->calcWallPID();

        //Reset the Wall PID flag
        readyToWallPID = false;
    }

    //Update the pitch angle
    filter->CalcAngle();

    //Run the motors at the efforts output from the speed PIDs
    motors.setSpeeds(pid->getLeftSpeedEffort(), pid->getRightSpeedEffort());

    //Run the state machine
    return runStateMachine();
}

bool Robot::runStateMachine() {
    switch(currentState) {
        case STARTUP:
            //Stop the robot
            pid->setSpeedTargets(0, 0);

            //If we detect an IR signal (button pressed on the remote)
            if(line->detectIR()) {
                //Start a 1 second timer
                timer->Start(1000);

                //Go to the WAIT_1S state
                incrementState();

                //Give feedback about which state the robot is in
                lcd.clear();
                lcd.print("WAIT_1S");
            }
            break;

        case WAIT_1S:
            //Stop the robot
            pid->setSpeedTargets(0, 0);

            //If the 1 second timer has expired
            if(timer->CheckExpired()) {
                //Go to the WALL_FOLLOW state
                incrementState();

                //Store the current encoder values, in case the robot was moved before starting
                resetEncoderOffset();
            }
            break;

        case WALL_FOLLOW:
            //Set the targets of the speed controller to the efforst output by the Wall PID
            pid->setSpeedTargets(pid->getLeftWallEffort(), pid->getRightWallEffort());
            Serial.print("Left line motor effort: ");
            Serial.print(pid->getLeftWallEffort());
            Serial.print("\t");
            Serial.print("right line motor effort: ");
            Serial.print(pid->getRightWallEffort());
            Serial.print("\t");
            Serial.print("Base motor efforts speed: ");
            Serial.println(pid->getCurrentBaseSpeed());

            //If we detect a line
            /*if(line->detectLine()) {
                //Store the current encoder values to execute a turn
                resetEncoderOffset();

                //Go to the TURN_LEFT_90 state
                incrementState();
            }
            */

            //line->detectLine();

            //pid->setSpeedTargets(PIVOT_SPEED, PIVOT_SPEED);
            if(/*getDegreesTurned() > 90*/ false) {
                lcd.clear();
                lcd.print("YAY");
                pid->setSpeedTargets(0, 0);
            }

            break;

        case TURN_LEFT_90:
            //Set the speed targets for the speed PID such that we spin
            pid->setSpeedTargets(PIVOT_SPEED, -PIVOT_SPEED);

            //If we have rotated 90 degrees since the last resetEncoderOffset() call (made at the end of the last state)
            if(getDegreesTurned() > 90) {
                //TODO: Make sure we're updating speed targets so that the pivot speeds get overwritten when trying to acquire the line
                if(line->doAlign(pid->getLeftLineEffort(), pid->getRightLineEffort())) {
                    //incrementState();
                }
            }
            break;

        case LINE_FOLLOW:
            pid->calcLinePID(pid->getCurrentBaseSpeed());

            //Set the speed targets for the speed PID based on the effort values calculated by the line following PID
            pid->setSpeedTargets(pid->getLeftLineEffort(), pid->getRightLineEffort());

            //If we detect a button press from the remote control
            if(line->detectIR()) {
                //Reset the encoder offset to execute a turn
                resetEncoderOffset();

                //Go to the TURN_RIGHT_90 state
                incrementState();
            }
            break;

        case TURN_RIGHT_90:
            //Set the speed targets for the speed PID such that we spin
            pid->setSpeedTargets(-PIVOT_SPEED, PIVOT_SPEED);

            //If we have rotates 90 degrees since the last resetEncoderOffset() call
            if(getDegreesTurned() < -90) {
                //Go to the DRIVE_UP_RAMP state
                incrementState();
            }
            break;

        case DRIVE_UP_RAMP:
            //Set the speed targets for the speed PID such that we drive straight
            pid->setSpeedTargets(PIVOT_SPEED, PIVOT_SPEED);

            //If we are pitched upwards and are not already on the ramp
            if(filter->getCurrentAngle() < -20 && !onRamp) {
                //Give visual feedback
                lcd.clear();
                lcd.print("ON RAMP");

                //Set the onRamp flag, so that we know 0 degrees is the top of the ramp
                onRamp = true;
            }

            //If we were previously on the ramp, but the pitch is under 10 degrees
            if(onRamp && abs(filter->getCurrentAngle()) < 10) {
                //Give visual feedback
                lcd.clear();
                lcd.print("SPINNING");

                //Reset the encoder offset to execute a turn
                resetEncoderOffset();

                //Go to the SPIN_360 state
                incrementState();
            }
            break;
        case SPIN_360:
            //Set the speed targets such that we pivot
            pid->setSpeedTargets(PIVOT_SPEED, -PIVOT_SPEED);

            Serial.println(getDegreesTurned());

            //If we have turned more than 360 degrees
            if(getDegreesTurned() > 360) {
                //Stop the robot
                pid->setSpeedTargets(0, 0);

                //Go to the STOP state
                incrementState();
            }
            break;
        case STOP:
            //Do nothing in the stop state except keep the robot stopped
            pid->setSpeedTargets(0, 0);
            return true;
            break;
        default:
            break;
    }

    return false;
}


/*
 * Increment the current state. Necessary because incrementing an instance of an
 * enum is not trivial. We must cast the enum to an int, increment, then cast back
 * to an enum.
*/
void Robot::incrementState() {
    currentState = (StateMachine) ((int) currentState + 1);
}


/*
 * Reset the current encoder offset. This is called before we execute turns, so
 * that we know how many encoder ticks we have traveled since beginning the turn
*/
void Robot::resetEncoderOffset() {
    countsLeftOffset = countsLeft;
    countsRightOffset = countsRight;
}


/*
 * Calculates the number of degrees the robot has pivoted based on encoder ticks.
 * We assume that the robot is pivoting, as we are not trying to drive any arcs.
 * We can calculate how far the drivebase has traveled, then find the percentage
 * of the circle we have spun, then turn it into degrees.
 * @returns the number of degrees rotated since the last resetEncoderOffset() call
*/
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


/*
 * Returns the encoder object from the Zumo32U4 library. Used by the encoder ISR
 * so that it can update the encoder tick snapshot.
*/
Zumo32U4Encoders Robot::getEncoders() {
    return encoders;
}


/*
 * Setup the timer to fire interrupts for input capture on the encoders.
*/
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
