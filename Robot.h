#ifndef _ROBOT_H
#define _ROBOT_H

#include <Arduino.h>
#include "EventTimer.h"
#include "PID.h"
#include "SharpIR.h"
#include "LineFollowing.h"
#include "filter.h"

#define WHEEL_TRACK 8.5 //Distance between wheels in CM
#define TICKS_TO_CM 52.0 //TODO: Find this value

#define PIVOT_CIRCUMFERENCE (PI*WHEEL_TRACK)

#define PIVOT_SPEED 20

class SharpIR;
class LineFollowing;
class PID;

class Robot {
    static Robot *instance; //Singleton design patter -- Makes ISRs bearable
private:
    Robot();

    enum StateMachine {
        STARTUP,
        WAIT_1S,
        WALL_FOLLOW,
        TURN_LEFT_90,
        LINE_FOLLOW,
        TURN_RIGHT_90,
        DRIVE_UP_RAMP,
        SPIN_360,
        STOP,
    };

    enum StateMachine currentState = STARTUP;

    void incrementState();

    void updateSensors();
    bool runStateMachine();

    EventTimer *timer;
    SharpIR *ir;
    LineFollowing *line;

    Zumo32U4Motors motors;
    Zumo32U4Encoders encoders;
    Zumo32U4LineSensors lineSensors;
    Zumo32U4LCD lcd;

    ComplementaryFilter *filter;

    PID *pid;

    bool onRamp = false; //Whether or not we've reached the ramp in the DRIVE_UP_RAMP state

    int16_t countsLeftOffset = 0; //Used to "reset" the values of the encoders
    int16_t countsRightOffset = 0;

    void resetEncoderOffset();

    void setupEncoderTimer();

public:
    static Robot *getRobot();

    void init();
    bool loop();

    bool readyToSpeedPID = false;
    bool readyToWallPID = false;

    int16_t countsLeft = 0;
    int16_t countsRight = 0;

    float getDegreesTurned();

    Zumo32U4Encoders getEncoders();
};

#endif
