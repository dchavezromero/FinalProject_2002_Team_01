#ifndef _ROBOT_H
#define _ROBOT_H

#include <Arduino.h>
#include "EventTimer.h"
#include "PID.h"
#include "SharpIR.h"
#include "LineFollowing.h"
#include "filter.h"

//Distance between wheels in CM
#define WHEEL_TRACK 8.5

//Number of ticks per centimeters displaced
#define TICKS_TO_CM 52.0

#define PIVOT_CIRCUMFERENCE (PI*WHEEL_TRACK) //The circumference of the circle that the robot sits on

//How fast the robot should pivot for turns. Out of a maximum of 75
#define PIVOT_SPEED 12

//Forward declare classes because of include dependencies
class SharpIR;
class LineFollowing;
class PID;

class Robot {
    static Robot *instance; //Singleton design patter -- Makes ISRs bearable
private:
    Robot();

    //The different states the robot can be in
    enum StateMachine {
        STARTUP,
        WAIT_1S,
        WALL_FOLLOW,
        CREEP_FORWARD,
        TURN_LEFT_90,
        LINE_FOLLOW,
        TURN_RIGHT_90,
        DRIVE_UP_RAMP,
        SPIN_360,
        STOP,
    };

    //The robot should start in the STARTUP state
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

    //Whether or not we've reached the ramp in the DRIVE_UP_RAMP state
    bool onRamp = false;

    //Used to "reset" the values of the encoders
    int16_t countsLeftOffset = 0;
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
