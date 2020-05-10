#ifndef _ROBOT_H
#define _ROBOT_H

#include "EventTimer.h"
#include "PID.h"

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
        TURN_LEFT_90_2,
        DRIVE_UP_RAMP,
        SPIN_360,
        STOP,
    };

    enum StateMachine currentState = STARTUP;

    void updateSensors();
    bool runStateMachine();

    EventTimer *timer;
    SharpIR *ir;
    SpeedControl *speed;
    LineFollowing *line;

    Zumo32U4Motors motors;
    Zumo32U4Encoders encoders;
    Zumo32U4LineSensors lineSensors;
    Zumo32U4ProximitySensors proxSensors;
    Zumo32U4LCD lcd;

    ComplementaryFilter *filter;

    PID *pid;

public:
    static Robot *getRobot();

    void init();
    bool loop();

    bool readyToPid = false;
    int16_t countsLeft = 0;
    int16_t countsRight = 0;
};

#endif
