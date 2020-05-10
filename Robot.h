#ifndef _ROBOT_H
#define _ROBOT_H

class Robot {
    static Robot *instance; //Singleton design patter -- Makes ISRs bearable
private:
    Robot();

    enum StateMachine {
        STARTUP,
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
public:
    static Robot *getRobot();

    void init();
    bool loop();
};

#endif
