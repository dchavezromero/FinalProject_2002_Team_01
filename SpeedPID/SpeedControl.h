#include "arduino.h"
#include <Zumo32U4Motors.h>

#ifndef _SPEEDCONTROL_H
#define _SPEEDCONTROL_H

class SpeedControl{
private:
    int targetLeftSpeed, targetRightSpeed;
public:
    Zumo32U4Motors motors;

    float Kp = 18;
    float Ki = 1.5;
    float Kd = 0;

    float targetLeft = 0;
    float targetRight = 0;


    int16_t speedLeft = 0;
    int16_t speedRight = 0;

    //for tracking previous counts
    int16_t prevLeft = 0;
    int16_t prevRight = 0;

    //error sum
    int16_t sumLeft = 0;
    int16_t sumRight = 0;

    int16_t errorLeft = 0;
    int16_t errorRight = 0;

    SpeedControl(float P, float I, float D);
    SpeedControl();

    void Init();
    void speedPID(int16_t countsLeft, int16_t countsRight, int targetLeft, int targetRight);

    void setTargetSpeeds(int targetLeft, int targetRight);
};


#endif
