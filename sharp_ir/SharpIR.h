#include "arduino.h"
#include <EventTimer.h>

#ifndef _SHARPIR_H
#define _SHARPIR_H

class SharpIR {
private:
    float leftEffort, rightEffort;
public:
    EventTimer timer;                 //nonblocking timer for when to read
    int IRPin = A6;                   //default IR pin
    int val = 0;                      //raw IR reading
    unsigned int waitingTime = 50;    //millis between readings

    double Kp = 0.4;    //PID P constant
    double Ki = 0.1;    //PID I constant
    double Kd = 0;      //PID D constant

    double lastWallError = 0;   //used to calculate the D term in IRPID
    double wallSum = 0;         //used for I term in IRPID
    double sumWall[10];         //used to limit I term in rollingSUM
    int i = 0;                  //used to rotate through arrays terms

    SharpIR(uint8_t pin);
    SharpIR();
    float getDistance(float& Dist);
    setIRPID(double P, double I, double D);
    float IRPID(float& LeftEffort, float&RightEffort, float& wallError,double distance, double baseSpeed);
    double rollingSum(float error);
};


  #endif
