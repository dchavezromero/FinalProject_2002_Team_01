#ifndef _FILTER_H
#define _FILTER_H

#include "arduino.h"
#include <Wire.h>
#include <Zumo32U4.h>

#define RAD_TO_DEG 57.29577
#define READING_TO_DPS (8.75 * 1000) //Sensitivity * 1000 (mdps -> dps)

class ComplementaryFilter {
public:
    LSM303 accel;
    L3G gyro;
    Zumo32U4Motors motors;
    //Zumo32U4ButtonA buttonA;

    int LastMillis;           //used in CalcAngle to get DeltaT
    int dt;               //change in time to change gyro d/s to Degrees
    double k = .8;            //tune filter towards gyro vs accelerometer
    float prediction;         //gyro angle prediction
    float gyroBias = 0;       //how far off the gyro is
    double E = 0.05;          //tune how much the difference in angles changes the bias
    float accXoffset = 0;     //used to rest the what angle is 0 in X
    float accYoffset = 0;     //in Y
    float accZoffset = 0;     // in Z
    float observedAngle = 0;  //the angle based on the accelerometer
    float gyro_dps = 0;           //the raw reading in degrees per second from the gyro
    int i = 201;              //used to keep track of the number of readings used to reset the angle
    int n = 0;                //in Average to shift the array input

    double Kp = 0.13;         //P value in GyroPID
    double Ki = 0.0007;       //I value in GyroPID
    double Kd = 0;            //D value in GyroPID

    bool initialized = false;   //flag to keep track of initialization
    double turnSum = 0;         //I term for GyroPID
    double gyroInitial = 0;     //initialization value to get rid of bias
    double lastTurnError = 90;  //used to calc D term for GyroPID
    double lastMillis = 0;      //used to calc DeltaT in GyroPID
    double LastAngle = 0;       //used to find the angle and work multiplle time in a row
    bool doneTurn = false;   //changes to true when the error is within 2 degrees


    Init(void);
    bool CalcAngle(float& Gyro, float& prediction, int axis);
    float Average(float prediction);
    Read(void);
    void setGyroPID(double P, double I, double D);
    float GyroPID(float& LeftEffort, float&RightEffort, float& turnError, double angle);
    boolean doneTurning();
};



#endif
