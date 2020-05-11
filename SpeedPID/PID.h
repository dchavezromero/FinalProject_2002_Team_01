#ifndef _PID_H
#define _PID_H

#include <SharpIR.h>            //include the IR
#include <filter.h>             //include IMU
#include <SpeedControl.h>       //include Speed PID control

#define BASE_WALL_FOLLOW_SPEED 30
#define TARGET_DISTANCE 30

class PID {

private:
  double speedConsts[3] = {0, 0, 0};
  double wallConsts[3] = {0, 0, 0};
  double gyroConsts[3] = {0, 0, 0};

  double wallIntegralSum[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //used to limit I term
  double lastWallError = 0;   //used to calculate the D term in IRPID
  uint4_t currIndex = 0; //used to reference integral windup buffer


  SharpIR sharp;
  LineFollowing line;

  int16_t targetLeft = 0;
  int16_t targetRight = 0;

  float speedEffortLeft = 0;
  float speedEffortRight = 0;

  float wallEffortLeft = 0;
  float wallEffortRight = 0;


public:
  PID(SharpIR *this_sharp, LineFollowing *this_line);

  void calcSpeedPID(int16_t countsLeft, int16_t countsRight);
  void calcWallPID();
  void calcGyroPID();

  void setSpeedTargets(int16_t targetLeftSpeed, int16_t targetRightSpeed);

  void setSpeedPID(double P, double I, double D);
  void setWallPID(double P, double I, double D);
  void setGyroPID(double P, double I, double D);

  void getSpeedEfforts(float &left, float &right);
  void getWallEfforts(float &left, float &right);
  void getGyroEfforts(float &left, float &right);


};

#endif
