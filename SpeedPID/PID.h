#ifndef _PID_H
#define _PID_H

#include "../sharp_ir/SharpIR.h"            //include the IR
#include "../InertialSensors/filter.h"             //include IMU
#include "../linefollowing/LineFollowing.h"

class PID {

private:
  double speedConsts[3] = {0, 0, 0};
  double wallConsts[3] = {0, 0, 0};
  double gyroConsts[3] = {0, 0, 0};

  SharpIR *sharp;
  LineFollowing *line;

  float effortLeft = 0;
  float effortRight = 0;

  int16_t targetLeft = 0;
  int16_t targetRight = 0;

  float leftWallEffort = 0;
  float rightWallEffort = 0;

  float leftLineEffort = 0;


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

  float getLeftWallEffort();
  float getRightWallEffort();

  float getLeftLineEffort();
  float getRightLineEffort();
};

#endif
