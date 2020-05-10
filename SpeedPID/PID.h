#ifndef _PID_H
#define _PID_H

#include <SharpIR.h>            //include the IR
#include <filter.h>             //include IMU
#include <SpeedControl.h>       //include Speed PID control

class PID {

private:
  double motorConsts[3] = {0, 0, 0};
  double irConsts[3] = {0, 0, 0};
  double gyroConsts[3] = {0, 0, 0};

  SpeedControl speed;
  SharpIR sharp;
  LineFollowing line;

public:
  PID(SharpIR *this_sharp, SpeedControl *this_speed, LineFollowing *this_line,
  double kp_motors, double ki_motors, double kd_motors,
  double kp_ir, double ki_ir, double kd_ir,
  double kp_gyro, double ki_gyro, double kd_gyro);

  void calcMotorsPID();
  void calcIRPID();
  void calcGyroPID();

  void setMotorsPID(double P, double I, double D);
  void setIRPID(double P, double I, double D);
  void setGyroPID(double P, double I, double D);

  void getMotorEfforts(float &left, float &right);
  void getIREfforts(float &left, float &right);
  void getGyroEfforts(float &left, float &right);


};

#endif
