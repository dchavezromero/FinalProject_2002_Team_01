#include "PID.h"

PID::PID(SharpIR *this_sharp, SpeedControl *this_speed, LineFollowing *this_line,
double kp_motors, double ki_motors, double kd_motors,
double kp_ir, double ki_ir, double kd_ir,
double kp_gyro, double ki_gyro, double kd_gyro)
{
  this_sharp = sharp;
  this_speed = speed;
  this_line = line;

  motorConsts[0] = kp_motors;
  motorConsts[1] = ki_motors;
  motorConsts[2] = kd_motors;

  irConsts[0] = kp_ir;
  irConsts[1] = ki_ir;
  irConsts[2] = kd_ir;

  gyroConsts[0] = kp_gyro;
  gyroConsts[1] = ki_gyro;
  gyroConsts[2] = kd_gyro;
}

void PID::setMotorsPID(double P, double I, double D)
{
  motorConsts[0] = kp_motors;
  motorConsts[1] = ki_motors;
  motorConsts[2] = kd_motors;
}

void PID::setIRPID(double P, double I, double D)
{
  irConsts[0] = kp_ir;
  irConsts[1] = ki_ir;
  irConsts[2] = kd_ir;
}

void PID::setGyroPID(double P, double I, double D)
{
  gyroConsts[0] = kp_gyro;
  gyroConsts[1] = ki_gyro;
  gyroConsts[2] = kd_gyro;
}
