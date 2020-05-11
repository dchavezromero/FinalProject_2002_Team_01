#include "PID.h"

PID::PID(SharpIR *this_sharp, LineFollowing *this_line)
{
  this_sharp = sharp;
  this_line = line;

  //TODO: default to pulling from params.h

  // speedConsts[0] = kp_motors;
  // speedConsts[1] = ki_motors;
  // speedConsts[2] = kd_motors;
  //
  // irConsts[0] = kp_ir;
  // irConsts[1] = ki_ir;
  // irConsts[2] = kd_ir;
  //
  // gyroConsts[0] = kp_gyro;
  // gyroConsts[1] = ki_gyro;
  // gyroConsts[2] = kd_gyro;
}

void PID::setSpeedPID(double P, double I, double D)
{
  speedConsts[0] = P;
  speedConsts[1] = I;
  speedConsts[2] = D;
}

void PID::setWallPID(double P, double I, double D)
{
  wallConsts[0] = P;
  wallConsts[1] = I;
  wallConsts[2] = D;
}

void PID::setGyroPID(double P, double I, double D)
{
  gyroConsts[0] = P;
  gyroConsts[1] = I;
  gyroConsts[2] = D;
}

void PID::setSpeedTargets(int16_t targetLeftSpeed, int16_t targetRightSpeed) {
  targetLeft = targetLeftSpeed;
  targetRight = targetRightSpeed;
}

void PID::calcSpeedPID(int16_t countsLeft, int16_t countsRight)
{
  noInterrupts();
  int16_t speedLeft = countsLeft - prevLeft;
  int16_t speedRight = countsRight - prevRight;

  int16_t prevLeft = countsLeft;
  int16_t prevRight = countsRight;
  interrupts();

  int16_t errorLeft = targetLeft - speedLeft;
  int16_t sumLeft += errorLeft;
  int16_t errorRight = targetRight - speedRight;
  int16_t sumRight += errorRight;

  if(sumLeft>= 200){
     sumLeft = 200;
     sumRight = 200;
   }

  speedEffortLeft = speedConsts[0] * errorLeft + speedConsts[1] * sumLeft;
  speedEffortRight = speedConsts[0] * errorRight + speedConsts[1] * sumRight;
}

void PID::calcWallPID()
{
  double wallSum = 0;

  double wallError = TARGET_DISTANCE - sharp.getDistance();

  if(currIndex >= 10)
    currIndex = 0;

  wallIntegralSum[currIndex] = wallError;
  currIndex++;

  for(uint4_t i = 0, i < 10, i++)
  {
    wallSum += wallIntegralSum[i];
  }

  wallSum = wallSum / currIndex;

  // double wallChange = lastWallError - wallError; TODO: implement derivate term
  // lastWallError = wallError;

  double effort = wallConsts[0] * wallError + wallConsts[1] * wallSum;

  wallEffortLeft = BASE_WALL_FOLLOW_SPEED - effort;
  wallEffortRight = BASE_WALL_FOLLOW_SPEED + effort;
}
