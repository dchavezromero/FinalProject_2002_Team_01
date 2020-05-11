#include "PID.h"

PID::PID(SharpIR *this_sharp, LineFollowing *this_line)
{
  this_sharp = sharp;
  this_line = line;

  //TODO: default to pulling from params.h

  speedConsts[0] = KP_SPEED;
  speedConsts[1] = KI_SPEED;
  speedConsts[2] = KD_SPEED;

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

void PID::setLinePID(double P, double I, double D)
{
  lineConsts[0] = P;
  lineConsts[1] = I;
  lineConsts[2] = D;
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

  prevLeft = countsLeft;
  prevRight = countsRight;
  interrupts();

  int16_t errorLeft = targetLeft - speedLeft;
  sumLeft += errorLeft;
  int16_t errorRight = targetRight - speedRight;
  sumRight += errorRight;

  if(sumLeft >= 200){
     sumLeft = 200;
     sumRight = 200;
   }

  speedEffortLeft = speedConsts[0] * errorLeft + speedConsts[1] * sumLeft;
  speedEffortRight = speedConsts[0] * errorRight + speedConsts[1] * sumRight;
}

void PID::calcWallPID()
{
  double wallSum = 0;

  double wallError = TARGET_DISTANCE - sharp->getDistance();

  if(currWallIndex >= 10)
    currWallIndex = 0;

  wallIntegralSum[currWallIndex] = wallError;
  currWallIndex++;

  for(char i = 0; i < 10; i++)
  {
    wallSum += wallIntegralSum[i];
  }

  wallSum = wallSum / currWallIndex;

  // double wallChange = lastWallError - wallError; TODO: implement derivate term
  // lastWallError = wallError;

  double effort = wallConsts[0] * wallError + wallConsts[1] * wallSum;

  wallEffortLeft = BASE_WALL_FOLLOW_SPEED - effort;
  wallEffortRight = BASE_WALL_FOLLOW_SPEED + effort;
}

void PID::calcLinePID(float thisLineEffortLeft, float thisLineEffortRight, float baseSpeedModifier)
{
  double lineError = 0;
  double lineSum = 0;
  lineEffortLeft = thisLineEffortLeft;
  lineEffortRight = thisLineEffortRight;

  if (line->isParallel()){
    lineError = line->getPositionAlongLine();
  }
  else{
    lineError = line->getPosition();
  }

  if(currlineIndex >= 10)
    currlineIndex = 0;

  lineIntegralSum[currlineIndex] = lineError;
  currlineIndex++;

  for(char i = 0; i < 10; i++)
  {
    lineSum += lineIntegralSum[i];
  }

  lineSum = lineSum / currlineIndex;

  double lineDiff = lastlineError - lineError;
  lastlineError = lineError;

  if (lineError > 0){
    lineEffortLeft = (BASE_LINE_FOLLOW_SPEED - baseSpeedModifier);
    lineEffortRight = (BASE_LINE_FOLLOW_SPEED - baseSpeedModifier) + (lineConsts[0] * lineError + lineConsts[1] * lineSum + lineConsts[2] * lineDiff);
  }
  else if (lineError < 0){
    lineEffortLeft = (BASE_LINE_FOLLOW_SPEED - baseSpeedModifier) - (lineConsts[0] * lineError + lineConsts[1] * lineSum + lineConsts[2] * lineDiff);
    lineEffortRight = (BASE_LINE_FOLLOW_SPEED - baseSpeedModifier);
  }
    else if(lineError == 0){
      lineEffortLeft = (BASE_LINE_FOLLOW_SPEED - baseSpeedModifier);
      lineEffortRight = (BASE_LINE_FOLLOW_SPEED - baseSpeedModifier);
  }
}

void PID::updateCurrentBaseSpeed(float baseSpeedModifier) {
  currentBaseSpeed = currentBaseSpeed - baseSpeedModifier;
}

float PID::getLeftLineEffort() {
    return lineEffortLeft;
}

float PID::getRightLineEffort() {
    return lineEffortRight;
}

float PID::getCurrentBaseSpeed() {
    return currentBaseSpeed;
}

float PID::getLeftWallEffort() {
    return wallEffortLeft;
}

float PID::getRightWallEffort() {
    return wallEffortRight;
}

float PID::getLeftSpeedEffort() {
    return speedEffortLeft;
}

float PID::getRightSpeedEffort() {
    return speedEffortRight;
}
