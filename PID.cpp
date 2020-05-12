#include "PID.h"

PID::PID(SharpIR *this_sharp, LineFollowing *this_line)
{
  sharp = this_sharp;
  line = this_line;

  //TODO: default to pulling from params.h

  speedConsts[0] = KP_SPEED;
  speedConsts[1] = KI_SPEED;
  speedConsts[2] = KD_SPEED;

  wallConsts[0] = KP_WALL;
  wallConsts[1] = KI_WALL;
  wallConsts[2] = KD_WALL;

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

   //Serial.println(errorLeft);

  speedEffortLeft = speedConsts[0] * errorLeft + speedConsts[1] * sumLeft;
  speedEffortRight = speedConsts[0] * errorRight + speedConsts[1] * sumRight;
}

void PID::calcWallPID()
{
    dtWall = millis() - lastWallMillis;

    lastWallMillis = millis();

    //caculate error
    float wallError = TARGET_DISTANCE - sharp->AverageDistance();
    //Serial.println(wallError);

    //calculate derivate error
    double wallDerivativeError = (lastWallPosition - sharp->AverageDistance())/(dtWall * pow(10, -3)); //*10^-3 due to millis reading
    lastWallPosition = sharp->AverageDistance();

    //calculate integral error
    wallSum -= wallIntegralSum[currWallIndex];
    wallIntegralSum[currWallIndex] = wallError;
    wallSum += wallError;
    currWallIndex++;

    if(wallIterFlag == false)
        runningWallAvg = wallSum / currWallIndex;
    else
        runningWallAvg = wallSum / wallSampleSize;


    if(currWallIndex == wallSampleSize){ //if buffer is full
      currWallIndex = 0; //reset index to dynamically update error entries
      wallIterFlag = true; //set flag to true
    }

    // double wallChange = lastWallError - wallError; TODO: implement derivate term
    // lastWallError = wallError;

    double effort = wallConsts[0] * wallError + wallConsts[1] * runningWallAvg + wallConsts[2] * wallDerivativeError;

    wallEffortLeft = BASE_WALL_FOLLOW_SPEED - effort;
    wallEffortRight = BASE_WALL_FOLLOW_SPEED + effort;
}

void PID::calcLinePID(float baseSpeedModifier)
{
  double linePosition = 0;

  dtLine = millis() - lastLineMillis;
  lastLineMillis = millis();

  if (line->isParallel()){
    linePosition = line->getPositionAlongLine();
  }
  else{
    linePosition = line->getPosition();
  }
  double lineError = lastLinePositon - linePosition;

//  if(abs(linePosition) + abs(lastLinePositon) > abs(linePosition + lastLinePositon)) //check for sign change in position
//    clearLineIntegralBuffer();

  double lineDerivativeError = (lastLinePositon - linePosition)/dtLine;
  lastLinePositon = linePosition;

  lineSum -= lineIntegralSum[currLineIndex];
  lineIntegralSum[currLineIndex] = lineError;
  lineSum += lineError;
  currLineIndex++;

  if(lineIterFlag == false)
      runningLineAvg = lineSum / currLineIndex;
  else
      runningLineAvg = lineSum / lineSampleSize;

  if(currLineIndex == lineSampleSize){ //if buffer is full
    currLineIndex = 0; //reset index to dynamically update error entries
    lineIterFlag = true; //set flag to true
  }

  if (lineError > 0){
    lineEffortLeft = (BASE_LINE_FOLLOW_SPEED - baseSpeedModifier);
    lineEffortRight = (BASE_LINE_FOLLOW_SPEED - baseSpeedModifier) + (lineConsts[0] * lineError + lineConsts[1] * runningLineAvg + lineConsts[2] * lineDerivativeError);
  }
  else if (lineError < 0){
    lineEffortLeft = (BASE_LINE_FOLLOW_SPEED - baseSpeedModifier) - (lineConsts[0] * lineError + lineConsts[1] * runningLineAvg + lineConsts[2] * lineDerivativeError);
    lineEffortRight = (BASE_LINE_FOLLOW_SPEED - baseSpeedModifier);
  }
    else if(lineError == 0){
      lineEffortLeft = (BASE_LINE_FOLLOW_SPEED - baseSpeedModifier);
      lineEffortRight = (BASE_LINE_FOLLOW_SPEED - baseSpeedModifier);
  }
}

void PID::clearLineIntegralBuffer(void) {

  for(uint8_t i = 0; i < lineSampleSize; i++) {
    lineIntegralSum[i] = 0;
  }

	lineSum = 0; //reset vars
	currLineIndex = 0;
	lineIterFlag = false;
}

void PID::updateCurrentBaseSpeed(float baseSpeedModifier) {
  currentBaseSpeed = currentBaseSpeed - baseSpeedModifier;
}

float PID::getLeftLineEffort() {
//    Serial.print("Left line effort: ");
//    Serial.print("\t");
//    Serial.print(lineEffortLeft);
    return lineEffortLeft;
}

float PID::getRightLineEffort() {
//    Serial.print("Right line effort: ");
//    Serial.print("\t");
//    Serial.print(lineEffortRight);
//    Serial.println("");
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
