#ifndef _PID_H
#define _PID_H

#include "SharpIR.h"            //include the IR
#include "filter.h"             //include IMU
#include "LineFollowing.h"

#define BASE_WALL_FOLLOW_SPEED 15
#define BASE_LINE_FOLLOW_SPEED 10

#define TARGET_DISTANCE 20

#define KP_WALL 0.5
#define KI_WALL 0.05
#define KD_WALL 10.0

#define KP_LINE 2.0
#define KI_LINE 0.003
#define KD_LINE -0.001

#define KP_SPEED 18.0
#define KI_SPEED 1.5
#define KD_SPEED 0.0

class LineFollowing;
class SharpIR;

class PID {
private:
  double speedConsts[3] = {0, 0, 0};
  double wallConsts[3] = {0, 0, 0};
  double lineConsts[3] = {0, 0, 0};

  static const int wallSampleSize = 10;
  double wallIntegralSum[wallSampleSize] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //used to limit I term
  double lastWallError = 0;   //used to calculate the D term in IRPID
  float lastWallPosition = 0;
  float wallSum = 0;
  float runningWallAvg = 0;
  char currWallIndex = 0; //used to reference integral windup buffer
  unsigned long lastWallMillis = 0;
  unsigned long dtWall = 0;
  bool wallIterFlag = false;

  int sumLeft = 0;
  int sumRight = 0;

  int16_t prevLeft = 0;
  int16_t prevRight = 0;

  unsigned long lastLineMillis = 0;
  unsigned long dtLine = 0;
  bool lineIterFlag = false;
  float lineSum = 0;
  char currLineIndex = 0; //used to reference integral windup buffer
  float runningLineAvg = 0;
  static const int lineSampleSize = 10;
  double lineIntegralSum[lineSampleSize] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //used to limit I term
  double lastLineError = 0;   //used to calculate the D term in IRPID

  char currlineIndex = 0; //used to reference integral windup buffer
  float currentBaseSpeed = BASE_LINE_FOLLOW_SPEED;

  SharpIR *sharp;
  LineFollowing *line;

  int16_t targetLeft = 0;
  int16_t targetRight = 0;

  float speedEffortLeft = 0;
  float speedEffortRight = 0;

  float wallEffortLeft = 0;
  float wallEffortRight = 0;

  float lineEffortLeft = 0;
  float lineEffortRight = 0;

public:
  PID(SharpIR *this_sharp, LineFollowing *this_line);

  void calcSpeedPID(int16_t countsLeft, int16_t countsRight);
  void calcWallPID();
  void calcLinePID(float baseSpeedModifier);

  void setSpeedTargets(int16_t targetLeftSpeed, int16_t targetRightSpeed);

  void setSpeedPID(double P, double I, double D);
  void setWallPID(double P, double I, double D);
  void setLinePID(double P, double I, double D);

  float getLeftSpeedEffort();
  float getRightSpeedEffort();

  float getLeftWallEffort();
  float getRightWallEffort();

  float getLeftLineEffort();
  float getRightLineEffort();
  float getCurrentBaseSpeed();

  void updateCurrentBaseSpeed(float baseSpeedModifier);
};

#endif
