#ifndef _PID_H
#define _PID_H

#include "SharpIR.h"            //include the IR
#include "filter.h"             //include IMU
#include "LineFollowing.h"

#define BASE_WALL_FOLLOW_SPEED 30
#define BASE_LINE_FOLLOW_SPEED 10

#define TARGET_DISTANCE 30

class LineFollowing;

class PID {
private:
  double speedConsts[3] = {0, 0, 0};
  double wallConsts[3] = {0, 0, 0};
  double lineConsts[3] = {0, 0, 0};

  double wallIntegralSum[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //used to limit I term
  double lastWallError = 0;   //used to calculate the D term in IRPID
  char currWallIndex = 0; //used to reference integral windup buffer

  int16_t sumLeft = 0;
  int16_t sumRight = 0;

  int16_t prevLeft = 0;
  int16_t prevRight = 0;

  double lineIntegralSum[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //used to limit I term
  double lastlineError = 0;   //used to calculate the D term in IRPID
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
  void calcLinePID(float thisLineEffortLeft, float thisLineEffortRight, float baseSpeedModifier);

  void setSpeedTargets(int16_t targetLeftSpeed, int16_t targetRightSpeed);

  void setSpeedPID(double P, double I, double D);
  void setWallPID(double P, double I, double D);
  void setLinePID(double P, double I, double D);

  float getLeftWallEffort();
  float getRightWallEffort();

  float getLeftLineEffort();
  float getRightLineEffort();
  float getCurrentBaseSpeed();

  void updateCurrentBaseSpeed(float baseSpeedModifier);
};

#endif
