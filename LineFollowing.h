#ifndef _LINEFOLLOWING_H
#define _LINEFOLLOWING_H

#include <Arduino.h>
#include <Zumo32U4.h>
#include "EventTimer.h"
#include "PID.h"

#define LIGHT_THRESHOLD 1000

class LineFollowing {

private:
  Zumo32U4LineSensors lineSensors;
  Zumo32U4ProximitySensors proxSensors;
  PID *pid;
  uint16_t lineSensorValues[3] = {0, 0, 0};  //stores linesensor data
  //stores if each of the line sensors see a line
  bool detectLeft = false;
  bool detectCenter = false;
  bool detectRight = false;
  bool parallel = false;      //used to switch between positioning error methods

  double line = 0;            //position of the line


public:
  LineFollowing(PID *thisPID);

  void initLineFollowing(void);
  void update(void);
  void detectLine(void);

  bool detectIR(void);
  bool isParallel(void);
  bool doAlign(float leftEffort, float rightEffort);
  bool doAlignAlong(float leftEffort, float rightEffort);

  double getPosition(void);
  double getPositionAlongLine(void);
};


#endif
