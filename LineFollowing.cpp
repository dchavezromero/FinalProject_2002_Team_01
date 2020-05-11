#include "LineFollowing.h"
#include "arduino.h"

//allows to change the line sensor threshold
LineFollowing::LineFollowing() {
}

//initiate the line sensor and proximity sensor
void LineFollowing::Init(PID *thisPid) {
    pid = thisPid;

    lineSensors.initThreeSensors();
    proxSensors.initFrontSensor();
}


//update the line sensors(doing the IR was giving a lot of noise)
void LineFollowing::update(void) {
   lineSensors.read(lineSensorValues)
}

//should detect when the remote sends a signal
bool LineFollowing::detectIR(void) {
  return proxSensors.readBasicFront();
}


//set the values based on if it detected a line
void LineFollowing::detectLine(void) {
  detectLeft = (lineSensorValues[0] > LIGHT_THRESHOLD);
  detectCenter = (lineSensorValues[1] > LIGHT_THRESHOLD);
  detectRight = (lineSensorValues[2] > LIGHT_THRESHOLD);
}

/*
 * declares values to the position of the line based on what sensors can see it
 * with the line being centered as zero
 */
double LineFollowing::getPosition(void) {
  this->detectLine();

  if(detectLeft && !detectCenter && !detectRight){
    line = 2;
  }
  else if(detectLeft && detectCenter && !detectRight){
    line = 1;
  }
  else if(!detectLeft && detectCenter && !detectRight){
    line = 0;
  }
  else if(!detectLeft && detectCenter && detectRight){
    line = -1;
  }
  else if(!detectLeft && !detectCenter && detectRight){
    line = -2;
  }
  else
    line = 0;

  return line;
}


/*
 * declares values to the position of the line based on what sensors can see it
 * with the line being along all of the sensors being zero
 */
double LineFollowing::getPositionAlongLine(void){
  this->detectLine();

  if(detectLeft && detectCenter && !detectRight){
    line = 2;
  }
  else if(detectLeft && !detectCenter && !detectRight){
    line = 1;
  }
   else if(!detectLeft && !detectCenter && !detectRight){
    line = 0;
  }
  else if(!detectLeft && !detectCenter && detectRight){
    line = -1;
  }
   else if(!detectLeft && detectCenter && detectRight){
    line = -2;
  }

  else if(!detectLeft && detectCenter && !detectRight){
    line = 5;
   }
  else
    line = 0.5;

  return line;
}

bool LineFollowing::isParallel(void)
{
  return parallel;
}

//adjusts slowly to try to align the center sensor to the line
bool LineFollowing::doAlign(float leftEffort, float rightEffort){

    pid->calcLinePID(leftEffort, rightEffort, 10);

    if (leftEffort == 0 && rightEffort == 0){
        return true;
    }

    return false;
}

//adjusts slowly but uses the other values for location
bool LineFollowing::doAlignAlong(float leftEffort, float rightEffort){

  parallel = true;

  pid->calcLinePID(leftEffort, rightEffort, 10);

  if (leftEffort == 0 && rightEffort == 0){
    parallel = false;
    return true;
  }

  return false;
}
