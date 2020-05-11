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
    proxSensors.pullupsOn();
}


//update the line sensors(doing the IR was giving a lot of noise)
void LineFollowing::update(void) {
   lineSensors.read(lineSensorValues);
}

//should detect when the remote sends a signal
bool LineFollowing::detectIR(void) {
    //TODO: MAKE SURE THIS ISNT A BUG
    bool detected = false;

    proxSensors.lineSensorEmittersOff();
    detected = proxSensors.readBasicFront();

    //This is sorcery. lineSensorsEmittersOn() does not exist in the Zumo32U4 library
    //  so we had to reverse engineer the hacker 4-chan to figure out how to turn the
    //  emitters back on
    digitalWrite(22, HIGH);
    pinMode(22, INPUT_PULLUP);
    delayMicroseconds(578);

    return detected;
}


//set the values based on if it detected a line
bool LineFollowing::detectLine(void) {
  detectLeft = (lineSensorValues[0] > LIGHT_THRESHOLD);
  detectCenter = (lineSensorValues[1] > LIGHT_THRESHOLD);
  detectRight = (lineSensorValues[2] > LIGHT_THRESHOLD);

  Serial.print("Detected left line: ");
  Serial.print("\t");
  Serial.print(lineSensorValues[0]);
  Serial.print("\t");
  Serial.print(detectLeft);
  Serial.println("");
  Serial.print("Detected center line: ");
  Serial.print("\t");
  Serial.print(lineSensorValues[1]);
  Serial.print("\t");
  Serial.print(detectCenter);
  Serial.println("");
  Serial.print("Detected right line: ");
  Serial.print("\t");
  Serial.print(lineSensorValues[2]);
  Serial.print("\t");
  Serial.print(detectRight);
  Serial.println("");

return (detectLeft || detectCenter || detectRight);
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

    pid->calcLinePID(leftEffort, rightEffort, BASE_LINE_FOLLOW_SPEED);

    if (leftEffort == 0 && rightEffort == 0){
        return true;
    }

    return false;
}

//adjusts slowly but uses the other values for location
bool LineFollowing::doAlignAlong(float leftEffort, float rightEffort){

  parallel = true;

  pid->calcLinePID(leftEffort, rightEffort, BASE_LINE_FOLLOW_SPEED);

  if (leftEffort == 0 && rightEffort == 0){
    parallel = false;
    return true;
  }

  return false;
}
