#include "LineFollowing.h"
#include "arduino.h"

//allows to change the line sensor threshold
LineFollowing::LineFollowing(double threshold){
  Threshold = threshold;
}

//overloaded constructor sets it to default 1000
LineFollowing::LineFollowing(){
  Threshold = 1000;
}

//initiate the line sensor and proximity sensor
LineFollowing::Init(){
  lineSensors.initThreeSensors();
  proxSensors.initFrontSensor();
}


//update the line sensors(doing the IR was giving a lot of noise)
LineFollowing::Update(){
   lineSensors.read(lineSensorValues);
}

//should detect when the remote sends a signal
bool LineFollowing::DetectIR(){
  return proxSensors.readBasicFront();
}

//can reset the PID values
LineFollowing::setLinePID(double P, double I, double D){
  Kp = P;
  Ki = I;
  Kd = D;
}

//set the values based on if it detected a line
LineFollowing::Detect(){
  DetectLeft = (lineSensorValues[0] > Threshold);
  DetectCenter =(lineSensorValues[1] > Threshold);
  DetectRight =(lineSensorValues[2] > Threshold);
}

/*
 * declares values to the position of the line based on what sensors can see it
 * with the line being centered as zero
 */
int LineFollowing::Position(){
  this->Detect();
  if(DetectLeft && !DetectCenter && !DetectRight){
    line = 2;
  }
  else if(DetectLeft && DetectCenter && !DetectRight){
    line = 1;
  }
  else if(!DetectLeft && DetectCenter && !DetectRight){
    line = 0;
  }
  else if(!DetectLeft && DetectCenter && DetectRight){
    line = -1;
  }
  else if(!DetectLeft && !DetectCenter && DetectRight){
    line = -2;
  }
  else
  line = 0.5;

  return line;
}


/*
 * declares values to the position of the line based on what sensors can see it
 * with the line being along all of the sensors being zero
 */
int LineFollowing::PositionAlongLine(){
  this->Detect();

  if(DetectLeft && DetectCenter && !DetectRight){
    line = 2;
  }
  else if(DetectLeft && !DetectCenter && !DetectRight){
    line = 1;
  }
   else if(!DetectLeft && !DetectCenter && !DetectRight){
    line = 0;
  }
  else if(!DetectLeft && !DetectCenter && DetectRight){
    line = -1;
  }
   else if(!DetectLeft && DetectCenter && DetectRight){
    line = -2;
  }

  else if(!DetectLeft && DetectCenter && !DetectRight){
    line = 5;
   }
  else
  line = 0.5;

  return line;
}

//adjusts slowly to try to align the center sensor to the line
bool LineFollowing::Align(float LeftEffort, float RightEffort){
    float left = LeftEffort;
    float right = RightEffort;
    
    this->LinePID(left, right, 0);
    if (right == 0 && left == 0){
        return true;
    }

    return false;
}

//adjusts slowly but uses the other values for location
bool LineFollowing::AlignAlong(float& LeftEffort, float& RightEffort){
    parallel = true;
    this->LinePID(LeftEffort, RightEffort, 0);
    if (RightEffort == 0 && LeftEffort == 0){
    parallel = false;
        return true;
    }

    return false;
}

//PID based line following with slow adjustments
float LineFollowing::LinePID(float& LeftEffort, float& RightEffort,double basespeed){
 if (parallel){
  error = this->PositionAlongLine();
 }
 else{
  error = this->Position();
 }
  lineSum += Sum(error);
  double lineDiff = lastError - error;
  lastError = error;
  if (error > 0){
    LeftEffort = basespeed;
    RightEffort = basespeed + (Kp * error + Ki * lineSum + Kd * lineDiff);
  }
  else if (error < 0){
    LeftEffort = basespeed - (Kp * error + Ki * lineSum + Kd * lineDiff);
    RightEffort = basespeed;
  }
  else if(error == 0){
    LeftEffort = basespeed;
    RightEffort = basespeed;
  }


}

//limits I term
LineFollowing::Sum(int error){
   sumLine[i] = error;
    i++;
    if (i >= 10){
     i = 0;
    }
    double sumLineError = sumLine[0] + sumLine[1] + sumLine[2] + sumLine[3] + sumLine[4] +
     sumLine[5] + sumLine[6] + sumLine[7] + sumLine[8] + sumLine[9];
  return sumLineError;

}
