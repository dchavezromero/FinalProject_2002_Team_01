#include "SharpIR.h"
#include "arduino.h"

//can change what pin it reads
SharpIR::SharpIR(uint8_t pin){
  IRPin = pin;
}

//overloaded constructor defaults to pin A6
SharpIR::SharpIR(){
  IRPin = A6;
}

//converts the IR reading into a distance 
//most accurate around 18 cm and drifts a little around 10 cm from there
float SharpIR::Distance(float& Dist){
  if (timer.CheckExpired()){
    timer.Start(waitingTime);
    val = analogRead(IRPin);
  }
  double voltage = (val/204.8);
  Dist = 95.9972 * pow(0.315141, voltage);
}


//allows for the PID constraints to change
SharpIR::setIRPID(double P, double I, double D){
  Kp = P;
  Ki = I;
  Kd = D;
}

//does PID control for the distance from the wall
//Left Effort and Right Effort are motor speeds
//wallError is how far away from target distance it thinks it is
//distance is how far away from the wall you want it to be 
//base speed is generally how fast it will move forward
float SharpIR::IRPID(float& LeftEffort, float&RightEffort, float& wallError, double distance, double baseSpeed){
  float dist;
  this->Distance(dist);
  wallError = distance - dist;
  wallSum = this->rollingSum(wallError);
  
  double wallChange = lastWallError - wallError;
  lastWallError = wallError;
  
  double Effort = Kp *wallError + Ki * wallSum + Kd * wallChange;
  
    LeftEffort = baseSpeed - Effort;
    RightEffort = baseSpeed + Effort;
  }


//The I term of the PID needed to be limited so this adds up the past ten values 
  double SharpIR::rollingSum(float error){
    sumWall[i] = error;
    i++;
    if (i >= 10){
     i = 0;
    }
    double sumWallError = sumWall[0] + sumWall[1] + sumWall[2] + sumWall[3] + sumWall[4] +
     sumWall[5] + sumWall[6] + sumWall[7] + sumWall[8] + sumWall[9];
  return sumWallError;
}
