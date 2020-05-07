#include "filter.h"
#include "arduino.h"
#include <Wire.h>
#include <Zumo32U4.h>


//initializes sensors
ComplementaryFilter::Init(void) {
  Wire.begin();

  if (!compass.init())
  {
    // Failed to detect the compass.
    ledRed(1);
    while (1)
    {
      Serial.println(F("Failed to detect the compass."));
      delay(100);
    }
  }

  compass.enableDefault();

  if (!gyro.init())
  {
    // Failed to detect the gyro.
    ledRed(1);
    while (1)
    {
      Serial.println(F("Failed to detect gyro."));
      delay(100);
    }
  }

  compass.enableDefault();
  gyro.enableDefault();

  //while(!Serial){} //wait for the Serial Monitor
  compass.writeReg(LSM303::CTRL1, 0x67);
  gyro.writeReg(LSM303::CTRL1, 0xBF);
  uint8_t ctrl4 = gyro.readReg(LSM303::CTRL1);
}

/*
  determines angle with a complementary filter
  gyroAngle is the angle based on the gyro alone
  estimate angle is the angle through the filter
  axis is what axis its reading (0 for yaw, 1 for pitch, 2 for roll)
*/
bool ComplementaryFilter::CalcAngle(float& GyroAngle, float& estimateAngle, int axis) {
  this->Read();
  if (buttonA.getSingleDebouncedPress()) { //resets what is considered 0 degrees
    i = 1;
    accYoffset = 0;
    accZoffset = 0;
    accXoffset = 0;
  }
  uint16_t StatusA = compass.readReg(LSM303::STATUS_A);
  int Ready = bitRead(StatusA, 3);

  gyro.read();
  //yaw
  if (axis == 0) {
    if (i < 200) {
      accYoffset = ((accYoffset * (i - 1))  + compass.a.y) / i;
      i++;
    }
    observedAngle = atan2(-(compass.a.y - accYoffset), compass.a.x) * 57.29577;
    Gyro = gyro.g.z / 114285.7;
  }

  //pitch
  if (axis == 1) {
    if (i < 200) {
      accXoffset = ((accXoffset * (i - 1))  + compass.a.x) / i;
      i++;
    }
    observedAngle = atan2(-(compass.a.x - accXoffset), compass.a.z) * 57.29577;
    Gyro = gyro.g.y / 114285.7;
  }

  //roll
  else if (axis == 2) {
    if (i < 200) {
      accZoffset = ((accZoffset * (i - 1))  + compass.a.z) / i;
      i++;
    }
    observedAngle = atan2(-(compass.a.z - accZoffset), compass.a.y) * 57.29577;
    Gyro = gyro.g.x / 114285.7;
  }


  DeltaT = millis() - LastMillis;
  LastMillis = millis();
  this->prediction +=  DeltaT * (Gyro - gyroBias);
  GyroAngle += DeltaT * Gyro;

  gyroBias += E * (prediction - observedAngle);

  estimateAngle = k * (prediction) + (1 - k) * observedAngle;


  return (Ready && 1);
}


/*
   smoothes out the readings a bit more
*/
float ComplementaryFilter::Average(float estimateAngle) {
  float Average[7];
  Average[n] = estimateAngle;
  n++;
  if (n == 7) {
    n = 0;
  }

  return (Average[0] + Average[1] + Average[2] + Average[3] + Average[4] + Average[5] + Average[6]) / 7;
}

/*
   reads the sensors
*/
ComplementaryFilter::Read(void) {
  compass.read();
}

/*
   allows to change the PID constants for GyroPID
*/
void ComplementaryFilter::setGyroPID(double P, double I, double D) {
  Kp = P;
  Ki = I;
  Kd = D;
}


/*
 * PID control for turning based on the gyro
 * left effort and right effort are motor speeds
 * turnError is the error between the gyro and angle
 * angle is the amount to turn in degrees
 */
float ComplementaryFilter::GyroPID(float& LeftEffort, float&RightEffort, float& turnError, double angle) {
  doneTurn = false;
  double DeltaT = millis() - lastMillis;
  lastMillis = millis();
 
  if(DeltaT > 20){
    DeltaT = 10.0;
  }

  if (!initialized) {
    gyroInitial = DeltaT * gyro.g.z / 114285.7 + LastAngle; //initializes the value to get rid of bias built up
    initialized = true;
  }
   
  double gyroAngle = LastAngle + DeltaT * gyro.g.z / 114285.7;
  LastAngle = gyroAngle;
  turnError = angle - (gyroAngle - gyroInitial);
  turnSum += turnError;
  double turnChange = lastTurnError - turnError;
  lastTurnError = turnError;

  lcd.clear();
  lcd.println(turnError);
  
  double Effort = Kp * turnError + Ki * turnSum + Kd * turnChange;


  //it doesnt turn in place to try to get the line sensor centered on the line
  //turning left
  if (angle > 0) {
    RightEffort = Effort;
    LeftEffort = Effort / 8;
  }
  //turning right
  else if (angle < 0) {
    RightEffort = Effort / 8;
    LeftEffort = Effort;
  }

  if (turnError < 2) {
    doneTurn = true;
    initialized = false;
  }
}

//returns true when the error is within 2 degrees
boolean ComplementaryFilter::doneTurning() {
  return doneTurn;
}
