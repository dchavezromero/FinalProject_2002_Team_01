#include "filter.h"
#include "arduino.h"
#include <Wire.h>
#include <Zumo32U4.h>


//initializes sensors
ComplementaryFilter::Init(void) {
  Wire.begin();

  if (!accel.init())
  {
    // Failed to detect the accel.
    ledRed(1);
    while (1)
    {
      Serial.println(F("Failed to detect the accel."));
      delay(100);
    }
  }

  accel.enableDefault();

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

  accel.enableDefault();
  gyro.enableDefault();

  //while(!Serial){} //wait for the Serial Monitor
  accel.writeReg(LSM303::CTRL1, 0x67);
  gyro.writeReg(LSM303::CTRL1, 0xBF);
  uint8_t ctrl4 = gyro.readReg(LSM303::CTRL1);
}

/*
  determines angle with a complementary filter
  gyroAngle is the angle based on the gyro alone
  estimate angle is the angle through the filter
  axis is what axis its reading (0 for yaw, 1 for pitch, 2 for roll)
*/
bool ComplementaryFilter::CalcAngle(float& gyroAngle, float& estimateAngle) {
    accel.read();
    gyro.read();

    /*if (buttonA.getSingleDebouncedPress()) { //resets what is considered 0 degrees
    i = 1;
    accYoffset = 0;
    accZoffset = 0;
    accXoffset = 0;
    }*/

    uint16_t StatusA = accel.readReg(LSM303::STATUS_A);
    int Ready = bitRead(StatusA, 3);

    //If we haven't iterated 200 times,
    if (i < 200) {
        accYoffset = ((accYoffset * (i - 1))  + accel.a.y) / i;
        i++;
    }

    observedAngle = atan2(-(accel.a.y - accYoffset), accel.a.x) * RAD_TO_DEG;
    gyro_dps = gyro.g.z / READING_TO_DPS;


    dt = (millis() - LastMillis) / 1000; //dT in seconds
    LastMillis = millis(); //update last time
    this->prediction +=  dt * (gyro_dps - gyroBias);
    gyroAngle += dt * gyro_dps;

    gyroBias += E * (prediction - observedAngle);

    estimateAngle = k * (prediction) + (1 - k) * observedAngle;

    currentAngle = estimateAngle;

    //TODO: Figure out what Dom meant to do here
    return (Ready && 1);
}

/*
   reads the sensors
*/
ComplementaryFilter::Read(void) {
  accel.read();
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
  double dt = millis() - lastMillis;
  lastMillis = millis();

  if(dt > 20){
    dt = 10.0;
  }

  if (!initialized) {
    gyroInitial = dt * gyro.g.z / READING_TO_DPS + LastAngle; //initializes the value to get rid of bias built up
    initialized = true;
  }

  double gyroAngle = LastAngle + dt * gyro.g.z / READING_TO_DPS;
  LastAngle = gyroAngle;
  turnError = angle - (gyroAngle - gyroInitial);
  turnSum += turnError;
  double turnChange = lastTurnError - turnError;
  lastTurnError = turnError;

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
bool ComplementaryFilter::doneTurning() {
  return doneTurn;
}

float ComplementaryFilter::getCurrentAngle() {
    return currentAngle;
}
