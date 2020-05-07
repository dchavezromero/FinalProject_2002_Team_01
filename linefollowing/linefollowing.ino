#include "LineFollowing.h"
#include <SpeedControl.h>       //include Speed PID control
#include <Zumo32U4.h>
Zumo32U4Encoders encoders;  

SpeedControl Speed;       //creates a speed PID with default constants 
LineFollowing line;
volatile uint8_t readyToPID = 0;   //a flag that is set when the PID timer overflows
volatile int16_t countsLeft = 0;   //left encoder counts from ISR
volatile int16_t countsRight = 0;  //right encoder counts from ISR
int targetLeft = 0;         //left speed to try to reach with speed PID
int targetRight = 0;        //right speed to try to reach with speed 
double basespeed = 10;

float leftLine;       //Line follower left motor effort
float rightLine;      //Line follower right motor effort
bool Align = false;   //flag to tell if it has been aligned before going

void setup() {
  line.Init();
  Speed.Init();
  
  
}

void loop() {
  if(readyToPID){
    //clear the timer flag
    readyToPID = 0;
    Speed.speedPID(countsLeft, countsRight, targetLeft, targetRight); //drive/update speedPID
  }
  line.Update();
//  if (!Align){
    line.AlignAlong(leftLine, rightLine);
//  }
//  else{
//    line.LinePID(leftLine, rightLine, basespeed);
//  }
  
  Drive(leftLine, rightLine);

//  Serial.print(line.lineSensorValues[0]);
//  Serial.print("\t");
//  Serial.print(line.DetectLeft);
//  Serial.print("\t");
//  Serial.print(line.lineSensorValues[1]);
//  Serial.print("\t");
//  Serial.print(line.DetectCenter);
//  Serial.print("\t");
//  Serial.print(line.lineSensorValues[2]);
//  Serial.print("\t");
//  Serial.print(line.DetectRight);
//  Serial.print("\n");
  
}



void Drive(int Left, int Right){  
      targetLeft = Left;
      targetRight = Right;
}



/*
 * ISR for timing. Basically, raise a flag on overflow. Timer4 is set up to run with a pre-scaler 
 * of 1024 and TOP is set to 249. Clock is 16 MHz, so interval is dT = (1024 * 250) / 16 MHz = 16 ms.
 */
ISR(TIMER4_OVF_vect)
{
  //Capture a "snapshot" of the encoder counts for later processing
  countsLeft = encoders.getCountsLeft();
  countsRight = encoders.getCountsRight();

  readyToPID = 1;
}
