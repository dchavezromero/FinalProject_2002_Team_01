#include <Zumo32U4Encoders.h>
#include <Wire.h>
#include <Zumo32U4.h>

#include <button.h>             //include the button class 
#include <EventTimer.h>         //include the event timer
#include <SharpIR.h>            //include the IR
#include <filter.h>             //include IMU
#include <SpeedControl.h>       //include Speed PID control

EventTimer timer;         //assumes you named your class EventTimer
SharpIR Sharp;            //sets up IR with default pin A6
SpeedControl Speed;       //creates a speed PID with default constants 

Zumo32U4Motors motors;
Zumo32U4Encoders encoders;   
Zumo32U4LineSensors lineSensors;
Zumo32U4ProximitySensors proxSensors;
Zumo32U4LCD lcd;

enum DRIVE_STATE {STOP, DRIVE, WALL, SPINNING};
  DRIVE_STATE state = STOP;

volatile uint8_t readyToPID = 0;   //a flag that is set when the PID timer overflows
volatile int16_t countsLeft = 0;   //left encoder counts from ISR
volatile int16_t countsRight = 0;  //right encoder counts from ISR
int targetLeft = 0;         //left speed to try to reach with speed PID
int targetRight = 0;        //right speed to try to reach with speed PID

int readingLeft = 0;      //Line sensor 1
int readingCenter = 0;    //Line sensor 3
int readingRight = 0;     //Line sensor 5

ComplementaryFilter filter;
float gyro;               //the reading from gyro in CalcAngle
float prediction;         //the complementary filter angle in CalcAngle
//yaw = 0, pitch = 1, roll = 2
int axis = 0;             //determines what axis is being measures in CalcAngle

float turnError = 90;     //turn error in gyroPID
double Angle = 90;        //angle to turn to for gyroPID
float LeftSpeed = 0;      //Left motor speed for gyroPID
float RightSpeed = 0;     //right motor speed for gyroPID

float wallLeft = 0;       //left motor speed for IRPID
float wallRight = 0;      //right motor speed for IRPID
float wallError = 0;      //the error in IRPID
double baseSpeed = 30;    //speed with variable changes for IRPID
double TargetDist = 30;   //how far away from the wall for IRPID

uint16_t lineSensorValues[3] = { 0, 0, 0 };

void setup()
{
  Serial.begin(115200);
  Serial.println("Hello.");

  Speed.Init();
  filter.Init();                      //initializes the IMU/complementary filter
  lineSensors.initThreeSensors();     //initializes the line sensors
  proxSensors.initFrontSensor();      //initializes front proximity sensor/remote control
}



void loop() {  
  if(readyToPID){
    //clear the timer flag
    readyToPID = 0;
    Speed.speedPID(countsLeft, countsRight, targetLeft, targetRight); //drive/update speedPID
  }
   
  Sharp.IRPID(wallLeft, wallRight, wallError, TargetDist, baseSpeed); //update wall PID
  driveState();                                                       //state machine
  filter.CalcAngle(gyro, prediction, axis);                           //update IMU
  lineSensors.read(lineSensorValues);                                 //update line sensor values
}


void driveState(){
 
  switch(state)
  {
    case STOP:
      Drive(0, 0);
        if(proxSensors.readBasicFront()){ //wait for remote control
          timer.Start(1000);              //wait to get out of the way (more important with button)
          state = DRIVE;
        }
        else{
          state = STOP;
     }
    break;

    case DRIVE:
        if(timer.CheckExpired()){         //dont go until timer is done
          Drive(baseSpeed, baseSpeed);    //just start moving (possiblly execive)
          state = WALL;
        }
        else{
          state = DRIVE;
        }
     break;

     case WALL:
          Drive(wallLeft, wallRight);     //drive based on wall PID
          if((readingLeft > 1000 && readingCenter > 1000 && readingRight  >1000) || 
          proxSensors.readBasicFront()){ //if it detects line or remote (kept making escape attempts)
            state = SPINNING;
          }
          else{
            state = WALL;
          }
      break;

      case SPINNING:
      filter.GyroPID(LeftSpeed, RightSpeed, turnError, Angle);  //read gyro here to initialize bias
      Drive(LeftSpeed,RightSpeed);                              //drive based on gyro PID
          
          if (filter.doneTurning()){ //use a range to stop turning 
            state = STOP;      //STOP uses speed PID to 0, so turns a bit extra
          }
          else{
            state = SPINNING;
          }
       break;
    
  }
}


//sets the target speads for the speed PID and updates line sensors
void Drive(int Left, int Right){  
      targetLeft = Left;
      targetRight = Right;
     
      readingLeft = lineSensorValues[0];
      readingCenter = lineSensorValues[1];
      readingRight = lineSensorValues[2];   
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
