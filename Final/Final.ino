#include <Zumo32U4Encoders.h>
#include <Wire.h>
#include <Zumo32U4.h>

#include <button.h>             //include the button class
#include <EventTimer.h>         //include the event timer
#include <SharpIR.h>            //include the IR
#include <filter.h>             //include IMU
#include <SpeedControl.h>       //include Speed PID control
#include <LineFollowing.h>      //include line following

#define KP_IR 0.05
#define KI_IR 0.05
#define KP_IR -6.00

#define KP_GYRO 0.2
#define KI_GYRO 0.0
#define KD_GYRO 0.0

#define KP_MOTORS 18.0;
#define KI_MOTORS 1.5;
#define KD_MOTORS 0.0;

EventTimer timer;         //assumes you named your class EventTimer
SharpIR Sharp;            //sets up IR with default pin A6
SpeedControl Speed;       //creates a speed PID with default constants
LineFollowing line;       //creates a line following with default threshold


Zumo32U4Motors motors;
Zumo32U4Encoders encoders;
Zumo32U4LineSensors lineSensors;
Zumo32U4ProximitySensors proxSensors;
Zumo32U4LCD lcd;

enum DRIVE_STATE {STOP, DRIVE, WALL, TURN1, LINE, TURN2};
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

float lineLeft = 0;       //line following motor values
float lineRight = 0;

void setup()
{
  Serial.begin(115200);
  Serial.println("Hello.");

  Speed.Init();
  filter.Init();                      //initializes the IMU/complementary filter
  lineSensors.initThreeSensors();     //initializes the line sensors
  proxSensors.initFrontSensor();      //initializes front proximity sensor/remote control
  line.Init();
  Sharp.setIRPID(KP_IR, KI_IR, KP_IR);
  filter.setGyroPID(KP_GYRO, KI_GYRO, KD_GYRO);
  speed.setSpeedPID(KP_MOTORS, KI_MOTORS, KD_MOTORS);

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
  line.Update();                                                      //update line sensors and prox sensor
}


void driveState(){

  switch(state)
  {
    case STOP:
      Speed.setTargetSpeeds(0, 0);

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
          Speed.setTargetSpeeds(baseSpeed, baseSpeed);    //just start moving (possiblly execive)
          state = WALL;
        }
        else{
          state = DRIVE;
        }
     break;

     case WALL:
        Speed.setTargetSpeeds(wallLeft, wallRight);     //drive based on wall PID
        if(line.Detect() || proxSensors.readBasicFront()){ //if it detects line or remote (kept making escape attempts)
          state = TURN1;
        }
        else{
          state = WALL;
        }
      break;

      case TURN1:
        filter.GyroPID(LeftSpeed, RightSpeed, turnError, Angle);  //read gyro here to initialize bias
        Speed.setTargetSpeeds(LeftSpeed,RightSpeed);                              //drive based on gyro PID

        if (filter.doneTurning()){ //use a range to stop turning
          if(line.Align(lineLeft, lineRight)){
            state = LINE;
          }
            Speed.setTargetSpeeds(lineLeft, lineRight);
        }else{
          state = TURN1;
        }
       break;

       case LINE:
         line.LinePID(lineLeft, lineRight, baseSpeed - 20);
         Speed.setTargetSpeeds(lineLeft, lineRight);
         if (proxSensors.readBasicFront()){
          state = TURN2;
         }
         else{
          state = LINE;
         }
       break;

      case TURN2:
        filter.GyroPID(LeftSpeed, RightSpeed, turnError, 270);  //read gyro here to initialize bias
        Speed.setTargetSpeeds(LeftSpeed,RightSpeed);                              //drive based on gyro PID

        if (filter.doneTurning()){ //use a range to stop turning
          state = STOP;
        }
        else{
          state = TURN2;
        }
      break;

  }
}


// //sets the target speads for the speed PID and updates line sensors
// void Drive(int Left, int Right){
//       targetLeft = Left;
//       targetRight = Right;
// }


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
