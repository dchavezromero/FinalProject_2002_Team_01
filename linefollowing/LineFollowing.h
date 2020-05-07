  #include "arduino.h"
  #include <EventTimer.h>
  #include <Zumo32U4.h>
  
  #ifndef _LINEFOLLOWING_H
  #define _LINEFOLLOWING_H
  
  class LineFollowing{
    public:

Zumo32U4LineSensors lineSensors;
Zumo32U4ProximitySensors proxSensors;

double Threshold = 1000;                    //light dark threshold
uint16_t lineSensorValues[3] = { 0, 0, 0};  //stores linesensor data      

//stores if each of the line sensors see a line
bool DetectLeft = false;     
bool DetectCenter = false;
bool DetectRight = false;

double line = 0;            //position of the line
double lineSum = 0;         //used for I term 
double sumLine[10];         //used to limit I term
int i = 0;                  //used to rotate through arrays terms 
double lastError = 0;       //stores last error value for D term
bool parallel = false;      //used to switch between positioning error methods
double error = 0;           //initialized value 

double Kp = 2;              //default P constant
double Ki = 0.0003;         //default I constant
double Kd = -.0001;         //default D constant


LineFollowing(double threshold);
LineFollowing();
Init();
Update();
bool DetectIR();
setLinePID(double P, double I, double D);
Detect();
int Position();
int PositionAlongLine();
bool Align(float& LeftEffort, float& RightEffort);
bool AlignAlong(float& LeftEffort, float& RightEffort);
float LinePID(float& LeftEffort, float& RightEffort,double basespeed);
Sum(int error);  
    
    };


  #endif
