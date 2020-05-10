#include "SpeedControl.h"
#include "arduino.h"
#include <Zumo32U4Motors.h>

  SpeedControl::SpeedControl(float P, float I, float D){
  Kp = P;
  Ki = I;
  Kd = D;
 }

 SpeedControl::SpeedControl(){
  Kp = 18;
  Ki = 1.5;
  Kd = 0;
 }

 SpeedControl::Init(){
  noInterrupts();
  TCCR4A = 0x00; //disable some functionality -- no need to worry about this
  TCCR4B = 0x0C; //sets the prescaler -- look in the handout for values
  TCCR4C = 0x04; //toggles pin 6 at one-half the timer frequency
  TCCR4D = 0x00; //normal mode

  OCR4C = 0X6C;  //TOP goes in OCR4C //I picked 107?
  TIMSK4 = 0x04; //enable overflow interrupt
  interrupts();
 }



 SpeedControl::speedPID(int16_t countsLeft, int16_t countsRight,int targetLeft,int targetRight){
  noInterrupts();
  speedLeft = countsLeft - prevLeft;
  speedRight = countsRight - prevRight;

  prevLeft = countsLeft;
  prevRight = countsRight;
  interrupts();

  errorLeft = targetLeft - speedLeft;
  sumLeft += errorLeft;
  errorRight = targetRight - speedRight;
  sumRight += errorRight;

  if(sumLeft>= 200){
     sumLeft = 200;
     sumRight = 200;
  }

  float effortLeft = Kp * errorLeft + Ki * sumLeft;
  float effortRight = Kp * errorRight + Ki * sumRight;

  motors.setSpeeds(effortLeft, effortRight); //up to you to add the right motor

 }

SpeedControl::setTargetSpeeds(int targetLeft, int targetRight) {
    targetLeftSpeed = targetLeft;
    targetRightSpeed = targetRight;
}
