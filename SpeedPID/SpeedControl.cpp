#include "SpeedControl.h"
#include "arduino.h"
#include <Zumo32U4Motors.h>

SpeedControl::SpeedControl(){}

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
