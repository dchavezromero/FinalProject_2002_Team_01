#include "SharpIR.h"
#include "arduino.h"

//can change what pin it reads
SharpIR::SharpIR(uint8_t thisPin = A6){
  pin = thisPin;
  pinMode(pin, INPUT);
  timer = new EventTimer();
}

//converts the IR reading into a distance
//most accurate around 18 cm and drifts a little around 10 cm from there
float SharpIR::getDistance(void){
    if (timer->CheckExpired()){
        timer->Start(waitingTime);
        val = analogRead(pin);

        Robot::getRobot()->readyToWallPID = true;
    }

    double voltage = (val * ADC_TO_VOUT);
    return (MAGIC_NUM_ONE * pow(MAGIC_NUM_TWO, voltage));
}
