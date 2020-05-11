#ifndef _SHARPIR_H
#define _SHARPIR_H

#include "arduino.h"
#include "EventTimer.h"

#define ADC_TO_VOUT (5 / 1024) //2^10 (10 bit microntroller)
#define MAGIC_NUM_ONE 95.9972 //TODO: figure out where these came from
#define MAGIC_NUM_TWO 0.315141

class SharpIR {
private:
    int val = 0;
public:
    EventTimer timer;                 //nonblocking timer for when to read
    uint8_t pin = A6;                   //default IR pin
    unsigned int waitingTime = 50;    //millis between readings

    SharpIR(uint8_t thisPin);
    float getDistance(void);
};


#endif
