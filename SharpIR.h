#ifndef _SHARPIR_H
#define _SHARPIR_H

#include "arduino.h"
#include "EventTimer.h"
#include "Robot.h"

#define ADC_TO_VOUT (5.0 / 1024.0) //2^10 (10 bit microntroller)
#define MAGIC_NUM_ONE 95.9972 //TODO: figure out where these came from
#define MAGIC_NUM_TWO 0.315141

class SharpIR {
private:
    int val = 0;
    int MaxIRIndex = 10;
    int IRIndex = 0;
    float averageIR[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    float average = 0;
public:
    EventTimer *timer;                 //nonblocking timer for when to read
    uint8_t pin = A6;                   //default IR pin
    unsigned int waitingTime = 15;    //millis between readings

    SharpIR(uint8_t thisPin);
    float getDistance(void);
    float AverageDistance(void);
};


#endif
