#ifndef _BUTTON_H
#define _BUTTON_H

#include <Arduino.h>

class Button {
private:
    enum ButtonState {WAIT_FOR_PRESS, WAIT_FOR_TIME};

    int pin;
    int timer = 10; //How long the button must be settled for before a button event fires

    bool pressed = false;

    bool tempState = false;
    long int lastUpdated;

    enum ButtonState buttonState = WAIT_FOR_PRESS;

    void UpdateState();

public:
    Button(int pin);
    void Init();

    bool CheckButtonPress();
};

#endif
