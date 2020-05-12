#include "button.h"

Button::Button(int pin) {
  this->pin = pin;
}


/*
 * Initialize the button by setting the pin mode to input, with a pullup resistor
*/
void Button::Init() {
  pinMode(pin, INPUT_PULLUP);
}


/*
 * Check whether a button press event has occured. Will only return true once
 * @returns whether a single button event has occured
*/
bool Button::CheckButtonPress() {
  UpdateState();

  if(buttonState == WAIT_FOR_PRESS) {
      bool temp = pressed;
      pressed = false;
      return temp;
  }

  return false;
}


/*
 * State machine for debouncing. Will wait and make sure that the button has settled
 * before signaling a button presse event.
*/
void Button::UpdateState() {
    //If we are currently waiting for a button press
    if(buttonState == WAIT_FOR_PRESS) {
        //Read the current state of the button (invert the read as pressed is LOW)
        bool curState = !digitalRead(pin);

        //If we detect a button press, and the button wasn't down on the last cycle
        if(curState && !tempState) {
            //Migrate to the WAIT_FOR_TIME state
            buttonState = WAIT_FOR_TIME;

            //Start the timer
            lastUpdated = millis();
        }

        //Update the tempState to ensure we don't count a single press multiple times (and later for debouncing)
        tempState = curState;
    }
    //Otherwise, we are waiting for 10ms of no bounces
    else if(buttonState == WAIT_FOR_TIME) {
        //Read the current state of the button (invert the read as pressed is LOW)
        bool curState = !digitalRead(pin);

        //If it has been 10ms since lastUpdatd
        if((millis() - lastUpdated) > timer) {
            //Switch back to waiting for a press
            buttonState = WAIT_FOR_PRESS;

            //Update the `pressed` variable with the final value of the button
            pressed = curState;
        }
        //Otherwise, we are not done waiting
        else {
            //If we have bounced
            if(curState != tempState) {
                //Reset the timer
                lastUpdated = millis();

                //Reset the temporary state
                tempState = curState;
            }
        }
    }
}
