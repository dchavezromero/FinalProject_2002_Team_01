#pragma once

#include <Arduino.h>

class Button {
private:
  enum ButtonState {WAIT_FOR_PRESS, WAIT_FOR_TIME};

  int pin;
  int timer = 10; //Default timer of 10ms

  bool pressed = false;

  bool tempState = false;
  long int lastUpdated;

  enum ButtonState buttonState = WAIT_FOR_PRESS;

public:
  Button(int pin);
  void Init();

  bool CheckButtonPress();
  void UpdateState(); //Called at the start of CheckButtonPress so that extra Update calls are not necessary
};
