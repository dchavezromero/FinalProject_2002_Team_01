#pragma once

#include <Arduino.h>

class EventTimer {
private:
  int timer_duration;
  long int start_time;
  bool expired = false;
public:
  EventTimer();

  void Start(int duration);
  bool CheckExpired();
  void Cancel();
};
