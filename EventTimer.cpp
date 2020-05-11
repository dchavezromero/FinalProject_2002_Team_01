#include "EventTimer.h"

EventTimer::EventTimer() {}

void EventTimer::Start(int duration) {
  timer_duration = duration;

  start_time = millis();

  expired = false;
}

bool EventTimer::CheckExpired() {
  if(((millis() - start_time) > timer_duration) || expired) {
    expired = true;
    return expired;
  }

  return false;
}

void EventTimer::Cancel() {
  this->expired = true;
}
