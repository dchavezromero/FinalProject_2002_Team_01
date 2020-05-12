#include "EventTimer.h"

EventTimer::EventTimer() {}


/*
 * Starts the timer for a duration.
 * @param duration the number of milliseconds to run the timer for
*/
void EventTimer::Start(int duration) {
  timer_duration = duration;

  start_time = millis();

  expired = false;
}


/*
 * Check whether or not the timer is finished. Should be called as fast as possible
 * otherwise the timer might "expire" later than it should.
 * @returns whether or not the timer is finished
*/
bool EventTimer::CheckExpired() {
    //If time elapsed is longer than the duration, or we have expired
    if(((millis() - start_time) > timer_duration) || expired) {
        //Set and return the expired flag
        expired = true;
        return expired;
    }

    return false;
}

/*
 * Manually cancel the timer. Simply sets the expired flag to true
*/
void EventTimer::Cancel() {
  this->expired = true;
}
