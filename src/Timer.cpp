#include "Arduino.h"
#include <math.h>
#include "Timer.h"

Timer::Timer(unsigned int ms)
{
    _ms = ms;
}

bool Timer::checkT(){
    _currentTime = millis();
    _elapsedTime = (double)(_currentTime - _previousTime);
    if(_currentTime - _previousTime >= _ms){
      _previousTime = millis();
      return true;
    }
    else{
      return false;
    }

}
