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
    if(_elapsedTime >= _ms){
      _previousTime = _currentTime;
      return true;
    }
    else{
      return false;
    }
}
