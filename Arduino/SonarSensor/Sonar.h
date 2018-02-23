#ifndef Sonar_h
#define Sonar_h

#include "Arduino.h"

class Sonar
{
    public:
        Sonar(int trig_pin, int echo_pin);
        long measure(void);
    private:
        int _trig_pin;//pin number of Arduino that is connected with trig pin of Sonar Ranger.
        int _echo_pin;//pin number that is connected to echo
};

#endif
