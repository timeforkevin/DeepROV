#include "Arduino.h"
#include "Sonar.h"

// constructor
Sonar::Sonar(int trig_pin, int echo_pin)
{
    _trig_pin = trig_pin;
    pinMode(_trig_pin, OUTPUT);
    _echo_pin = echo_pin;
    pinMode(_echo_pin, INPUT);
}

long Sonar::measure(void)
{
    long distance;
    digitalWrite(_trig_pin, LOW);
    delayMicroseconds(2);

    digitalWrite(_trig_pin, HIGH);
    delayMicroseconds(10);

    digitalWrite(_trig_pin, LOW);

    distance = pulseIn(_echo_pin, HIGH,26000);
    delay(50);
    return distance/58;
}
