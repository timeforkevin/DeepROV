#include "Arduino.h"
#include "LeakDetector.h"

LeakDetector::LeakDetector(int input_pin) {
    _input_pin = input_pin;
    pinMode(_input_pin, INPUT);
}

bool LeakDetector::detect() {
    if (digitalRead(_input_pin) == HIGH) return true;
    return false;
}
