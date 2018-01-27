#include "Arduino.h"
#include "LeakDetector.h"

LeakDetector::LeakDetector(int pins[], int num_pins) {
    _num_pins = num_pins;
    for(int i = 0; i < _num_pins; i++) {
        _pins[i] = pins[i];
        pinMode(_pins[i], INPUT);
    }
}

bool LeakDetector::detect() {
    for(int i = 0; i < _num_pins; i++) {
        int leak_state = digitalRead(_pins[i]);
        if (leak_state == HIGH) return true;
        else if (leak_state == LOW) return false;
    }
}
