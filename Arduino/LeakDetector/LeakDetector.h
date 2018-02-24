#include "Arduino.h"

class LeakDetector {
public:
    LeakDetector(int input_pin);
    bool detect();
private:
    int _input_pin;
};
