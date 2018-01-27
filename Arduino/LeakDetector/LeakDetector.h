#include "Arduino.h"

class LeakDetector {
public:
    LeakDetector(int input_pins[], int num_pins);
    bool detect();
private:
    int _pins[];
    int _num_pins;
};
