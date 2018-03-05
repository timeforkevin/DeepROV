#include "Arduino.h"
#include "LeakDetector.h"

const unsigned int input_pin = 4;

void init_leak_detector() {
  pinMode(input_pin, INPUT);
}

bool leaky() {
  if (digitalRead(input_pin) == HIGH) {
    return true;
  } else {
    return false;
  }
}