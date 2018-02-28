#include "Arduino.h"
#include "Sonar.h"

// In Water [m/s]
#define SPEED_OF_SOUND 1500

const unsigned int echo_pins[NUM_SONAR] = {1, A1};
const unsigned int trig_pins[NUM_SONAR] = {2, A2};
float sonar_outputs[NUM_SONAR];

void init_sonars() {
  for (int i = 0; i < NUM_SONAR; i++) {
    pinMode(trig_pins[i], OUTPUT);
    pinMode(echo_pins[i], INPUT);
  }
}

void measure_sonars() {
  for (int i = 0; i < NUM_SONAR; i++) {
    
    digitalWrite(trig_pins[i], LOW);
    delayMicroseconds(2);

    digitalWrite(trig_pins[i], HIGH);
    delayMicroseconds(10);

    digitalWrite(trig_pins[i], LOW);

    long us = pulseIn(echo_pins[i], HIGH, 26000);
    sonar_outputs[i] = float(us) * SPEED_OF_SOUND * 1E-6;
    delay(50);
  }
}
