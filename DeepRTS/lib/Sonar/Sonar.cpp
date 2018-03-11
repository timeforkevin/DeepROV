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

void measure_sonars(double y[]) {
  digitalWrite(trig_pins[0], LOW);
  delayMicroseconds(2);

  digitalWrite(trig_pins[0], HIGH);
  delayMicroseconds(10);

  digitalWrite(trig_pins[0], LOW);

  long us = pulseIn(echo_pins[0], HIGH, 26000);
  sonar_outputs[0] = float(us) * SPEED_OF_SOUND * 1E-6;
  y[0] = -sonar_outputs[0];
}
