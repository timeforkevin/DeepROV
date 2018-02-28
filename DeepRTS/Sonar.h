#ifndef SONAR_H
#define SONAR_H

#include "Arduino.h"

#define NUM_SONAR 2

extern float sonar_outputs[NUM_SONAR];

void init_sonars();
void measure_sonars();

#endif
