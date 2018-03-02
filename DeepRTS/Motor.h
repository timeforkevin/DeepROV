
#ifndef MOTOR_H
#define MOTOR_H

#include "Arduino.h"

#define POWER_MAX 100
#define POWER_MIN -100
#define NUM_MOTORS 5
#define MIN(X,Y) (((X) < (Y)) ? (X) : (Y))
#define MAX(X,Y) (((X) > (Y)) ? (X) : (Y))

extern int motor_power[NUM_MOTORS];

void init_motors();
void set_motors();
void set_motors_raw(long *pwms);

// typedef enum MOTORS {
//   T100,
//   BFM
// } MOTORTYPE;

#endif
