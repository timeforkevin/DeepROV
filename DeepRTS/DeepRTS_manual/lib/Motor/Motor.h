
#ifndef MOTOR_H
#define MOTOR_H

#include "Arduino.h"

#define POWER_MAX 30
#define POWER_MIN -30
#define NUM_MOTORS 5
#define MIN(X,Y) (((X) < (Y)) ? (X) : (Y))
#define MAX(X,Y) (((X) > (Y)) ? (X) : (Y))

extern int motor_power[NUM_MOTORS];
extern double droop_factor;

void init_motors();
void set_motors();
void set_motors_raw(long *pwms);
// double calc_droop();

// typedef enum MOTORS {
//   T100,
//   BFM
// } MOTORTYPE;

#endif
