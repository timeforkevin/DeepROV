
#ifndef MOTOR_H
#define MOTOR_H

#include "Arduino.h"

#define POWER_MAX 100
#define POWER_MIN -100
#define NUM_MOTORS 5
#define RAMP_FACTOR 5
#define RAMP_THRESHOLD 10
#define MIN(X,Y) (((X) < (Y)) ? (X) : (Y))
#define MAX(X,Y) (((X) > (Y)) ? (X) : (Y))

extern int motor_power[NUM_MOTORS];
extern double droop_factor;

void init_motors();
void set_motors();
void ramp_motors();
void set_motors_raw(long *pwms);
int pwm(int power, int i);

// typedef enum MOTORS {
//   T100,
//   BFM
// } MOTORTYPE;

#endif
