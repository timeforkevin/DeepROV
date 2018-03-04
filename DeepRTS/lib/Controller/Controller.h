
#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "kalman_filter_initialize.h"
#include "Motor.h"

typedef enum ControlMode {
  FullState = 0x00,
  FreeInZ   = 0x01,
  FreeInYaw = 0x02
} ControlMode;

extern double KLQR[NUM_MOTORS*NUM_STATES];

extern double target[NUM_STATES];
extern double dmu[NUM_STATES];

void dlqr(ControlMode mode,
          const double mu[NUM_STATES],
          int u[NUM_MOTORS]);

#endif
