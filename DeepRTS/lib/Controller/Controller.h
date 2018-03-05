
#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "kalman_filter_initialize.h"
#include "Motor.h"

typedef enum ControlMode {
  FullState   = 0x00,
  ManualZ     = 0x01,
  ManualRoll  = 0x02,
  ManualPitch = 0x04,
  ManualYaw   = 0x08
} ControlMode;

extern double KLQR[NUM_MOTORS*NUM_STATES];
extern double target[NUM_STATES];
extern double manual[NUM_STATES];
extern int    target_vel;
extern double pitch_factor;
extern unsigned int dlqr_mode;

void dlqr(const double mu[NUM_STATES],
          int u[NUM_MOTORS]);
void vel_control(int u[NUM_MOTORS]);
#endif
