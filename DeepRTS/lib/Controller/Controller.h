
#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "kalman_filter_initialize.h"
#include "Motor.h"

#define MANUAL_STATES 4

typedef enum ControlMode {
  FullState    = 0x00,
  ManZVel      = 0x01,
  ManRollTrim  = 0x02,
  ManPitchTrim = 0x04,
  ManYawVel    = 0x08,
  ManXVel      = 0x10
} ControlMode;

extern double KLQR[NUM_MOTORS*NUM_STATES];
extern double target[NUM_STATES];

extern int   man_x_vel;
extern double man_z_vel;
extern double man_y_vel;
extern int   man_r_trim;
extern int   man_p_trim;
extern double pitch_factor;
extern unsigned int dlqr_mode;

void controller(const double mu[NUM_STATES],
                int u[NUM_MOTORS]);
#endif
