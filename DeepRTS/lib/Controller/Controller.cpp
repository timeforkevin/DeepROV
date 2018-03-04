
#include "Controller.h"

#include <math.h>
#include "kalman_filter_initialize.h"
#include "Motor.h"

// From MATLAB DeepRTS_DLQR.m
double KLQR[NUM_MOTORS*NUM_STATES] =
{
  0.0000,  0.0000,  0.0000, -0.2225,
  0.0000,  0.0000,  0.0000,  0.2225,
  0.1677, -0.2217, -0.1354,  0.0000,
  0.1843, -0.0000,  0.2488,  0.0000,
  0.1677,  0.2217, -0.1354,  0.0000
};

double target[NUM_STATES] =
{
  -36,
  0,
  0,
  0
};

void dlqr(ControlMode mode,
          const double mu[NUM_STATES],
          int u[NUM_MOTORS]) {
  if (mode & FreeInZ) {
    target[0] = mu[0];
  }
  if (mode & FreeInYaw) {
    target[3] = mu[3];
  }

  for (int i = 0; i < NUM_MOTORS; i++) {
    double u_f = 0;
    for (int j = 0; j < NUM_STATES; j++) {
      // Left Shift trick for *4
      u_f += KLQR[(i << 2) + j] * (target[j] - mu[j]);
    }
    u[i] = round(u_f);
  }
}