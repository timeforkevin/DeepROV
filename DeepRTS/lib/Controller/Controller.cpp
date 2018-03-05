
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

double manual[NUM_STATES] =
{
  0,
  0,
  0,
  0
};

int    target_vel = 0;
// Pitch Factor makes pitch more stable when diving
// due to three motors controlling pitch
double pitch_factor = 1.85;

unsigned int dlqr_mode = FullState;

void dlqr(const double mu[NUM_STATES],
          int u[NUM_MOTORS]) {
  // Check dlqr_mode for manual controls
  for (int i = 0; i < NUM_STATES; i++) {
    if ((dlqr_mode >> i) & 0x01) {
      target[i] = mu[i] + manual[i];
    }
  }

  // Matrix Multiplication of DLQR K matrix with dmu
  for (int i = 0; i < NUM_MOTORS; i++) {
    double u_f = 0;
    for (int j = 0; j < NUM_STATES; j++) {
      // Left Shift trick for *4
      u_f += KLQR[(i << 2) + j] * (target[j] - mu[j]);
    }
    u[i] = round(u_f);
  }

  // Fancy Non-Linear Saturation Logic
  if (u[3] > POWER_MAX || u[3] < POWER_MIN) {
    u[2] /= fabs(u[3]);
    u[4] /= fabs(u[3]);
  }
  u[2] /= pitch_factor;
  u[4] /= pitch_factor;
  for (int i = 0; i < NUM_MOTORS; i++) {
    u[i] = MAX(MIN(u[i],POWER_MAX),POWER_MIN);
  }
}

void vel_control(int u[NUM_MOTORS]) {
  // Linear Velocity
  int diff = u[0] - u[1];
  if ((target_vel + abs(diff) / 2) > POWER_MAX) {
    u[0] += POWER_MAX - abs(diff) / 2;
    u[1] += POWER_MAX - abs(diff) / 2;
  } else if ((-target_vel - abs(diff) / 2) < POWER_MIN) {
    u[0] += POWER_MIN + abs(diff) / 2;
    u[1] += POWER_MIN + abs(diff) / 2;
  } else {
    u[0] += target_vel;
    u[1] += target_vel;
  }
}