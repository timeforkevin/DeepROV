
#include "Controller.h"

#include <math.h>
#include "kalman_filter_initialize.h"
#include "Motor.h"

// From MATLAB DeepRTS_DLQR.m
double KLQR[NUM_MOTORS*NUM_STATES] =
{
  0.0000,  0.0000,  0.0000, -0.2191,  0.0000,  0.0000,  0.0000, -0.9127,
  0.0000,  0.0000,  0.0000,  0.2191,  0.0000,  0.0000,  0.0000,  0.9127,
  0.1395,  0.2167,  0.1509,  0.0000,  0.2713,  0.7886,  0.5820,  0.0000,
  0.2052,  0.0000, -0.2202,  0.0000,  0.3087,  0.0000, -0.8751,  0.0000,
  0.1395, -0.2167,  0.1509,  0.0000,  0.2713, -0.7886,  0.5820,  0.0000
};

double target[NUM_STATES] =
{
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0
};

double threshold[NUM_STATES] =
{
  4,
  10*DEG_TO_RAD,
  10*DEG_TO_RAD,
  20*DEG_TO_RAD,
  0.8,
  2*DEG_TO_RAD,
  2*DEG_TO_RAD,
  2*DEG_TO_RAD
};

int   man_x_vel = 0;
double man_z_vel = 0;
double man_y_vel = 0;
int   man_r_trim = 0;
int   man_p_trim = 0;
// Pitch Factor makes pitch more stable when diving/rising
// due to three motors controlling pitch
double pitch_factor = 1.35;

unsigned int dlqr_mode = FullState;

void dlqr(const double mu[NUM_STATES],
          int u[NUM_MOTORS]);
void man_control();
void vel_control(int u[NUM_MOTORS]);


void controller(const double mu[NUM_STATES],
                int u[NUM_MOTORS]) {
  // Check dlqr_mode for manual controls
  man_control();
  dlqr(mu, u);
  vel_control(motor_power);
}

void dlqr(const double mu[NUM_STATES],
          int u[NUM_MOTORS]) {
  double u_f[NUM_MOTORS] = {0, 0, 0, 0, 0};
  // Matrix Multiplication of DLQR K matrix with dmu
  for (int i = 0; i < NUM_MOTORS; i++) {
    for (int j = 0; j < NUM_STATES; j++) {
      double dmu_j = target[j] - mu[j];
      // If yaw
      if (j == 3 || j == 7) {
        if (dmu_j > M_PI) {
          dmu_j -= 2*M_PI;
        }
        if (dmu_j < -M_PI) {
          dmu_j += 2*M_PI;
        }
      }

      // Thresholding trick sacrifices a little stability
      // for a lot of power saving. Simulation shows minimal
      // loss in stability especially with motor control delay.
      dmu_j = (fabs(dmu_j) > threshold[j]) ? dmu_j : 0;

      // Left Shift trick for i*8
      u_f[i] += KLQR[(i << 3) + j] * dmu_j * 1.5;
    }
  }

  // Fancy Non-Linear Saturation Logic
  if (u_f[3] > 1.0 || u_f[3] < -1.0) {
    u_f[2] /= fabs(u_f[3]);
    u_f[4] /= fabs(u_f[3]);
  }
  // Pitch Factor makes pitch more stable when diving/rising
  // due to three motors controlling pitch
  u_f[2] /= pitch_factor;
  u_f[4] /= pitch_factor;
  for (int i = 0; i < NUM_MOTORS; i++) {
    u[i] = round(u_f[i] * 100);
    u[i] = MAX(MIN(u[i],POWER_MAX),POWER_MIN);
  }
}

void man_control() {
  if (dlqr_mode & ManZVel) {
    // Relinquish control on position state in Z
    // Set target velocity states to manual control
    target[0] = mu[0];
    target[4] = man_z_vel;
  } else {
    // Reset Velocity Command
    target[4] = 0;
  }
  if (dlqr_mode & ManRollTrim) {
    // Trim Roll by given number of degrees
    target[1] += man_r_trim*DEG_TO_RAD;
    man_r_trim = 0;
  }
  if (dlqr_mode & ManPitchTrim) {
    // Trim Roll by given number of degrees
    target[2] += man_p_trim*DEG_TO_RAD;
    man_p_trim = 0;
  }
  // Relinquish control on position state in Yaw
  target[3] = mu[3];
  if (dlqr_mode & ManYawVel) {
    // Set target velocity states to manual control
    target[7] = man_y_vel;
  } else {
    // Reset Velocity Command
    target[7] = 0;
  }
}

void vel_control(int u[NUM_MOTORS]) {
  // Linear Velocity Control if requested
  if (dlqr_mode & ManXVel) {
    int diff = u[0] - u[1];
    if ((man_x_vel + abs(diff) / 2) > POWER_MAX) {
      u[0] += POWER_MAX - abs(diff) / 2;
      u[1] += POWER_MAX - abs(diff) / 2;
    } else if ((-man_x_vel - abs(diff) / 2) < POWER_MIN) {
      u[0] += POWER_MIN + abs(diff) / 2;
      u[1] += POWER_MIN + abs(diff) / 2;
    } else {
      u[0] += man_x_vel;
      u[1] += man_x_vel;
    }
  }
}