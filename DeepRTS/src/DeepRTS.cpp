
#include "Motor.h"
#include "Sonar.h"
#include "LeakDetector.h"
#include "IMU.h"
#include "Controller.h"

/* MATLAB Generated Kalman Filter code*/
#include "kalman_filter.h"
#include "kalman_filter_terminate.h"
#include "kalman_filter_initialize.h"

ControlMode mode = FullState;

void setup() {
  init_motors();
  init_sonars();
  init_leak_detector();
  // init_MPU9250();
  init_LSM9DS0();
  kalman_filter_initialize();
}

void loop() {
  // put your main code here, to run repeatedly:
  if (leaky()) {
    // Do the squeaky
    // Can't take this L
  }

  // Measurement Vector
  measure_sonars(y);   // How often can we ping?
  measure_LSM9DS0(y);

  // Calculate State Vector
  kalman_filter(mu, cov, y, C, Q_est, R_est,
                mu, cov);

  // Calculate Control Outputs
  dlqr(mode, mu, motor_power);

  // Execute Control Outputs
  set_motors();
}
