
#include "Motor.h"
#include "Sonar.h"
#include "LeakDetector.h"
#include "IMU.h"

/* MATLAB Generated Kalman Filter code*/
#include "kalman_filter.h"
#include "kalman_filter_terminate.h"
#include "kalman_filter_initialize.h"

void setup() {
  init_motors();
  init_sonars();
  init_leak_detector();
  // init_MPU9250();
  init_LSM9DS0();
}

void loop() {
  // put your main code here, to run repeatedly:
  if (leaky()) {
    // Do the squeaky
    // Can't take this L
  }

  // Measurements
  measure_sonars();   // How often can we ping?
  measure_LSM9DS0();

  // Calculate State Vector
  float dt = IMU_LSM9DS0.deltat;
  float x[4] = {sonar_outputs[0]*cos(IMU_LSM9DS0.pitch)*cos(IMU_LSM9DS0.roll), 
                IMU_LSM9DS0.roll,
                IMU_LSM9DS0.pitch,
                IMU_LSM9DS0.yaw};

  // Calculate Control Outputs


  // Execute Control Outputs
  set_motors();
}
