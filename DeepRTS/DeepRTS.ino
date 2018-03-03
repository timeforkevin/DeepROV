
#include "Motor.h"
#include "Sonar.h"
#include "LeakDetector.h"


#include "IMU.h"

void setup() {
  // put your setup code here, to run once:
  init_motors();
  init_sonars();
  init_leak_detector();
  init_MPU9250();
  init_LSM9DS0();
}

void loop() {
  // put your main code here, to run repeatedly:
  if (leaky()) {
    // Do the squeaky
    // Can't take this L
  }
  motor_power[0] = 10;
  set_motors();
  measure_sonars();
}
