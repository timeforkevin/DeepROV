
#include "Motor.h"
#include "Sonar.cpp"
#include "LeakDetector.h"
#include "IMU.h"

void setup() {
  // put your setup code here, to run once:
  init_motors();
  init_sonars();
  init_leak_detector();
  init_IMU();
}

void loop() {
  // put your main code here, to run repeatedly:
  if (leaky()) {
    // Do the squeaky
  }
  motor_power[0] = 10;
  set_motors();
  measure_sonars();
}
