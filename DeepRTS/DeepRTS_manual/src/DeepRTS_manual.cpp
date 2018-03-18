#include "Motor.h"

#define NUM_MOTORS 5
#define DROOP_PIN A15
long motor_pwms[5] = {1500, 1500, 1500, 1500, 1500};


void setup() {
  Serial.begin(115200);
  Serial.setTimeout(30);
  analogReference(INTERNAL2V56);
  delay(5000);
  init_motors();

  double voltage = (double)analogRead(DROOP_PIN) * 2.56 * 12.0 / (2.5 * 1023.0);
  Serial.print("Voltage="); Serial.println(voltage);

  for (int i = 0; i < NUM_MOTORS; i++) {
    motor_power[i] = 0;
  }
  set_motors();
  delay(1000);

  for (int i = 0; i < NUM_MOTORS; i++) {
    motor_power[i] = POWER_MAX;
  }
  set_motors();
  delay(2000);

  for (int i = 0; i < NUM_MOTORS; i++) {
    motor_power[i] = 0;
  }
  set_motors();
  delay(1000);
}

void loop() {
  // run the motors forward

  motor_pwms[0] = motor_pwms[1] = 1500;
  set_motors_raw(motor_pwms);
  delay(1000);

  for(long i = 1500; i <= 1700; i += 10) {
    motor_pwms[0] += 10;
    motor_pwms[1] -= 10;
    set_motors_raw(motor_pwms);
    double voltage = (double)analogRead(DROOP_PIN) * 2.56 * 12.0 / (2.5 * 1023.0);
    Serial.print("Voltage="); Serial.println(voltage);
    delay(1500);
  }

  // run the motors backwards
  for(long i = 1700; i >= 1500; i -= 10) {
    motor_pwms[0] -= 10;
    motor_pwms[1] += 10;
    set_motors_raw(motor_pwms);
    double voltage = (double)analogRead(DROOP_PIN) * 2.56 * 12.0 / (2.5 * 1023.0);
    Serial.print("Voltage="); Serial.println(voltage);
    delay(1500);
  }
}
