
#include "Motor.h"
#include "DepthSensor.h"
#include "LeakDetector.h"
#include "IMU.h"
#include "Controller.h"

/* MATLAB Generated Kalman Filter code*/
#include "kalman_filter.h"
#include "kalman_filter_terminate.h"
#include "kalman_filter_initialize.h"

typedef enum SerialCommand {
  ZVelCommand      = (int)'Z',
  RollTrimCommand  = (int)'R',
  PitchTrimCommand = (int)'P',
  YawVelCommand    = (int)'Y',
  XVelCommand      = (int)'X',
} SerialCommand;

#define DROOP_PIN A15

void log_serial();
void read_serial_csv();
void read_serial_commands();

unsigned long state_init_time;
long last_motor_set = 0;


void setup() {

  Serial.begin(115200);
  Serial.setTimeout(30);
  analogReference(INTERNAL2V56);

  init_motors();
  init_leak_detector();
#ifdef USE_MPU9250
  init_MPU9250();
#endif
#ifdef USE_LSM9DS0
  init_LSM9DS0();
#endif
#ifdef USE_LSM9DS1
  init_LSM9DS1();
#endif
  kalman_filter_initialize();

  // Motor Init Time
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

  state_init_time = millis() + 10000;
  Serial.println("Serial Ready");
}


void loop() {
  if (leaky()) {
    // Do the squeaky
    // Can't take this L
    Serial.println("WARNING: LEAKY");
  }
  if (Serial.available() > 0) {
    read_serial_csv();
  }

  measure_depth(y);
#ifdef USE_MPU9250
  measure_MPU9250(y);
  double dt = IMU_MPU9250.deltat;
#endif
#ifdef USE_LSM9DS0
  measure_LSM9DS0(y);
  double dt = IMU_LSM9DS0.deltat;
#endif
#ifdef USE_LSM9DS1
  measure_LSM9DS1(y);
  double dt = IMU_LSM9DS1.deltat;
#endif

  // Calculate State Vector
  kalman_filter(mu, cov, y, C, Q_est, R_est, dt,
                mu, cov);

  if (millis() < state_init_time) {
    // Initialize target Z and Yaw
    target[0] = mu[0];
    target[3] = mu[3];
  } else {
    // Calculate Control Outputs
    controller(mu, motor_power);
  }

  Serial.print("IMU: ");
  Serial.print("yaw="); Serial.print(mu[3] * RAD_TO_DEG); Serial.print(";");
  Serial.print("pitch="); Serial.print(mu[2] * RAD_TO_DEG); Serial.print(";");
  Serial.print("roll="); Serial.print(mu[1] * RAD_TO_DEG); Serial.print(";");

  Serial.println();

  double voltage = (double)analogRead(DROOP_PIN) * 2.56 * 12.0 / (2.5 * 1023.0);
  Serial.print("Voltage: "); Serial.println(voltage);
  if(voltage < 10) {
    droop_factor = 0.8;
  } else if(voltage < 9){
    droop_factor = 0.75;
  } else if(voltage < 8.5) {
    droop_factor = 0.5;
  } else {
    droop_factor = 1;
  }

  if (millis() - last_motor_set > 75) {
    // Execute Control Outputs every 75ms
    last_motor_set = millis();
    ramp_motors();
  }
  // Logging
  // log_serial();
}

void log_serial() {
  Serial.println("motor powers:");
  for (int i = 0; i < NUM_MOTORS; i++) {
    Serial.print(motor_power[i]);
    Serial.print(',');
  }
  Serial.println();
}

void read_serial_csv() {
  // Reset dlqr_mode
  dlqr_mode = FullState;

  String input = Serial.readString();
  int counter = 0;
  int lastIndex = 0;

  for (unsigned int i = 0; i < input.length(); i++) {
    if (input.substring(i, i+1) == "," || input.substring(i, i+1) == "\n") {
      switch (counter) {
      case 0:
        dlqr_mode |= ManXVel;
        man_x_vel = input.substring(lastIndex, i).toInt();
        break;
      case 1:
        dlqr_mode |= ManRollTrim;
        man_r_trim = input.substring(lastIndex, i).toInt();
        break;
      case 2:
        dlqr_mode |= ManYawVel;
        man_y_vel = (double)input.substring(lastIndex, i).toInt();
        break;
      case 3:
        dlqr_mode |= ManZVel;
        man_z_vel = input.substring(lastIndex, i).toDouble();
        break;
      case 4:
        dlqr_mode |= ManPitchTrim;
        man_p_trim = input.substring(lastIndex, i).toInt();
        break;
      }
      lastIndex = i + 1;
      counter++;
    }
  }
}
