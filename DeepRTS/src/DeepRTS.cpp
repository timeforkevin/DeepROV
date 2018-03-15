
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
  XVelCommand      = (int)'X'
} SerialCommand;

int droopPin = A15;
int voltage = 0;

void log_serial();
void read_serial_csv();
void read_serial_commands();

unsigned long state_init_time;
long last_motor_set = 0;

void setup() {

  Serial.begin(115200);
  Serial.setTimeout(30);

  voltage = analogRead(droopPin) * 5 * 3 / 1023;
  Serial.print("Voltage: "); Serial.println(voltage);

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
    motor_power[i] = 100;
  }
  set_motors();
  delay(2000);

  for (int i = 0; i < NUM_MOTORS; i++) {
    motor_power[i] = 0;
  }
  set_motors();
  delay(1000);

  state_init_time = millis() + 15000;
  Serial.println("Serial Ready");
}


void loop() {
  voltage = analogRead(droopPin) * 5 * 3 / 1023;
  Serial.print("Voltage: "); Serial.println(voltage);

  if (leaky()) {
    // Do the squeaky
    // Can't take this L
    Serial.println("WARNING: LEAKY");
  }
  if (Serial.available() > 0) {
    read_serial_csv();
  }

  // measure_depth(y);
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

  if (millis() - last_motor_set > 100) {
    // Execute Control Outputs every 100ms
    last_motor_set = millis();
    set_motors();
  }
  // Logging
  log_serial();
}

void log_serial() {
  // for (int i = 0; i < NUM_MEASURE; i++) {
  //   Serial.print(y[i],4);
  //   Serial.print(',');
  // }
  // for (int i = 0; i < 4; i++) {
  //   Serial.print(mu[i]);
  //   Serial.print(',');
  // }
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

  for (int i = 0; i < input.length(); i++) {
    if (input.substring(i, i+1) == "," || input.substring(i, i+1) == "\n") {
      Serial.print(input.substring(lastIndex, i));
      Serial.print(',');
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
        man_z_vel = (double)input.substring(lastIndex, i).toInt();
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
  // Serial.print(man_x_vel);
  // Serial.print(',');
  // Serial.print(man_r_trim);
  // Serial.print(',');
  // Serial.print(man_y_vel);
  // Serial.print(',');
  // Serial.print(man_z_vel);
  // Serial.print(',');
  // Serial.print(man_p_trim);
  Serial.print(':');
}

// void read_serial_commands() {
//   // Reset dlqr_mode
//   dlqr_mode = FullState;
//   String s = Serial.readString();
//   char *line = const_cast<char*> (s.c_str());
//   char *p_next = line;
//   bool done = false;
//   while (*p_next != '\0' && !done) {
//     switch (*p_next) {
//     case ZVelCommand:
//       dlqr_mode |= ManZVel;
//       man_z_vel = strtod(p_next+1, &p_next);
//       break;
//     case RollTrimCommand:
//       dlqr_mode |= ManRollTrim;
//       man_r_trim = strtol(p_next+1, &p_next, 10);
//       break;
//     case PitchTrimCommand:
//       dlqr_mode |= ManPitchTrim;
//       man_p_trim = strtol(p_next+1, &p_next, 10);
//       break;
//     case YawVelCommand:
//       dlqr_mode |= ManYawVel;
//       man_y_vel = strtod(p_next+1, &p_next);
//       break;
//     case XVelCommand:
//       dlqr_mode |= ManXVel;
//       man_x_vel = strtol(p_next+1, &p_next, 10);
//       break;
//     default:
//       done = true;
//       break;
//     }
//   }
// }
