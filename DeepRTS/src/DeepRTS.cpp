
#include "Motor.h"
#include "Sonar.h"
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

unsigned long last_command_time;

void setup() {
  init_motors();
  init_sonars();
  init_leak_detector();
  // init_MPU9250();
  init_LSM9DS0();
  kalman_filter_initialize();

  Serial.begin(115200);
  Serial.setTimeout(0);

  last_command_time = millis();
}

void loop() {
  if (leaky()) {
    // Do the squeaky
    // Can't take this L
  }

  // Check Serial every so often
  if ((millis() - last_command_time) > 5) {
    last_command_time = millis();
    read_serial_commands();
  }

  // Measurement Vector
  measure_sonars(y);   // How often can we ping?
  // measure_MPU9520(y);
  measure_LSM9DS0(y);

  // Calculate State Vector
  double dt = IMU_LSM9DS0.deltat;
  kalman_filter(mu, cov, y, C, Q_est, R_est, dt,
                mu, cov);

  // Calculate Control Outputs
  controller(mu, motor_power);

  // Execute Control Outputs
  set_motors();

  // Logging
  log_serial();
}

void log_serial() {
  // for (int i = 0; i < NUM_MEASURE; i++) {
  //   Serial.print(y[i]);
  //   Serial.print(',');
  // }
  for (int i = 0; i < NUM_STATES; i++) {
    Serial.print(mu[i]);
  
    Serial.print(',');
  }
  Serial.println();
}

void read_serial_commands() {
  dlqr_mode = FullState;
  String s = Serial.readString();
  char *line = const_cast<char*> (s.c_str());
  char *p_next = line;
  bool done = false;
  while (*p_next != '\0' && !done) {
    switch (*p_next) {
    case ZVelCommand:
      dlqr_mode |= ManZVel;
      man_z_vel = strtod(p_next+1, &p_next);
      break;
    case RollTrimCommand:
      dlqr_mode |= ManRollTrim;
      man_r_trim = strtol(p_next+1, &p_next, 10);
      break;
    case PitchTrimCommand:
      dlqr_mode |= ManPitchTrim;
      man_p_trim = strtol(p_next+1, &p_next, 10);
      break;
    case YawVelCommand:
      dlqr_mode |= ManYawVel;
      man_y_vel = strtod(p_next+1, &p_next);
      break;
    case XVelCommand:
      dlqr_mode |= ManXVel;
      man_x_vel = strtol(p_next+1, &p_next, 10);
      break;
    default:
      done = true;
      break;
    }
  }
}