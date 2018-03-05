
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
  ManualZCommand     = (int)'Z',
  ManualRollCommand  = (int)'R',
  ManualPitchCommand = (int)'P',
  ManualYawCommand   = (int)'Y',
  VelocityCommand    = (int)'V'
} SerialCommand;


void setup() {
  init_motors();
  init_sonars();
  init_leak_detector();
  // init_MPU9250();
  init_LSM9DS0();
  kalman_filter_initialize();

  Serial.begin(115200);
  Serial.setTimeout(5);
}

void loop() {
  if (leaky()) {
    // Do the squeaky
    // Can't take this L
  }

  // Check Serial every so often
  if (1) {
    // Usage: dlqr_mode = FullState | ManualZ;
    // Usage: manual[0] = -0.1; // To go up
    dlqr_mode = FullState;
    String s = Serial.readString();
    char *line = const_cast<char*> (s.c_str());
    char *p_next = line;
    bool done = false;
    while (*p_next != '\0' && !done) {
      switch (*p_next) {
      case ManualZCommand:
        dlqr_mode |= ManualZ;
        manual[0] = strtod(p_next+1, &p_next);
        Serial.print(manual[0]);
        break;
      case ManualRollCommand:
        dlqr_mode |= ManualRoll;
        manual[1] = strtod(p_next+1, &p_next);
        Serial.print(manual[1]);
        break;
      case ManualPitchCommand:
        dlqr_mode |= ManualPitch;
        manual[2] = strtod(p_next+1, &p_next);
        Serial.print(manual[2]);
        break;
      case ManualYawCommand:
        dlqr_mode |= ManualYaw;
        manual[3] = strtod(p_next+1, &p_next);
        Serial.print(manual[3]);
        break;
      case VelocityCommand:
        target_vel = strtod(p_next+1, &p_next);
        Serial.print(target_vel);
        break;
      default:
        done = true;
        break;
      }
    }
  }

  if (dlqr_mode & ManualZ) {
    Serial.print((char)ManualZ);
    Serial.print(manual[0]);
  }
  if (dlqr_mode & ManualRoll) {
    Serial.print((char)ManualRoll);
    Serial.print(manual[1]);
  }
  if (dlqr_mode & ManualPitch) {
    Serial.print((char)ManualPitch);
    Serial.print(manual[2]);
  }
  if (dlqr_mode & ManualYaw) {
    Serial.print((char)ManualYaw);
    Serial.print(manual[3]);
  }
  Serial.println('a');
  // Measurement Vector
  measure_sonars(y);   // How often can we ping?
  // measure_MPU9520(y);
  measure_LSM9DS0(y);

  // Calculate State Vector
  kalman_filter(mu, cov, y, C, Q_est, R_est,
                mu, cov);

  // Calculate Control Outputs
  dlqr(mu, motor_power);

  // Execute Control Outputs
  set_motors();
}
