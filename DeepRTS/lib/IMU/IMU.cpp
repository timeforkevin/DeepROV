
#include "Arduino.h"
#include "IMU.h"
#include "Wire.h"

#ifdef USE_LSM9DS1

LSM9DS1 IMU_LSM9DS1;

void init_LSM9DS1() {

  Serial.begin(115200);
  IMU_LSM9DS1.settings.device.commInterface = IMU_MODE_I2C;
  IMU_LSM9DS1.settings.device.mAddress = LSM9DS1_M;
  IMU_LSM9DS1.settings.device.agAddress = LSM9DS1_AG;
  if (!IMU_LSM9DS1.begin()) {
    Serial.println("Failed to communicate with LSM9DS1.");
  }
// Set data output ranges; choose lowest ranges for maximum resolution
// Accelerometer scale can be: A_SCALE_2G, A_SCALE_4G, A_SCALE_6G, A_SCALE_8G, or A_SCALE_16G
  IMU_LSM9DS1.setAccelScale(accel_scale::A_SCALE_2G);
// Gyro scale can be:  G_SCALE__245, G_SCALE__500, or G_SCALE__2000DPS
  IMU_LSM9DS1.setGyroScale(gyro_scale::G_SCALE_245DPS);
// Magnetometer scale can be: M_SCALE_4GS, M_SCALE_8GS, M_SCALE_12GS, M_SCALE_16GS
  IMU_LSM9DS1.setMagScale(mag_scale::M_SCALE_4GS);

// Set output data rates
// Accelerometer output data rate (ODR) can be: A_ODR_3125 (3.225 Hz), A_ODR_625 (6.25 Hz), A_ODR_125 (12.5 Hz), A_ODR_25, A_ODR_50,
//                                              A_ODR_100,  A_ODR_200, A_ODR_400, A_ODR_800, A_ODR_1600 (1600 Hz)
  IMU_LSM9DS1.setAccelODR(accel_odr::XL_ODR_238); // Set accelerometer update rate at 100 Hz
// Accelerometer anti-aliasing filter rate can be 50, 194, 362, or 763 Hz
// Anti-aliasing acts like a low-pass filter allowing oversampling of accelerometer and rejection of high-frequency spurious noise.
// Strategy here is to effectively oversample accelerometer at 100 Hz and use a 50 Hz anti-aliasing (low-pass) filter frequency
// to get a smooth ~150 Hz filter update rate
//   IMU_LSM9DS1.setAccelABW(accel_abw::A_ABW_50); // Choose lowest filter setting for low noise

// Gyro output data rates can be: 95 Hz (bandwidth 12.5 or 25 Hz), 190 Hz (bandwidth 12.5, 25, 50, or 70 Hz)
//                                 380 Hz (bandwidth 20, 25, 50, 100 Hz), or 760 Hz (bandwidth 30, 35, 50, 100 Hz)
  IMU_LSM9DS1.setGyroODR(gyro_odr::G_ODR_238);  // Set gyro update rate to 190 Hz with the smallest bandwidth for low noise

// Magnetometer output data rate can be: 3.125 (ODR_3125), 6.25 (ODR_625), 12.5 (ODR_125), 25, 50, or 100 Hz
  IMU_LSM9DS1.setMagODR(mag_odr::M_ODR_80); // Set magnetometer to update every 80 ms

// Use the FIFO mode to average accelerometer and gyro readings to calculate the biases, which can then be removed from
// all subsequent measurements.
  // Serial.println("Calibrate Gyro Accel");
  // IMU_LSM9DS1.calibrate(true);
  // Serial.println("Calibrate Mag");
  IMU_LSM9DS1.calibrateMag(true);
  // Serial.println("Done");
}
void measure_LSM9DS1(double y[]) {

    // To read from the gyroscope,  first call the
    // readGyro() function. When it exits, it'll update the
    // gx, gy, and gz variables with the most current data.
    IMU_LSM9DS1.readGyro();
    float gx = -IMU_LSM9DS1.calcGyro(IMU_LSM9DS1.gx) * DEG_TO_RAD;
    float gy = -IMU_LSM9DS1.calcGyro(IMU_LSM9DS1.gy) * DEG_TO_RAD;
    float gz = -IMU_LSM9DS1.calcGyro(IMU_LSM9DS1.gz) * DEG_TO_RAD;
    // To read from the accelerometer, first call the
    // readAccel() function. When it exits, it'll update the
    // ax, ay, and az variables with the most current data.
    IMU_LSM9DS1.readAccel();
    float ax = IMU_LSM9DS1.calcAccel(IMU_LSM9DS1.ax);
    float ay = IMU_LSM9DS1.calcAccel(IMU_LSM9DS1.ay);
    float az = IMU_LSM9DS1.calcAccel(IMU_LSM9DS1.az);
    // To read from the magnetometer, first call the
    // readMag() function. When it exits, it'll update the
    // mx, my, and mz variables with the most current data.
    IMU_LSM9DS1.readMag();
    float mx = IMU_LSM9DS1.calcMag(IMU_LSM9DS1.mx);
    float my = IMU_LSM9DS1.calcMag(IMU_LSM9DS1.my);
    float mz = IMU_LSM9DS1.calcMag(IMU_LSM9DS1.mz);

    IMU_LSM9DS1.updateTime();
    MadgwickQuaternionUpdate(ax, -ay, az,
                             -gx, gy, gz,
                             mx, -my, mz,
                             IMU_LSM9DS1.deltat, IMU_LSM9DS1.q);
  double yaw   = atan2(2.0f * (IMU_LSM9DS1.q[1] * IMU_LSM9DS1.q[2] + IMU_LSM9DS1.q[0] * IMU_LSM9DS1.q[3]), 1 - 2.0f * (IMU_LSM9DS1.q[2]*IMU_LSM9DS1.q[2] + IMU_LSM9DS1.q[3]*IMU_LSM9DS1.q[3]));
  double pitch = -asin(2.0f * (IMU_LSM9DS1.q[0] * IMU_LSM9DS1.q[2] - IMU_LSM9DS1.q[1] * IMU_LSM9DS1.q[3]));
  double roll  = atan2(-2.0f * (IMU_LSM9DS1.q[0] * IMU_LSM9DS1.q[1] + IMU_LSM9DS1.q[2] * IMU_LSM9DS1.q[3]), -1 + 2.0f * (IMU_LSM9DS1.q[1]*IMU_LSM9DS1.q[1] + IMU_LSM9DS1.q[2]*IMU_LSM9DS1.q[2]));

  y[1] = -roll;
  y[2] = -pitch;
  y[3] = yaw;
}

#endif


#ifdef USE_MPU9250

MPU9250 IMU_MPU9250;

void init_MPU9250() {
  Wire.begin();
  // TWBR = 12;  // 400 kbit/sec I2C speed
  // Serial.begin(38400);

  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(12, INPUT);
  digitalWrite(12, LOW);

  byte c = IMU_MPU9250.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX);
  Serial.print(" I should be "); Serial.println(0x71, HEX);
  if (c == 0x71) // WHO_AM_I should always be 0x68
  {
    Serial.println("MPU9250 is online...");

    // Start by performing self test and reporting values
    IMU_MPU9250.MPU9250SelfTest(IMU_MPU9250.SelfTest);
    Serial.print("x-axis self test: acceleration trim within : ");
    Serial.print(IMU_MPU9250.SelfTest[0],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: acceleration trim within : ");
    Serial.print(IMU_MPU9250.SelfTest[1],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: acceleration trim within : ");
    Serial.print(IMU_MPU9250.SelfTest[2],1); Serial.println("% of factory value");
    Serial.print("x-axis self test: gyration trim within : ");
    Serial.print(IMU_MPU9250.SelfTest[3],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: gyration trim within : ");
    Serial.print(IMU_MPU9250.SelfTest[4],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: gyration trim within : ");
    Serial.print(IMU_MPU9250.SelfTest[5],1); Serial.println("% of factory value");

    // Calibrate gyro and accelerometers, load biases in bias registers
    IMU_MPU9250.calibrateMPU9250(IMU_MPU9250.gyroBias, IMU_MPU9250.accelBias);
    IMU_MPU9250.initMPU9250();

    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    Serial.println("MPU9250 initialized for active data mode....");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = IMU_MPU9250.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX);
    Serial.print(" I should be "); Serial.println(0x48, HEX);

    // Get magnetometer calibration from AK8963 ROM
    IMU_MPU9250.initAK8963(IMU_MPU9250.magCalibration);
    // Initialize device for active mode read of magnetometer
    Serial.println("AK8963 initialized for active data mode....");
  }
}

void measure_MPU9250(double y[]) {
  IMU_MPU9250.readAccelData(IMU_MPU9250.accelCount);  // Read the x/y/z adc values
  IMU_MPU9250.getAres();

  // Now we'll calculate the accleration value into actual g's
  // This depends on scale being set
  IMU_MPU9250.ax = (float)IMU_MPU9250.accelCount[0]*IMU_MPU9250.aRes; // - accelBias[0];
  IMU_MPU9250.ay = (float)IMU_MPU9250.accelCount[1]*IMU_MPU9250.aRes; // - accelBias[1];
  IMU_MPU9250.az = (float)IMU_MPU9250.accelCount[2]*IMU_MPU9250.aRes; // - accelBias[2];

  IMU_MPU9250.readGyroData(IMU_MPU9250.gyroCount);  // Read the x/y/z adc values
  IMU_MPU9250.getGres();

  // Calculate the gyro value into actual degrees per second
  // This depends on scale being set
  IMU_MPU9250.gx = (float)IMU_MPU9250.gyroCount[0]*IMU_MPU9250.gRes;
  IMU_MPU9250.gy = (float)IMU_MPU9250.gyroCount[1]*IMU_MPU9250.gRes;
  IMU_MPU9250.gz = (float)IMU_MPU9250.gyroCount[2]*IMU_MPU9250.gRes;

  IMU_MPU9250.readMagData(IMU_MPU9250.magCount);  // Read the x/y/z adc values
  IMU_MPU9250.getMres();

  // Calculate the magnetometer values in milliGauss
  // Include factory calibration per data sheet and user environmental
  // corrections
  // Get actual magnetometer value, this depends on scale being set
  IMU_MPU9250.mx = (float)IMU_MPU9250.magCount[0]*IMU_MPU9250.mRes*IMU_MPU9250.magCalibration[0] -
             IMU_MPU9250.magbias[0];
  IMU_MPU9250.my = (float)IMU_MPU9250.magCount[1]*IMU_MPU9250.mRes*IMU_MPU9250.magCalibration[1] -
             IMU_MPU9250.magbias[1];
  IMU_MPU9250.mz = (float)IMU_MPU9250.magCount[2]*IMU_MPU9250.mRes*IMU_MPU9250.magCalibration[2] -
             IMU_MPU9250.magbias[2];

  // Must be called before updating quaternions!
  IMU_MPU9250.updateTime();
  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
  // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
  // (+ up) of accelerometer and gyro! We have to make some allowance for this
  // orientationmismatch in feeding the output to the quaternion filter. For the
  // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
  // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
  // modified to allow any convenient orientation convention. This is ok by
  // aircraft orientation standards! Pass gyro rate as rad/s
  MadgwickQuaternionUpdate(IMU_MPU9250.ax, IMU_MPU9250.ay, IMU_MPU9250.az,
    IMU_MPU9250.gx*DEG_TO_RAD, IMU_MPU9250.gy*DEG_TO_RAD, IMU_MPU9250.gz*DEG_TO_RAD,
    IMU_MPU9250.mx, IMU_MPU9250.my, IMU_MPU9250.mz, IMU_MPU9250.deltat, IMU_MPU9250.q);
  // MahonyQuaternionUpdate(IMU_MPU9250.ax, IMU_MPU9250.ay, IMU_MPU9250.az, IMU_MPU9250.gx*DEG_TO_RAD,
  //                        IMU_MPU9250.gy*DEG_TO_RAD, IMU_MPU9250.gz*DEG_TO_RAD, IMU_MPU9250.my,
  //                        IMU_MPU9250.mx, IMU_MPU9250.mz, IMU_MPU9250.deltat, IMU_MPU9250.q, IMU_MPU9250.eInt);


  // Define output variables from updated quaternion---these are Tait-Bryan
  // angles, commonly used in aircraft orientation. In this coordinate system,
  // the positive z-axis is down toward Earth. Yaw is the angle between Sensor
  // x-axis and Earth magnetic North (or true North if corrected for local
  // declination, looking down on the sensor positive yaw is counterclockwise.
  // Pitch is angle between sensor x-axis and Earth ground plane, toward the
  // Earth is positive, up toward the sky is negative. Roll is angle between
  // sensor y-axis and Earth ground plane, y-axis up is positive roll. These
  // arise from the definition of the homogeneous rotation matrix constructed
  // from quaternions. Tait-Bryan angles as well as Euler angles are
  // non-commutative; that is, the get the correct orientation the rotations
  // must be applied in the correct order which for this configuration is yaw,
  // pitch, and then roll.
  // For more see
  // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  // which has additional links.
  IMU_MPU9250.yaw   = atan2(2.0f * (IMU_MPU9250.q[1] * IMU_MPU9250.q[2] + IMU_MPU9250.q[0] * IMU_MPU9250.q[3]), 1 - 2.0f * (IMU_MPU9250.q[2]*IMU_MPU9250.q[2] + IMU_MPU9250.q[3]*IMU_MPU9250.q[3]));
  IMU_MPU9250.pitch = asin(2.0f * (IMU_MPU9250.q[0] * IMU_MPU9250.q[2] - IMU_MPU9250.q[1] * IMU_MPU9250.q[3]));
  IMU_MPU9250.roll  = atan2(-2.0f * (IMU_MPU9250.q[0] * IMU_MPU9250.q[1] + IMU_MPU9250.q[2] * IMU_MPU9250.q[3]), -1 + 2.0f * (IMU_MPU9250.q[1]*IMU_MPU9250.q[1] + IMU_MPU9250.q[2]*IMU_MPU9250.q[2]));
  y[1] = -IMU_MPU9250.roll;
  y[2] = -IMU_MPU9250.pitch;
  y[3] = IMU_MPU9250.yaw;
}

#endif
#ifdef USE_LSM9DS0

LSM9DS0 IMU_LSM9DS0(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);

void init_LSM9DS0() {
  Serial.begin(115200); // Start serial at 115200 bps

  // begin() returns a 16-bit value which includes both the gyro
  // and accelerometers WHO_AM_I response. You can check this to
  // make sure communication was successful.
  uint32_t status = IMU_LSM9DS0.begin();

  Serial.print("LSM9DS0 WHO_AM_I's returned: 0x");
  Serial.println(status, HEX);
  Serial.println("Should be 0x49D4");
  Serial.println();
  delay(2000);

// Set data output ranges; choose lowest ranges for maximum resolution
// Accelerometer scale can be: A_SCALE_2G, A_SCALE_4G, A_SCALE_6G, A_SCALE_8G, or A_SCALE_16G
  IMU_LSM9DS0.setAccelScale(IMU_LSM9DS0.A_SCALE_2G);
// Gyro scale can be:  G_SCALE__245, G_SCALE__500, or G_SCALE__2000DPS
  IMU_LSM9DS0.setGyroScale(IMU_LSM9DS0.G_SCALE_245DPS);
// Magnetometer scale can be: M_SCALE_2GS, M_SCALE_4GS, M_SCALE_8GS, M_SCALE_12GS
  IMU_LSM9DS0.setMagScale(IMU_LSM9DS0.M_SCALE_2GS);

// Set output data rates
// Accelerometer output data rate (ODR) can be: A_ODR_3125 (3.225 Hz), A_ODR_625 (6.25 Hz), A_ODR_125 (12.5 Hz), A_ODR_25, A_ODR_50,
//                                              A_ODR_100,  A_ODR_200, A_ODR_400, A_ODR_800, A_ODR_1600 (1600 Hz)
  IMU_LSM9DS0.setAccelODR(IMU_LSM9DS0.A_ODR_200); // Set accelerometer update rate at 100 Hz
// Accelerometer anti-aliasing filter rate can be 50, 194, 362, or 763 Hz
// Anti-aliasing acts like a low-pass filter allowing oversampling of accelerometer and rejection of high-frequency spurious noise.
// Strategy here is to effectively oversample accelerometer at 100 Hz and use a 50 Hz anti-aliasing (low-pass) filter frequency
// to get a smooth ~150 Hz filter update rate
  IMU_LSM9DS0.setAccelABW(IMU_LSM9DS0.A_ABW_50); // Choose lowest filter setting for low noise

// Gyro output data rates can be: 95 Hz (bandwidth 12.5 or 25 Hz), 190 Hz (bandwidth 12.5, 25, 50, or 70 Hz)
//                                 380 Hz (bandwidth 20, 25, 50, 100 Hz), or 760 Hz (bandwidth 30, 35, 50, 100 Hz)
  IMU_LSM9DS0.setGyroODR(IMU_LSM9DS0.G_ODR_190_BW_125);  // Set gyro update rate to 190 Hz with the smallest bandwidth for low noise

// Magnetometer output data rate can be: 3.125 (ODR_3125), 6.25 (ODR_625), 12.5 (ODR_125), 25, 50, or 100 Hz
  IMU_LSM9DS0.setMagODR(IMU_LSM9DS0.M_ODR_125); // Set magnetometer to update every 80 ms

// Use the FIFO mode to average accelerometer and gyro readings to calculate the biases, which can then be removed from
// all subsequent measurements.
  IMU_LSM9DS0.calLSM9DS0(IMU_LSM9DS0.gbias, IMU_LSM9DS0.abias);
}

void measure_LSM9DS0(double y[]) {
  IMU_LSM9DS0.readGyro();          // Read gyro data
  IMU_LSM9DS0.readAccel();         // Read accelerometer data
  IMU_LSM9DS0.readMag();           // Read magnetometer data

  // Must be called before updating quaternions!
  IMU_LSM9DS0.updateTime();

  // Sensors x- and y-axes are aligned but magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
  // This is ok by aircraft orientation standards!
  // Pass gyro rate as rad/s
  MadgwickQuaternionUpdate(IMU_LSM9DS0.ax, IMU_LSM9DS0.ay, IMU_LSM9DS0.az,
    IMU_LSM9DS0.gx*DEG_TO_RAD, IMU_LSM9DS0.gy*DEG_TO_RAD, IMU_LSM9DS0.gz*DEG_TO_RAD,
    IMU_LSM9DS0.mx, IMU_LSM9DS0.my, IMU_LSM9DS0.mz, IMU_LSM9DS0.deltat, IMU_LSM9DS0.q);
  // MahonyQuaternionUpdate(IMU_LSM9DS0.ax, IMU_LSM9DS0.ay, IMU_LSM9DS0.az,
  //   IMU_LSM9DS0.gx*DEG_TO_RAD, IMU_LSM9DS0.gy*DEG_TO_RAD, IMU_LSM9DS0.gz*DEG_TO_RAD,
  //   IMU_LSM9DS0.mx, IMU_LSM9DS0.my, IMU_LSM9DS0.mz, IMU_LSM9DS0.deltat, IMU_LSM9DS0.q, IMU_LSM9DS0.eInt);
  IMU_LSM9DS0.yaw   = atan2(2.0f * (IMU_LSM9DS0.q[1] * IMU_LSM9DS0.q[2] + IMU_LSM9DS0.q[0] * IMU_LSM9DS0.q[3]), 1 - 2.0f * (IMU_LSM9DS0.q[2]*IMU_LSM9DS0.q[2] + IMU_LSM9DS0.q[3]*IMU_LSM9DS0.q[3]));
  IMU_LSM9DS0.pitch = asin(2.0f * (IMU_LSM9DS0.q[0] * IMU_LSM9DS0.q[2] - IMU_LSM9DS0.q[1] * IMU_LSM9DS0.q[3]));
  IMU_LSM9DS0.roll  = atan2(-2.0f * (IMU_LSM9DS0.q[0] * IMU_LSM9DS0.q[1] + IMU_LSM9DS0.q[2] * IMU_LSM9DS0.q[3]), -1 + 2.0f * (IMU_LSM9DS0.q[1]*IMU_LSM9DS0.q[1] + IMU_LSM9DS0.q[2]*IMU_LSM9DS0.q[2]));

  y[1] = -IMU_LSM9DS0.roll;
  y[2] = -IMU_LSM9DS0.pitch;
  y[3] = IMU_LSM9DS0.yaw;
}

#endif
