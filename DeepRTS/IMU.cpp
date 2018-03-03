
#include "Arduino.h"
#include "IMU.h"
#include "Wire.h"

#ifdef USE_MPU9520

MPU9250 IMU_MPU9520;

void init_MPU9250() {
  Wire.begin();
  // TWBR = 12;  // 400 kbit/sec I2C speed
  Serial.begin(38400);

  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(12, INPUT);
  digitalWrite(12, LOW);

  byte c = IMU_MPU9520.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX);
  Serial.print(" I should be "); Serial.println(0x71, HEX);
  if (c == 0x71) // WHO_AM_I should always be 0x68
  {
    Serial.println("MPU9250 is online...");

    // Start by performing self test and reporting values
    IMU_MPU9520.MPU9250SelfTest(IMU_MPU9520.SelfTest);
    Serial.print("x-axis self test: acceleration trim within : ");
    Serial.print(IMU_MPU9520.SelfTest[0],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: acceleration trim within : ");
    Serial.print(IMU_MPU9520.SelfTest[1],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: acceleration trim within : ");
    Serial.print(IMU_MPU9520.SelfTest[2],1); Serial.println("% of factory value");
    Serial.print("x-axis self test: gyration trim within : ");
    Serial.print(IMU_MPU9520.SelfTest[3],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: gyration trim within : ");
    Serial.print(IMU_MPU9520.SelfTest[4],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: gyration trim within : ");
    Serial.print(IMU_MPU9520.SelfTest[5],1); Serial.println("% of factory value");

    // Calibrate gyro and accelerometers, load biases in bias registers
    IMU_MPU9520.calibrateMPU9250(IMU_MPU9520.gyroBias, IMU_MPU9520.accelBias);
    IMU_MPU9520.initMPU9250();

    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    Serial.println("MPU9250 initialized for active data mode....");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = IMU_MPU9520.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX);
    Serial.print(" I should be "); Serial.println(0x48, HEX);

    // Get magnetometer calibration from AK8963 ROM
    IMU_MPU9520.initAK8963(IMU_MPU9520.magCalibration);
    // Initialize device for active mode read of magnetometer
    Serial.println("AK8963 initialized for active data mode....");
  }
}

void measure_MPU9520() {
  IMU_MPU9520.readAccelData(IMU_MPU9520.accelCount);  // Read the x/y/z adc values
  IMU_MPU9520.getAres();

  // Now we'll calculate the accleration value into actual g's
  // This depends on scale being set
  IMU_MPU9520.ax = (float)IMU_MPU9520.accelCount[0]*IMU_MPU9520.aRes; // - accelBias[0];
  IMU_MPU9520.ay = (float)IMU_MPU9520.accelCount[1]*IMU_MPU9520.aRes; // - accelBias[1];
  IMU_MPU9520.az = (float)IMU_MPU9520.accelCount[2]*IMU_MPU9520.aRes; // - accelBias[2];

  IMU_MPU9520.readGyroData(IMU_MPU9520.gyroCount);  // Read the x/y/z adc values
  IMU_MPU9520.getGres();

  // Calculate the gyro value into actual degrees per second
  // This depends on scale being set
  IMU_MPU9520.gx = (float)IMU_MPU9520.gyroCount[0]*IMU_MPU9520.gRes;
  IMU_MPU9520.gy = (float)IMU_MPU9520.gyroCount[1]*IMU_MPU9520.gRes;
  IMU_MPU9520.gz = (float)IMU_MPU9520.gyroCount[2]*IMU_MPU9520.gRes;

  IMU_MPU9520.readMagData(IMU_MPU9520.magCount);  // Read the x/y/z adc values
  IMU_MPU9520.getMres();

  // Calculate the magnetometer values in milliGauss
  // Include factory calibration per data sheet and user environmental
  // corrections
  // Get actual magnetometer value, this depends on scale being set
  IMU_MPU9520.mx = (float)IMU_MPU9520.magCount[0]*IMU_MPU9520.mRes*IMU_MPU9520.magCalibration[0] -
             IMU_MPU9520.magbias[0];
  IMU_MPU9520.my = (float)IMU_MPU9520.magCount[1]*IMU_MPU9520.mRes*IMU_MPU9520.magCalibration[1] -
             IMU_MPU9520.magbias[1];
  IMU_MPU9520.mz = (float)IMU_MPU9520.magCount[2]*IMU_MPU9520.mRes*IMU_MPU9520.magCalibration[2] -
             IMU_MPU9520.magbias[2];

  // Must be called before updating quaternions!
  IMU_MPU9520.updateTime();
  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
  // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
  // (+ up) of accelerometer and gyro! We have to make some allowance for this
  // orientationmismatch in feeding the output to the quaternion filter. For the
  // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
  // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
  // modified to allow any convenient orientation convention. This is ok by
  // aircraft orientation standards! Pass gyro rate as rad/s
  MadgwickQuaternionUpdate(IMU_MPU9520.ax, IMU_MPU9520.ay, IMU_MPU9520.az,
    IMU_MPU9520.gx*DEG_TO_RAD, IMU_MPU9520.gy*DEG_TO_RAD, IMU_MPU9520.gz*DEG_TO_RAD,
    IMU_MPU9520.my, IMU_MPU9520.mx, IMU_MPU9520.mz, IMU_MPU9520.deltat, IMU_MPU9520.q);
  // MahonyQuaternionUpdate(IMU_MPU9520.ax, IMU_MPU9520.ay, IMU_MPU9520.az, IMU_MPU9520.gx*DEG_TO_RAD,
  //                        IMU_MPU9520.gy*DEG_TO_RAD, IMU_MPU9520.gz*DEG_TO_RAD, IMU_MPU9520.my,
  //                        IMU_MPU9520.mx, IMU_MPU9520.mz, IMU_MPU9520.deltat, IMU_MPU9520.q, IMU_MPU9520.eInt);


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
  IMU_MPU9520.yaw   = atan2(2.0f * (IMU_MPU9520.q[1] * IMU_MPU9520.q[2] + IMU_MPU9520.q[0] * IMU_MPU9520.q[3]), 1 - 2.0f * (IMU_MPU9520.q[2]*IMU_MPU9520.q[2] + IMU_MPU9520.q[3]*IMU_MPU9520.q[3]));
  IMU_MPU9520.pitch = asin(2.0f * (IMU_MPU9520.q[0] * IMU_MPU9520.q[2] - IMU_MPU9520.q[1] * IMU_MPU9520.q[3]));
  IMU_MPU9520.roll  = atan2(2.0f * (IMU_MPU9520.q[0] * IMU_MPU9520.q[1] + IMU_MPU9520.q[2] * IMU_MPU9520.q[3]), 1 - 2.0f * (IMU_MPU9520.q[1]*IMU_MPU9520.q[1] + IMU_MPU9520.q[2]*IMU_MPU9520.q[2]));
}

#endif
#ifdef USE_LSM9DS0

LSM9DS0 IMU_LSM9DS0(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);

void init_LSM9DS0() {
  Serial.begin(38400); // Start serial at 38400 bps

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

void measure_LSM9DS0() {
  IMU_LSM9DS0.readGyro(IMU_LSM9DS0.gyroCount);           // Read raw gyro data
  IMU_LSM9DS0.gx = (float)IMU_LSM9DS0.gyroCount[0]*IMU_LSM9DS0.gRes - IMU_LSM9DS0.gbias[0];
  IMU_LSM9DS0.gy = (float)IMU_LSM9DS0.gyroCount[1]*IMU_LSM9DS0.gRes - IMU_LSM9DS0.gbias[1];
  IMU_LSM9DS0.gz = (float)IMU_LSM9DS0.gyroCount[2]*IMU_LSM9DS0.gRes - IMU_LSM9DS0.gbias[2];

  IMU_LSM9DS0.readAccel(IMU_LSM9DS0.accelCount);         // Read raw accelerometer data
  IMU_LSM9DS0.ax = (float)IMU_LSM9DS0.accelCount[0]*IMU_LSM9DS0.aRes - IMU_LSM9DS0.abias[0];
  IMU_LSM9DS0.ay = (float)IMU_LSM9DS0.accelCount[1]*IMU_LSM9DS0.aRes - IMU_LSM9DS0.abias[1];
  IMU_LSM9DS0.az = (float)IMU_LSM9DS0.accelCount[2]*IMU_LSM9DS0.aRes - IMU_LSM9DS0.abias[2];

  IMU_LSM9DS0.readMag(IMU_LSM9DS0.magCount);           // Read raw magnetometer data
  IMU_LSM9DS0.mx = (float)IMU_LSM9DS0.magCount[0]*IMU_LSM9DS0.mRes;     // Convert to Gauss and correct for calibration
  IMU_LSM9DS0.my = (float)IMU_LSM9DS0.magCount[1]*IMU_LSM9DS0.mRes;
  IMU_LSM9DS0.mz = (float)IMU_LSM9DS0.magCount[2]*IMU_LSM9DS0.mRes;

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
  IMU_LSM9DS0.roll  = atan2(2.0f * (IMU_LSM9DS0.q[0] * IMU_LSM9DS0.q[1] + IMU_LSM9DS0.q[2] * IMU_LSM9DS0.q[3]), 1 - 2.0f * (IMU_LSM9DS0.q[1]*IMU_LSM9DS0.q[1] + IMU_LSM9DS0.q[2]*IMU_LSM9DS0.q[2]));
}

#endif
