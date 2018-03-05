#ifndef IMU_H
#define IMU_H

#include "Arduino.h"
#include "quaternionFilters.h"


// #define USE_MPU9520
#define USE_LSM9DS0



#ifdef USE_MPU9520
#include "MPU9250.h"
extern MPU9250 IMU_MPU9250;
void init_MPU9250();
void measure_MPU9520(double y[]);
#endif

#ifdef USE_LSM9DS0
#include "SFE_LSM9DS0.h"
// Create an instance of the LSM9DS0 library called `IMU` the
// parameters for this constructor are:
// [I2C Mode],[gyro I2C address],[xm I2C address]
#define LSM9DS0_XM  0x1D // Would be 0x1E if SDO_XM is LOW
#define LSM9DS0_G   0x6B // Would be 0x6A if SDO_G is LOW
extern LSM9DS0 IMU_LSM9DS0;
void init_LSM9DS0();
void measure_LSM9DS0(double y[]);
#endif

#endif
