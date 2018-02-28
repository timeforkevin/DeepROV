#ifndef IMU_H
#define IMU_H

#include "Arduino.h"
#include "MPU9250.h"
#include "quaternionFilters.h"

extern MPU9250 IMU;

void init_IMU();



#endif