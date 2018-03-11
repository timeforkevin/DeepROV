
#include "Arduino.h"
#include "DepthSensor.h"

#define SCALE_VMIN 0.5
#define SCALE_VMAX 4.5
#define SCALE_AMIN (SCALE_VMIN/5.0*1023)
#define SCALE_AMAX (SCALE_VMAX/5.0*1023)
#define SCALE_PMIN 0                 // [psi]
#define SCALE_PMAX 145.038           // [psi]
#define SCALE_PRESSURE_DEPTH 0.03703 // [psi/in]
#define PRESSURE_OFFSET 14.6959      // [psi]

const unsigned int depth_pin = A1;

void measure_depth(double y[]) {
  int a = analogRead(depth_pin);
  double p = (a-SCALE_AMIN)
           / (SCALE_AMAX-SCALE_AMIN)
           * (SCALE_PMAX-SCALE_PMIN)
           + SCALE_PMIN;
  double depth = (p-PRESSURE_OFFSET)/SCALE_PRESSURE_DEPTH;
  y[0] = -depth;
}