
#include "Arduino.h"
#include "DepthSensor.h"

#define SCALE_VMIN 0.5
#define SCALE_VMAX 4.5

#define AVREF (2.56)
#define SCALE_AMIN (SCALE_VMIN/AVREF*1023)
#define SCALE_AMAX (SCALE_VMAX/AVREF*1023)
#define SCALE_PMIN 0                 // [psi]
#define SCALE_PMAX 145.038           // [psi]
#define SCALE_PRESSURE_DEPTH 0.03703 // [psi/in]
#define PRESSURE_OFFSET 0      // [psi]

#define NUM_SAMPLES 100

const unsigned int depth_pin = A3;

void measure_depth(double y[]) {
  int acc = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    acc += analogRead(depth_pin);
  }
  double a = ((double)acc) / NUM_SAMPLES;

  double p = (a-SCALE_AMIN)
           / (SCALE_AMAX-SCALE_AMIN)
           * (SCALE_PMAX-SCALE_PMIN)
           + SCALE_PMIN;
  double depth = (p-PRESSURE_OFFSET)/SCALE_PRESSURE_DEPTH;
  // y[0] = depth;
}
