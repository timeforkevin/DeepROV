
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

#include "Arduino.h"
// just adding a random comment
double measure_depth() {
  int acc = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    acc += analogRead(A3);
  }
  double a = ((double)acc) / NUM_SAMPLES;
  
  double p = (a-SCALE_AMIN)
           / (SCALE_AMAX-SCALE_AMIN)
           * (SCALE_PMAX-SCALE_PMIN)
           + SCALE_PMIN;
  double depth = (p-PRESSURE_OFFSET)/SCALE_PRESSURE_DEPTH;
  return depth;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  analogReference(INTERNAL2V56);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(measure_depth());
}
