#include "Motor.h"
#include "LeakDetector.h"

#define NUM_MOTORS 5
long motor_pwms[5] = {1500, 1500, 1500, 1500};

void setPWM(long val) {
  if(val > 10000 && val < 20000) motor_pwms[0] = val - 10000;
  else if(val > 20000 && val < 30000) motor_pwms[1] = val - 20000;
  else if(val > 30000 && val < 40000) motor_pwms[2] = val - 30000;
  else if(val > 40000 && val < 50000) motor_pwms[3] = val - 40000;
  else if(val > 50000 && val < 60000) motor_pwms[4] = val - 50000;
}

void parseInput(String input) {
  int counter = 0;
  int lastIndex = 0;

  for (int i = 0; i < input.length(); i++) {
    if (input.substring(i, i+1) == "," || input.substring(i, i+1) == "\n") {
      setPWM(input.substring(lastIndex, i).toInt());
      lastIndex = i + 1;
      counter++;
    }
  }
}

void printPWMs() {
  for(int i = 0; i < NUM_MOTORS; i++) {
    Serial.print("motor");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(motor_pwms[i]);
    if( (i+1) == NUM_MOTORS) Serial.println();
    else Serial.print(", ");
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(20);
  init_motors();
  set_motors_raw(motor_pwms);
  delay(2000);
  // init_leak_detector();
}

void loop() {
  // if(leaky()) {
  //   // Took the L
  // } else {
  //   while(Serial.available() > 0) {
  //     parseInput(Serial.readString());
  //     printPWMs();
  //     set_motors_raw(motor_pwms);
  //   }
  // }

  while(Serial.available() > 0) {
    parseInput(Serial.readString());
    printPWMs();
    set_motors_raw(motor_pwms);
  }

}
