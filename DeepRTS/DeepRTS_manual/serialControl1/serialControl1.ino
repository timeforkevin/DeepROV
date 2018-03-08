#include "Arduino.h"
#include "Servo.h"
#include "DeepROV.h"

DeepROV AUV;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  AUV.init();
}

void loop() {
  // put your main code here, to run repeatedly:
  
  /*String rString = "";

  rString = readSerial();

  if (rString.length() > 0) {
    Serial.println(rString);  //so you can see the captured string
    int n = rString.toInt();  //convert readString into a number
    rString=""; //empty for next input
    AUV.setMotor(n);
    n=0;
  }*/

  int readInt = 0;
  readInt = readSerial();
  if (readInt > 1000) {
    AUV.setMotor(readInt);
  }
  
}
