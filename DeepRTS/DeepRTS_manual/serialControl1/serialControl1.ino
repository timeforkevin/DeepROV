#include <Servo.h>

int servoPin = 5;
Servo servo;

void setup() {
  servo.attach(servoPin);

//  servo.writeMicroseconds(1500); // send "stop" signal to ESC.
  delay(1000);
  servo.writeMicroseconds(1700);
  delay(1000); // delay to allow the ESC to recognize the stopped signal

}

void loop() {

//  servo.writeMicroseconds(1700); // Send signal to ESC.
}
