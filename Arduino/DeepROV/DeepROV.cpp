#include "Arduino.h"
#include "Servo.h"
//#include "LeakDetector.h"
//#include "Sonar.h"
//#include "Motor.h"
#include "DeepROV.h"

DeepROV::DeepROV() : leakSensor(_leakPin), sonarSensor(_trigPin, _echoPin), 
					 motor1("T100", 5), motor2("T100", 6), motor3("T100", 9), motor4("T100", 10), motor5("T100", 11) {
	_leakPin = 4;
	_trigPin = 2;
	_echoPin = 1;

}

void DeepROV::init() {
	motor1.init();
	motor2.init();
	motor3.init();
	motor4.init();
	motor5.init();
}

void DeepROV::setMotor(int motorNum, int pwm) {
	Serial.println("Set2");
	switch(motorNum) {
		case(1): motor1.writePWM(pwm);
		case(2): motor2.writePWM(pwm);
		case(3): motor3.writePWM(pwm);
		case(4): motor4.writePWM(pwm);
		case(5): motor5.writePWM(pwm);
	}
}

void DeepROV::setMotor(int setNum) {

	int _pwmVal = setNum%10000;

	/// if first digit is a 1 then set motor1 speed
    if (setNum >= 10000 && setNum < 20000) {
      if (_pwmVal>=1000 && _pwmVal<=2000) {
       	motor1.writePWM(_pwmVal);
      }
    }
    //if first digit is a 2 then set motor2 speed
    else if (setNum >= 20000 && setNum < 30000) {
      if (_pwmVal>=1000 && _pwmVal<=2000) {
        motor2.writePWM(_pwmVal);
      }
    }
    //if first digit is a 3 then set motor3 speed
    else if (setNum >= 30000 && setNum < 40000) {
      if (_pwmVal>=1000 && _pwmVal<=2000) {
        motor3.writePWM(_pwmVal);
      }
    }
    //if first digit is a 4 then set motor4 speed
    else if (setNum >= 40000 && setNum < 50000) {
      if (_pwmVal>=1000 && _pwmVal<=2000) {
        motor4.writePWM(_pwmVal);
      }
    }
    //if first digit is a 5 then set motor5 speed
    else if (setNum >= 50000 && setNum < 60000) {
      if (_pwmVal>=1000 && _pwmVal<=2000) {
        motor5.writePWM(_pwmVal);
      }
    }
    else {
    	Serial.println(setNum);
    }
}

void DeepROV::stopAll() {
	motor1.stop();
	motor2.stop();
	motor3.stop();
	motor4.stop();
	motor5.stop();
}

int readSerial() {

	String _readString = "";
  
  	while (Serial.available()) {
    	char c = Serial.read();  //gets one byte from serial buffer
    	_readString += c; //converts from raw serial bytes to a string of characters
    	delay(2);  //slow looping to allow buffer to fill with next character
  	}

	return _readString.toInt();
}