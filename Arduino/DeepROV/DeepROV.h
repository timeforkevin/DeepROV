#include "Arduino.h"
#include "Servo.h"
#include "LeakDetector.h"
#include "Sonar.h"
#include "Motor.h"

class DeepROV  {
public:
	DeepROV();
	void init();
	void setMotor(int motorNum, int pwm); //for manually setting motors
	void setMotor(int serialVal); //for easy setting of motors from read serial value
	void stopAll(); //stop all motors

	LeakDetector leakSensor;
	Sonar sonarSensor;
	Motor motor1;
	Motor motor2;
	Motor motor3;
	Motor motor4;
	Motor motor5;

private:
	int _leakPin;
	int _trigPin;
	int _echoPin;
};

int readSerial();