#include "Arduino.h"
#include "Servo.h"

class Motor {
public:
	Motor(String type, int pin);
	Motor(int minPWM, int maxPWM, int stopPWM, int pin);
	void init();
	void writePWM(int pwm);
	int readPWM();
	void stop();
	String getType();
	int getMaxPWM();
	int getMinPWM();
	int getStopPWM();
	int getPin();
	
private:
	String _type;
	Servo _motor;
	int _pin;
	int _pwm;
	int _maxPWM;
	int _minPWM;
	int _stopPWM;
};