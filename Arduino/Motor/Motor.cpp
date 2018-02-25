#include "Arduino.h"
#include "Servo.h"
#include "Motor.h"

//for default motors
Motor::Motor(String type, int pin) {
	_type = type;
	_pin = pin;
	if (_type == "T100") { 
		_maxPWM = 1900;
		_minPWM = 1100;
		_stopPWM = 1500;
	}
	else if (_type == "EMAX") {
		_maxPWM = 2000;
		_minPWM = 1000;
		_stopPWM = 1000;
	}
	else {
		Serial.println("Error");
	}
}

//for custom motors
Motor::Motor(int minPWM, int maxPWM, int stopPWM, int pin) {
	_type = "Custom";
	_motor.attach(pin);
	_maxPWM = maxPWM;
	_minPWM = minPWM;
	_stopPWM = stopPWM;
}

void Motor::init() {
	_motor.attach(_pin);
	_motor.writeMicroseconds(_stopPWM);
}

void Motor::writePWM(int pwm) {
	_pwm = pwm;
	if (_pwm >= _minPWM && _pwm <= _maxPWM) {
		_motor.write(_pwm);
	}
}

int Motor::readPWM() {
	return _pwm;
}

void Motor::stop() {
	_motor.write(_stopPWM);
}

String Motor::getType() {
	return _type;
}

int Motor::getMaxPWM() {
	return _maxPWM;
}

int Motor::getMinPWM() {
	return _minPWM;
}

int Motor::getStopPWM() {
	return _stopPWM;
}

int Motor::getPin(){
	return _pin;
}