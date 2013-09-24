#include "arduino_ctrl.h"

#define BOARD RASPBERRY_PI // Must be placed before gnublin.h
#include "./External/gnublin.h"

#include <iostream>
#include <string>
#include <sstream>

using namespace std;

gnublin_i2c i2c;


int _checked;

void ctrlInit(void)
{
	i2c.setAddress(ARDUINO_I2C_ADDRESS);

	_checked = 0;

	cout << "ROBOT READY!" << endl << endl;
}




unsigned char statusMode[2];
unsigned char statusBusy[2];
unsigned char statusWait[2];
unsigned char statusScaleTime[2];
unsigned char statusScaleDegree[2];

void statusCheck()
{
	i2c.receive(statusMode, 1);
	int mode = atoi(reinterpret_cast<const char*>(statusMode));
	i2c. receive(statusBusy, 1);
	int busy = atoi(reinterpret_cast<const char*>(statusBusy));
	i2c.receive(statusWait, 1);
	int wait = atoi(reinterpret_cast<const char*>(statusWait));
	i2c.receive(statusScaleTime, 1);
	int scaleTime = atoi(reinterpret_cast<const char*>(statusScaleTime));
	i2c.receive(statusScaleDegree, 1);
	int scaleDegree = atoi(reinterpret_cast<const char*>(statusScaleDegree));

	_checked = 1;

	cout << "Arduino response: " << mode << " " << busy << " " << wait << " " << scaleTime << " " << scaleDegree << endl;
}



unsigned char _inputBrake[8] = "13";
unsigned char _inputForward[8] = "1 1";
unsigned char _inputBackward[8] = "1 1";
unsigned char _inputLeft20[8] = "3 20";
unsigned char _inputLeft90[8] = "3 90";
unsigned char _inputRight20[8] = "4 20";
unsigned char _inputRight90[8] = "4 90";

void ctrlMotor(motorCommand command)
{
	switch (command) {
		case BRAKE:
			i2c.send(reinterpret_cast<unsigned char*>(_inputBrake), strlen(reinterpret_cast<const char*>(_inputBrake)));
			i2c.send(reinterpret_cast<unsigned char*>(_inputBrake), strlen(reinterpret_cast<const char*>(_inputBrake)));
		break;
		case FORWARD:
			i2c.send(reinterpret_cast<unsigned char*>(_inputForward), strlen(reinterpret_cast<const char*>(_inputForward)));
		break;
		case BACKWARD:
			i2c.send(reinterpret_cast<unsigned char*>(_inputBackward), strlen(reinterpret_cast<const char*>(_inputBackward)));
		break;
		case LEFTSMALL:
			i2c.send(reinterpret_cast<unsigned char*>(_inputLeft20), strlen(reinterpret_cast<const char*>(_inputLeft20)));
		break;
		case LEFTBIG:
			i2c.send(reinterpret_cast<unsigned char*>(_inputLeft90), strlen(reinterpret_cast<const char*>(_inputLeft90)));
		break;
		case RIGHTSMALL:
			i2c.send(reinterpret_cast<unsigned char*>(_inputRight20), strlen(reinterpret_cast<const char*>(_inputRight20)));
		break;
		case RIGHTBIG:
			i2c.send(reinterpret_cast<unsigned char*>(_inputRight90), strlen(reinterpret_cast<const char*>(_inputRight90)));
		break;
		default:
			cout << "ERROR! Invalid command: ctrlMotor called" << endl;
		break;
	}
}




unsigned char _inputServoLeftOpen[8] = "5 2";
unsigned char _inputServoLeftClose[8] = "5 1";
unsigned char _inputServoRightOpen[8] = "6 2";
unsigned char _inputServoRightClose[8] = "6 1";

void ctrlServo(servoLocation location, servoGate gate)
{
	switch (location) {
		case BOTH:
			if (gate == CLOSE) {
				i2c.send(reinterpret_cast<unsigned char*>(_inputServoLeftClose), strlen(reinterpret_cast<const char*>(_inputServoLeftClose)));
				i2c.send(reinterpret_cast<unsigned char*>(_inputServoRightClose), strlen(reinterpret_cast<const char*>(_inputServoRightClose)));
			} else {
				i2c.send(reinterpret_cast<unsigned char*>(_inputServoLeftOpen), strlen(reinterpret_cast<const char*>(_inputServoLeftOpen)));
				i2c.send(reinterpret_cast<unsigned char*>(_inputServoRightOpen), strlen(reinterpret_cast<const char*>(_inputServoRightOpen)));
			}
		break;
		case LEFT:
			if (gate == CLOSE) {
				i2c.send(reinterpret_cast<unsigned char*>(_inputServoLeftClose), strlen(reinterpret_cast<const char*>(_inputServoLeftClose)));
			} else {
				i2c.send(reinterpret_cast<unsigned char*>(_inputServoLeftOpen), strlen(reinterpret_cast<const char*>(_inputServoLeftOpen)));
			}
		break;
		case RIGHT:
			if (gate == CLOSE) {
				i2c.send(reinterpret_cast<unsigned char*>(_inputServoRightClose), strlen(reinterpret_cast<const char*>(_inputServoRightClose)));
			} else {
				i2c.send(reinterpret_cast<unsigned char*>(_inputServoRightOpen), strlen(reinterpret_cast<const char*>(_inputServoRightOpen)));
			}
		break;
		default:
			cout << "ERROR! Unknown servo position chosen: ctrlServo called" << endl;
		break;
	}
}


unsigned char _inputServoFront[8] = "7 90";
unsigned char _inputServoFloor[8] = "7 150";

void ctrlServoCamera(servoCamera angle)
{
	switch (angle) {
		case FRONT:
		i2c.send(reinterpret_cast<unsigned char*>(_inputServoFront), strlen(reinterpret_cast<const char*>(_inputServoFront)));
		break;
		case FLOOR:
		default:
		i2c.send(reinterpret_cast<unsigned char*>(_inputServoFloor), strlen(reinterpret_cast<const char*>(_inputServoFloor)));
	}
}



unsigned char _inputSolenoidPush[8] = "10 1";
unsigned char _inputSolenoidPull[8] = "10";
unsigned char _inputSolenoidKick[8] = "10 2";

void ctrlSolenoid(solenoidRod rod)
{
	switch (rod) {
		case PUSH:
			i2c.send(reinterpret_cast<unsigned char*>(_inputSolenoidPush), strlen(reinterpret_cast<const char*>(_inputSolenoidPush)));	
		break;
		case KICK:
			i2c.send(reinterpret_cast<unsigned char*>(_inputSolenoidKick), strlen(reinterpret_cast<const char*>(_inputSolenoidKick)));
			break;
		case PULL:
		default:
			i2c.send(reinterpret_cast<unsigned char*>(_inputSolenoidPull), strlen(reinterpret_cast<const char*>(_inputSolenoidPull)));
			break;
	}
}
