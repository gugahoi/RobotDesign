#ifndef ARDUINO_CTRL_H
#define ARDUINO_CTRL_H
/*
 * File: arduino_ctrl.h
 * Author: Ian Hua (c) 2013
 * Requirements:
 *     1) Need to have I2C for Raspberry Pi enabled.
 *     2) Need to have GNUBlin installed.
 * Use: Sends messages to an I2C device.
 */


/*
 * Includes:
 */
#define BOARD RASPBERRY_PI // Must be placed before gnublin.h
#include "./External/gnublin.h"

//#define DEBUG


#define ARDUINO_I2C_ADDRESS 0x04



void ctrlInit(void);



void statusCheck();




typedef enum {BRAKE, FORWARD, BACKWARD, LEFTSMALL, LEFTBIG, RIGHTSMALL, RIGHTBIG} motorCommand;

void ctrlMotor(motorCommand command);
void ctrlMotorScaleTime(int scale);
void ctrlMotorScaleDegree(int scale);



typedef enum {OPEN, CLOSE} servoGate;
typedef enum {LEFT, RIGHT, BOTH} servoLocation;
void ctrlServo(servoLocation location, servoGate gate);

typedef enum {FRONT, BALL, FLOOR} servoCamera;
void ctrlServoCamera(servoCamera angle);


typedef enum {PUSH, PULL, KICK} solenoidRod;
void ctrlSolenoid(solenoidRod rod);



/*
int main(int argc, char **argv)
{


	unsigned char input[8];

	#ifndef DEBUG
	while (1) {
		cout << "Please enter a command: ";
		gets(reinterpret_cast<char*>(input));

		while (strlen(reinterpret_cast<const char*>(input)) > 8) {
			cout << "ERROR! Buffer overflow. Please try again." << endl << endl;
			cout << "Please enter a command: ";
			gets(reinterpret_cast<char*>(input));
		}

		cout << "You entered: " << input << endl << endl;
	#else
	gets(reinterpret_cast<char*>(input));

	cout << "Sending: " << input << " continuously" << endl;
	while (1) {
		i2c.send(input, strlen(reinterpret_cast<const char*>(input)));
	}
	#endif

	#ifndef DEBUG
		if (atoi(reinterpret_cast<const char*>(input)) == 0) {
			if (strlen(reinterpret_cast<const char*>(input)) < 2) {
				statusCheck();
			} else {
				cout << "ERROR! Invalid status check command." << endl;
			}
		} else {
			if ((atoi(reinterpret_cast<const char*>(input)) < 100)) {
				if (_checked || atoi(reinterpret_cast<const char*>(input)) == 13) {
					i2c.send(input, strlen(reinterpret_cast<const char*>(input)));
					i2c.send(input, strlen(reinterpret_cast<const char*>(input)));
					cout << "Command sent twice" << endl;
					_checked = 0;
				} else {
					i2c.send(input, strlen(reinterpret_cast<const char*>(input)));
				}
			} else {
				cout << "ERROR! Command invalid" << endl;
			}
		}
	}
	#endif
}
*/

#endif
