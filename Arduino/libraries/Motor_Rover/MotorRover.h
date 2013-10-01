#ifndef __MOTORROVER_H
#define __MOTORROVER_H

class MotorRover {
public:
	/* Constructors */
	MotorRover(int EN, int CHL, int CHR);
	MotorRover(int EN, int CHL, int CHR, int DEBUG);

	/* Direction controls */
	void forward(long time);
	void backward(long time);
	void left(int deg);
	void right(int deg);
	void brake(int test);
	
	/* External control */
	long scaleTime;			// Set motor Move ratio
	long scaleDegree;		// Set motor Turn ratio

private:
	/* Initialisation routine */
	void initMotorRoverClass(int EN, int CHL, int CHR);

	/* Arming routine */
	void arm();

	/* Disarming routine */
	void disarm();

	/* Timer */
	//long previousMillis;

	/* Pins */
	int _pinEN;
	int _pinCHL;
	int _pinCHR;

	/* Emergency brake */
	int _brake;				

	/* Debug Status */
	int _DEBUGSTAT;
};
#endif
