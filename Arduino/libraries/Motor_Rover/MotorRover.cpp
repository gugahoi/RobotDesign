#include "Arduino.h"
#include <Time.h>

#include "MotorRover.h"

/* Constructor - without DEBUG */
MotorRover::MotorRover(int EN, int CHL, int CHR) {
  initMotorRoverClass(EN, CHL, CHR);
  _DEBUGSTAT = 0;
}

/* Constructor - with DEBUG */
MotorRover::MotorRover(int EN, int CHL, int CHR, int DEBUG) {
  initMotorRoverClass(EN, CHL, CHR);
  _DEBUGSTAT = 1;
}

/* Initialisation routine */
void MotorRover::initMotorRoverClass(int EN, int CHL, int CHR) {
  _pinEN  = EN;
  _pinCHL = CHL;
  _pinCHR = CHR;
  
  pinMode(_pinEN,   OUTPUT);
  pinMode(_pinCHL,  OUTPUT);
  pinMode(_pinCHR,  OUTPUT);
  
  scaleTime = 1000;
  scaleDegree = 6;

 digitalWrite(_pinEN, LOW);          // Disable and de-energise ALL motors
  _brake = 1;                         // Enable brakes
  //digitalWrite(_pinEN, HIGH);
  //_brake = 0;
}

/* Arming routine */
void MotorRover::arm() {
  digitalWrite(_pinEN, HIGH);           // Enable ALL motors
}

/* Disarming routine */
void MotorRover::disarm() {
  digitalWrite(_pinEN, LOW);          // Disable & De-energise ALL Motor
}

/* Emergency brake */
void MotorRover::brake(int test) {
  switch (test) {
    case 0:
      _brake = 0;
    break;
    default:
        _brake = 1; 
        disarm();
    break;
  }
}

/* FORWARD */
void MotorRover::forward(long time) {
  if (_DEBUGSTAT != 0)
    Serial.println("Going Forward");  // Debug: Output current driver direction

  digitalWrite(_pinCHL, LOW);         // Set Left Motor to rotate forward

  digitalWrite(_pinCHR, LOW);         // Set Right Motor to rotate forward

  //previousMillis = millis();          // Begin timing

  arm();

  /*
  if (time != 99 && !_brake) {
   while((millis() - previousMillis < (time*scaleTime)) && (!_brake)); // Keep motors running for time
  } else {
    while(!_brake);
  }
  */

  delay(time*scaleTime);
  disarm();

  if (_DEBUGSTAT != 0)
    Serial.println("Finished Forward"); // Debug: Output forward routine finished
}

/* BACKWARD */
void MotorRover::backward(long time) {
  if (_DEBUGSTAT != 0)
    Serial.println("Going Backwards");  // Debug: Output current driver direction

  digitalWrite(_pinCHL, HIGH);          // Set Left Motor to rotate backwards

  digitalWrite(_pinCHR, HIGH);           // Set Right Motor to rotate backwards

  //previousMillis = millis();            // Begin timing

  arm();

  /*
  while((millis() - previousMillis < (time*scaleTime)) && (!_brake)); // Keep motors running for 3 seconds

  disarm();
  */

  delay(time*scaleTime);
  disarm();

  if (_DEBUGSTAT != 0)
    Serial.println("Finished Backwards"); // Debug: Output backwards routine finished
}

/* LEFT */
void MotorRover::left(int deg) {
  if (_DEBUGSTAT != 0)
    Serial.println("Going Left"); // Debug: Output current driver direction

  digitalWrite(_pinCHL, LOW);     // Set Left Motor to rotate backwards

  digitalWrite(_pinCHR, HIGH);    // Set Right Motor to rotate forwards

  //previousMillis = millis();      // Begin timing

  arm();

  /*
  while((millis() - previousMillis < (int) (deg*scaleDegree)) && (!_brake)); // Keep motors running for 3 seconds

  disarm();
  */
  delay(deg*scaleDegree);
  disarm();

  if (_DEBUGSTAT != 0)
    Serial.println("Finished Left"); // Debug: Output left routine finished
}

/* RIGHT */
void MotorRover::right(int deg) {
  if (_DEBUGSTAT != 0)
    Serial.println("Going Right"); // Debug: Output current driver direction

  digitalWrite(_pinCHL, HIGH);     // Set Left Motor to rotate backwards

  digitalWrite(_pinCHR, LOW);      // Set Right Motor to rotate forwards

  //previousMillis = millis();       // Begin timing

  arm();
  
  /*
  while((millis() - previousMillis < (int) (deg*scaleDegree)) && (!_brake)); // Keep motors running for 3 seconds

  disarm();
  */
  delay(deg*scaleDegree);
  disarm();

  if (_DEBUGSTAT != 0)
    Serial.println("Finished Right"); // Debug: Output right routine finished
}
