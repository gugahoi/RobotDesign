/* Rover Intelligent
 * File: Rover_Intelligent.ino
 * Description: Control code to intelligently control a Rover using I2C 
 * Author: Ian Hua 2013
 * Date modified: 1 October 2013
 */
 
//#define DEBUG
 
#include <string.h>
#include <stddef.h>
#include <Wire.h>
 
#include <MotorRover.h>
#include <Servo.h>
#include <SoftwareSerial.h>
 
SoftwareSerial softSerial(2, 3);
 
int command = 0;                    // Control command: Report status, Forward, Backward, Left, Right, Set Move Scale, Set Turn Scale
long param = 0;                     // Control command parameter: <N/A, seconds, seconds, Degrees, Degrees, int, int>
 
int message = 0;                    // Has a ready message been issued to Serial?
int dataIn = 0;                     // Has the I2C master issued a command?
 
/* CURRENTLY UNUSED */
int busy = 0;                       // Current Arduino state: Doing a command or not
int mode = 0;                       // Current Arduino state: Looking for ball, looking for goal, score a goal
int wait = 0;                       // Current Arduino state: How long is the Arduino in a busy state?
 
#ifdef DEBUG
MotorRover motors(9, 7, 8, 1);     // Initiate a motor device with DEBUG.
#else
MotorRover motors(9, 7, 8);        // Initiage a motor device without DEBBUG.
#endif
 
Servo servoGateRight;
Servo servoCamera;
 
const int solenoid = 4;
 
const int trigLeft = 12;
const int echoLeft = 11;
 
/*
 * Program entry point:
 */
void setup() {
  Serial.begin(115200);             // Faster serial Baud rate. Optimisation purpose.
  Serial.println("Connected to PC");
 
  motors.brake(1);
 
  softSerial.begin(9600);
  Serial.println("Connected to RPi");
 
  servoGateRight.attach(5);         // Attach left gate
  servoCamera.attach(6);            // Attach camera
 
  servoGateRight.write(80);         // Close right forward
 
  pinMode(solenoid, OUTPUT);        // Set pin to control solenoid  
  digitalWrite(solenoid, LOW);      // Set solenoid OFF
  digitalWrite(solenoid, LOW);      // Set solenoid OFF
 
  pinMode(trigLeft, OUTPUT);        // Set pin to trigger left ultrasonic sensor
  pinMode(echoLeft, INPUT);         // Set pin to receive trigger echo from left ultrasonic sensor
 
  motors.scaleTime = 200;           // Set motor driver conversion ratio for distance
  motors.scaleDegree = 6;           // Set motor driver conversion ratio for turn angle
}
 
int roam = 1;
void loop() {
  readyMessage(&message);           // Print to serial a ready message if ready.
  wallCheck();
 
  if (roam) {
    motors.forward(1);
    //time_forward;
    motors.brake(1);
    Serial.println("R ON");
  } 
  else {
    Serial.println("R OFF");
  }
 
  dataReceive();
 
  if (dataIn & !busy) {             // I2C master has issed a command, and Arduino is not busy.
    processCommand();               // Process command
    motors.brake(1);
  }
}
 
 
 
 
 
/*
 * Ultrasonic
 */
int avgLeft = 0;
 
int ultrasonicCheckLeft() {
  int readingLeft[3];
  int i;
  long durationLeft = 0, cmLeft = 0;
 
  for(i = 0; i < 3; i++) {
    digitalWrite(trigLeft, LOW);
    delayMicroseconds(2);
    digitalWrite(trigLeft, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigLeft, LOW);
 
    durationLeft = pulseIn(echoLeft, HIGH);
 
    // convert the time into a distance
    cmLeft = microsecondsToCentimeters(durationLeft);
 
    //Serial.println(cmLeft);
    //delay(1000);
 
    readingLeft[i] = cmLeft;
  }
 
  avgLeft = 0;
  for (i = 0; i < 3; i++) {
    avgLeft += readingLeft[i];
  }
 
  avgLeft /= 3;
 
  Serial.print("avgLeft: ");
  Serial.println(avgLeft);
 
} 
 
long microsecondsToCentimeters(long microseconds)
{
  return microseconds / 54;
}
 
int wallCheck()
{
  motors.brake(1);
 
  ultrasonicCheckLeft();
 
  if (avgLeft <= 0) {
    avgLeft = 5;
  } 
  else {
    avgLeft += 5;
  }
 
  if (avgLeft < 15) {
    motors.backward(1);
    motors.brake(1);
 
    motors.right(45);
    motors.brake(1);
  } 
  else {
    Serial.println("NO WALL");
  }
}  
 
 
 
/*
 * Interrupt handler: I2C master sends data to Arduino:
 */
void dataReceive() 
{
  char buffer[9]; 
  const char delimiters[] = " ,-;:.";
  int i = 0;
 
  if (softSerial.available() > 0 ){
    softSerial.readBytesUntil('\n', buffer, 9);
  }
 
  command = atoi(strtok(buffer, delimiters));
  param =  atoi(strtok(NULL, delimiters));
 
  dataIn = 1;  
 
  if (command == 13 || command == 0 ) {               // Activate E-Brake
    busy = 1;
    Serial.print("INTERRUPT BRAKE");
 
    motors.brake(1);
    motors.brake(1);
    command = 0;
    commandStateReady();
 
    dataIn = 0;
  }
 
  //processCommand();
  //commandStateReady();
 
  if (command == 99 || command == 991 || command == 992) {
    roam = 0;
  } 
  else if(command == 98 || command == 981 || command == 982) {
    roam = 1;
  } 
}
 
 
 
 
/* 
 * Interrupt handler: Report current state to I2C master.
 */
int index = 0;
char _mode[2] = "m";
char _busy[2] = "b";
char _wait[2] = "w";
char _motorScaleTime[2] = "t";
char _motorScaleDegree[2] = "d";
 
void dataRequest() 
{
  sprintf(_mode, "%d", mode);
  sprintf(_busy, "%d", busy);
  sprintf(_wait, "%d", wait);
  sprintf(_motorScaleTime, "%d", motors.scaleTime);
  sprintf(_motorScaleDegree, "%d", motors.scaleDegree);
 
  switch (index) {
  case 0:
    Wire.write(_mode);
    index++;
    break;
  case 1:
    Wire.write(_busy);
    index++;
    break;
  case 2:
    Wire.write(_wait);
    index++;
    break;
  case 3:
    Wire.write(_motorScaleTime);
    index++;
    break;
  case 4:
    Wire.write(_motorScaleDegree);
    index++;
    break;
  default:
    index = 0;
    break;
  }
 
#ifdef DEBUG
  Serial.print("Mode: "); 
  Serial.println(mode, DEC);
  Serial.print("Busy: "); 
  Serial.println(busy, DEC);
  Serial.print("Wait: "); 
  Serial.println(wait, DEC);
  Serial.print("scaleTime: "); 
  Serial.println(motors.scaleTime, DEC);
  Serial.print("scaleDegree: "); 
  Serial.println(motors.scaleDegree, DEC);
#endif
}
 
 
 
 
/*
 * Issue a ready message to Serial if the Arduino is ready, and none has been printed yet.
 */
void readyMessage(int *_test)
{
  switch (*_test) {
  case 0:
    Serial.println();  
    Serial.println("Rover ONLINE");   
    *_test = 1;
    break;
  default:
    break;
  }
}
 
 
 
 
/*
 * Process the command the I2C master has given the Arduino
 */
void processCommand()
{
  switch (command) {
  case 1:                                                        // Move Forward command
    busy = 1;
    wait = param;
    Serial.print("Forward ");  
 
#ifdef DEBUG
    Serial.print(param * motors.scaleTime / 1000, DEC); 
    Serial.println("sec");
#endif
 
    motors.forward(param);
    commandStateReady();
    break;
  case 2:                                                        // Move Backward command
    busy = 1;
    wait = param;
    Serial.print("Backward ");
 
#ifdef DEBUG
    Serial.print(param * motors.scaleTime / 1000, DEC); 
    Serial.println("sec");
#endif
 
    motors.backward(param);
    commandStateReady();
    break;
  case 3:                                                        // Turn Left command
    busy = 1;
    Serial.print("Left "); 
 
#ifdef DEBUG
    Serial.print(param, DEC); 
    Serial.println("deg");
#endif
 
    motors.left(param);
    commandStateReady();
    break;
  case 4:                                                        // Turn Right command
    busy = 1;
    Serial.print("Right "); 
 
#ifdef DEBUG
    Serial.print(param, DEC); 
    Serial.println("deg");
#endif
 
    motors.right(param);
    commandStateReady();
    break;
 
  case 6:                                                        // Right gate servo
    busy = 1;
 
    if (param == 1) {                                            // Close gate
      servoGateRight.write(80);                                  
    } 
    else if (param == 2) {                                       // Open gate
      servoGateRight.write(130);
    } 
    else {                                                     // Trap solenoid
      servoGateRight.write(80);   
    }      
    commandStateReady();
    break;
 
  case 7:                                                        // Camera servo
    busy = 1;
    if (param == 1) {                                            // Face FRONT
      servoCamera.write(90);
    } 
    else if (param == 2) {                                       // Face BALL
      servoCamera.write(130);
    } 
    else if (param == 3) {                                       // Face FLOOR
      servoCamera.write(150);
    } 
    else {                                                       // Face FLOOR
      servoCamera.write(130);
    }
    commandStateReady();
    break;
 
  case 8:                                                        // Set Move ratio: DEFAULT: 1000
    busy = 1;
    Serial.print("Set Move Ratio: ");
 
    if (param > 5000) {
      Serial.println("INVALID Move Ratio");
      commandStateReady();
      break;
    }
 
    Serial.print(param, DEC);
    Serial.println();
    motors.scaleTime = param;
    commandStateReady();
    break;
  case 9:                                                        // Set Turn ratio: DEFAULT: 6
    busy = 1;
    Serial.print("Set Turn Ratio: ");
 
    if (param > 20) {
      Serial.println("INVALID Turn Ratio!");
      commandStateReady();
      break;
    }
 
    Serial.print(param, DEC);
    Serial.println();
    motors.scaleDegree = param;
    commandStateReady();
    break; 
 
  case 10:                                                        // Solenoid
    busy = 1;
    if (param == 1) {                                             // KICK
      Serial.println("KICK");
 
      digitalWrite(solenoid, HIGH);
      digitalWrite(solenoid, HIGH);
      delay(250);
    } 
 
 
    digitalWrite(solenoid, LOW);                                  // SOLENOID OFF
    digitalWrite(solenoid, LOW);
    commandStateReady();
    break;
 
  case 98:
    busy = 1;
    roam = 1;
    commandStateReady();
    break;
 
  case 99:
    busy = 1;
    roam = 0;
    commandStateReady();
    break;
 
 
  default:                                                       
    Serial.println("Unknown Command!");
    motors.brake(1);
    commandStateReady();
    break;
  }
}
 
 
/*
 * Set all state flags to ready.
 */
void commandStateReady(void)
{
  message = 0;
  dataIn = 0;
  busy = 0;
  wait = 0;
}
