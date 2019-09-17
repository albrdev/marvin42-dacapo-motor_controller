#ifdef DEBUG

#include "ArduinoMotorShieldR3.h"

// Constructors ////////////////////////////////////////////////////////////////
ArduinoMotorShieldR3::ArduinoMotorShieldR3() { }

ArduinoMotorShieldR3::ArduinoMotorShieldR3(unsigned char DIR_A, unsigned char BRK_A, unsigned char PWM_A, unsigned char CS_A, unsigned char DIR_B, unsigned char BRK_B, unsigned char PWM_B, unsigned char CS_B) { }

// Public Methods //////////////////////////////////////////////////////////////
void ArduinoMotorShieldR3::init() {}

void ArduinoMotorShieldR3::setM1Speed(int speed) {}

// Set speed for motor 2, speed is a number betwenn -400 and 400
// Motor B == Motor B
void ArduinoMotorShieldR3::setM2Speed(int speed) { }

// Set speed for motor 1 and 2
void ArduinoMotorShieldR3::setSpeeds(int m1Speed, int m2Speed) { }

// Brake motor 1
void ArduinoMotorShieldR3::setM1Brake() { }

// Brake motor 2
void ArduinoMotorShieldR3::setM2Brake() { }

// Brake motor 1 and 2, brake is a number between 0 and 400
void ArduinoMotorShieldR3::setBrakes() { }

// TODO: check this calculation
// Return motor 1 current value in milliamps.
unsigned int ArduinoMotorShieldR3::getM1CurrentMilliamps() { return 0U; }

// TODO: check this calculation
// Return motor 2 current value in milliamps.
unsigned int ArduinoMotorShieldR3::getM2CurrentMilliamps() { return 0U; }

#endif
