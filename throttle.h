///////////////////////////////
// throttle.h - Einlesen der Internetsteuerung sowie Arm und Disarm Funktion
// Emmanuel Küpfer
///////////////////////////////

//verhindern, dass Datei throttle.h mehr als einmal eingebunden wird.
#ifndef THROTTLE_H
#define THROTTLE_H

// Einbindung des benötigten Headerfile
#include "startup_calibration.h"

//extern da in thottle.cpp definiert und da von multicopter.ino gelesen wird
extern struct Position startwert;
extern boolean armed;

//"extern": Variabel ist im throttle.cpp bereits definiert.
extern float guiRoll;
extern float guiPitch;
extern float guiYaw;
extern float guiThrottle;

// Deklaration der Funktion der Datei throttle.cpp
int controlMulticopter(String command);

int speedMulticopter(String speedCommand);

int moveMulticopter(String moveCommand);

void setupMotorsAll();

void updateMotoresAll(float speedMotor1, float speedMotor2, float speedMotor3, float speedMotor4);

void initMotors();

#endif
