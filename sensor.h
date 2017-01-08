///////////////////////////////
// sensor.h - Berechnung der Winkelwerte anhand der Sensorwerte
// Emmanuel Kuepfer
///////////////////////////////

// verhindern, dass Datei sensor.h mehr als einmal eingebunden wird.
#ifndef sensor_h
#define sensor_h

#include "MPU9250.h"

// MPU9250 ist eine c++ Klasse, das in der Library MPU9250 definiert ist.
// "extern": Variabel ist im sensor.cpp bereits definiert.
extern MPU9250 sensor;

// Deklaration der Funktionen der Datei sensor.cpp
void sensorInit();

void sensorUpdate();

#endif
