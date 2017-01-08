///////////////////////
// startup_calibration.h - Bestimmung der Startwerte für Pitch, Roll und Yaw
// startup Calibration
///////////////////////

// verhindern, dass Datei startup_calibration.h mehr als einmal eingebunden wird.
#ifndef STARTUP_CALIBRATION_H
#define STARTUP_CALIBRATION_H

// c++ Fnuntion "struct" mit dem Namen Position
struct Position
{
  float pitch;
  float roll;
  float yaw;
};

// Deklaration der Funktionen der Datei startup_calibration.cpp
struct Position kalibrierung();

#endif
