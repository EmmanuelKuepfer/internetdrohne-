///////////////////////////////
// pid.h - Berechnung der PID Werte für alle Achsen
// Emmanuel Küpfer
///////////////////////////////

//verhindern, dass Datei pid.h mehr als einmal eingebunden wird.
#ifndef PID_H
#define PID_H

// Werte für berechnete PID Achsen
#define PID_PITCH 0
#define PID_ROLL  1
#define PID_YAW   2

// Initialisiere PID
void initPID ();

// Deklaration PID Berechnung
float berechne_PID(float E_t_, float dt, int achse);

#endif
