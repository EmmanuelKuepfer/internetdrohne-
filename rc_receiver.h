///////////////////////////////
// rc_receiver.h - Einlesen des PWM RC Empfänger
// Emmanuel Küpfer
///////////////////////////////

// verhindern, dass Datei rc_receiver.h mehr als einmal eingebunden wird.
#ifndef rc_receiver_h
#define rc_receiver_h

// Deklaration der Funktion der Datei rc_receiver.cpp
void rcReceiverSetup();

//"extern": Variabel ist im rc_receiver.cpp bereits definiert.
extern volatile int rcPitch;
extern volatile int rcRoll;
extern volatile int rcYaw;
extern volatile int rcThrottle;
extern volatile int rcArm;

#endif
