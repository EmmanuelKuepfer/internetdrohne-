///////////////////////////////
// rc_receiver.cpp - Einlesen des PWM RC Empfänger
// Emmanuel Kuepfer
/////////////////////////////

// http://rcarduino.blogspot.ch/2012/04/how-to-read-multiple-rc-channels-draft.html
// Schlüsselwort volatile
// http://openbook.rheinwerk-verlag.de/c_von_a_bis_z/009_c_funktionen_010.htm#mj347a8bbe7206e189cfc3764ade226922

// Einbindung des benötigten Headerfile
#include "application.h"

int startPitch;
int startRoll;
int startYaw;
int startThrottle;
int startArm;

// Damit Variablen bei einer interuptmethode (ISR) korrekt übergeben,
// Deklaration mit volatile
volatile int rcPitch = 0;
volatile int rcRoll = 0;
volatile int rcYaw = 0;
volatile int rcThrottle = 0;
volatile int rcArm = 0;

void interruptPinA0();
void interruptPinA1();
void interruptPinA2();
void interruptPinA4();
void interruptPinA6();

//  Initialisierung RC Empfängerpins
void rcReceiverSetup()
{
  attachInterrupt(A0, interruptPinA0, CHANGE);
  attachInterrupt(A1, interruptPinA1, CHANGE);
  attachInterrupt(A2, interruptPinA2, CHANGE);
  attachInterrupt(A4, interruptPinA4, CHANGE);
  attachInterrupt(A6, interruptPinA6, CHANGE);
}


// Kanal CH1 Pin A0, roll
// Abweichung vom Mittelwert 1500,
// weil sich die Fersteurungsgimbal in der
// Mittelposition befinden.
void interruptPinA0()
{

  if (digitalRead(A0) == HIGH)
  {
    // start Zeit PWM Pulsbreite HIGH
    startRoll = micros();
  }
  else
  {
    // Ende Zeit PWM Pulsbreite HIGH
    rcRoll = micros() - startRoll - 1500;
  }
}


// Kanal CH2 Pin A1, pitch
// Abweichung vom Mittelwert 1500
void interruptPinA1()
 {
  if (digitalRead(A1) == HIGH)
  {
    //Start Zeit PWM Pulsbreite HIGH
    startPitch = micros();
  }
  else
  {
    // Ende Zeit PWM Pulsbreite HIGH
    rcPitch = micros() - startPitch - 1500;
  }
}

// Kanal CH3 Pin A2, throttle
// Abweichung vom Mittelwert 1500
void interruptPinA2()
{
  if (digitalRead(A2) == HIGH)
  {
    //Start Zeit PWM Pulsbreite HIGH
    startThrottle = micros();
  }
  else
  {
    // Ende Zeit PWM Pulsbreite HIGH
    rcThrottle = micros() - startThrottle - 1000;
  }
}

// Kanal CH4 Pin A3, yaw
// Abweichung vom Nullwert 1000
void interruptPinA4()
{
  if (digitalRead(A4) == HIGH)
  {
    //Start Zeit PWM Pulsbreite HIGH
    startYaw = micros();
  }
   else
  {
    // Ende Zeit PWM Pulsbreite HIGH
    rcYaw = micros() - startYaw - 1500;
  }
}

// Kanal CH5 Pin A4, arm
// Abweichung vom Nullwert 1000
void interruptPinA6()
{
  if (digitalRead(A6) == HIGH)
  {
    //Start Zeit PWM Pulsbreite HIGH
    startArm = micros();
  }
  else
  {
    // Ende Zeit PWM Pulsbreite HIGH
    rcArm = micros() - startArm;
  }
}
