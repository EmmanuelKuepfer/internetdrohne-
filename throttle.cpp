///////////////////////////////
// throttle.cpp - Einlesen der Internetsteuerung sowie Arm und Disarm Funktion
// Emmanuel Küpfer
///////////////////////////////

// Einbindung der benötigten Headerfiles
#include "application.h"
#include "startup_calibration.h"
#include "throttle.h"
#include "pid.h"

//Position = Typ, startwert = Variabel, struct muss übernommen werden
//Globale Variabel muss static sein, falls nicht im ino File?
struct Position startwert;
boolean armed = false;

float guiRoll = 0;
float guiPitch = 0;
float guiYaw = 0;
float guiThrottle = 0;


//"int speedMulticopter(String speedComamnd)" wird aufgerufen durch Particle Cloud
// Nach Buch Make: Getting Started with the Photon, Simon Monk
// https://github.com/simonmonk/photon_book/blob/master/firmware/examples/p_06_LED_Function.cpp

// Arm (Aktivierung) Funktion
int controlMulticopter(String command)
{
  Serial.print("controlMulticopter ");
  Serial.println(command);

    if (command=="on")
    {
        armed = true;
        digitalWrite(D7, HIGH);
        startwert = kalibrierung();
        initMotors();
        initPID ();
        return 1;
    }
    else if (command=="off")
    {
        armed = false;
        digitalWrite(D7, LOW);
        guiThrottle = 0;
        return 0;
    }
    else
    {
        return -1;
    }
}


// wird von Particle Cloud aufgerufen
// Nach Buch Make: Getting Started with the Photon, Simon Monk
// https://github.com/simonmonk/photon_book/blob/master/firmware/examples/p_06_LED_Function.cpp
int speedMulticopter(String speedCommand)
{
    Serial.print("speedMulticopter ");
    Serial.println(speedCommand);

    //"atoi" = "asccii", wandelt Zahlen in Stings um
    guiThrottle = guiThrottle + atoi(speedCommand);
    return 1;
}

// wird von Particle Cloud aufgerufen
// Nach Buch Make: Getting Started with the Photon, Simon Monk
// https://github.com/simonmonk/photon_book/blob/master/firmware/examples/p_06_LED_Function.cpp
int moveMulticopter(String moveCommand)
{
    Serial.print("moveMulticopter ");
    Serial.println(moveCommand);

    if (moveCommand.equals("forward")) {
      guiPitch = guiPitch - 5;
    } else if (moveCommand.equals("back")) {
      guiPitch = guiPitch + 5;
    } else if (moveCommand.equals("left")) {
      guiRoll = guiRoll - 5;
    } else if (moveCommand.equals("right")) {
      guiRoll = guiRoll + 5;
    } else if (moveCommand.equals("rotateLeft")) {
      guiYaw = guiYaw - 5;
    } else if (moveCommand.equals("rotateRight")) {
      guiYaw = guiYaw + 5;
    } else if (moveCommand.equals("stop")) {
      guiRoll = 0;
      guiPitch = 0;
      guiYaw = 0;
    }
    return 1;
}

//Servo = Typ, motor = Variabel
void updateMotoresAll(float speedMotor1, float speedMotor2, float speedMotor3, float speedMotor4)
{
  if (armed) //armed
  {
      // analogWrite(Pin, Bereich 0 bis 255, Frequenz)
      // speedMotor1-4 werden durch 9.803... geteilt damit
      // sie im bereich von 0 bis 255 sind, wobei 1000 muss
      // auf 102 umgerechnet werden für Standschub.
      // 9.803... = 1000 / 102
      analogWrite(D2, speedMotor1 / 9.80392156863, 400);
      analogWrite(TX, speedMotor2 / 9.80392156863, 400);
      analogWrite(D3, speedMotor3 / 9.80392156863, 400);
      analogWrite(RX, speedMotor4 / 9.80392156863, 400);
  }
  else
  {
    // not armed
    // Motoren werden 1ms Pulsbreite beschrieben,
    // für Stillstand.
    analogWrite(D2, 100, 400);
    analogWrite(TX, 100, 400);
    analogWrite(D3, 100, 400);
    analogWrite(RX, 100, 400);
  }
}

// Initialisierung Motoren PWM Pins.
// Pulsweite 1ms fuer Stillstand.
void initMotors()
{
  //pinMode(Pin,Modus) Definition ob Pin Input oder Output
  pinMode(D2, OUTPUT);
  pinMode(TX, OUTPUT);
  pinMode(D3, OUTPUT);
  pinMode(RX, OUTPUT);

  // analogWrite(Pin, Bereich 0 bis 255, Frequenz)
  // für eine Millisekunde Pulsbreite bei 400Hz mit Peridendauer von2.5 ms
  // ist value 102 = 255 / 2.5 ms.
  analogWrite(D2, 100, 400);
  analogWrite(TX, 100, 400);
  analogWrite(D3, 100, 400);
  analogWrite(RX, 100, 400);
}
