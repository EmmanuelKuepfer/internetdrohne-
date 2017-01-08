///////////////////////
// multicopter_code.ino - Hauptimplementation
// Emmanuel Küpfer
///////////////////////

// System Headerfiles
#include "MPU9250.h"
#include "quaternionFilters.h"
#include "math.h"
#include "application.h"

// Eigene Headerfiles
#include "pid.h"
#include "sensor.h"
#include "startup_calibration.h"
#include "error.h"
#include "throttle.h"
#include "rc_receiver.h"

#define PRINT_COUNT 20

void printPlus(float value);

float ax, ay, az;
float pitch, roll, yaw;
float pidPitch, pidRoll, pidYaw;
float rcPitchCopy, rcRollCopy, rcYawCopy, rcThrottleCopy, rcArmCopy, rcArmLast;
float rcPitchStartwert, rcRollStartwert, rcYawStartwert;
float errorPitch, errorRoll, errorYaw;
float throttle;
// Particle variable muss Type double sein
double throttleVal = 0;
float speedMotor1, speedMotor2, speedMotor3, speedMotor4;
long now;
float dt;
long lastUpdate;

int printCount = 0;


void setup()
{
  //Particle.function verbindet Photon mit Particle Cloud d.h. mit dem Web GUI
  Particle.function("arm", controlMulticopter);
  Particle.function("throttle", speedMulticopter);
  Particle.function("move", moveMulticopter);
  Particle.variable("trottle_val", throttleVal);

  Serial.begin(115200);

  sensorInit();

  // benötigt nach der Initialisierung damit ein gültiger Wert gelesen werden kann.
  delay(3000);

  // TODO Bei Ausführen
  startwert = kalibrierung();
  lastUpdate = micros();

  Serial.print("Startwert Pitch ");   Serial.print(startwert.pitch, 2);
  Serial.print(" Roll ");   Serial.print(startwert.roll, 2);
  Serial.print(" Yaw ");    Serial.println(startwert.yaw, 2);

  rcReceiverSetup();
  initMotors();
}

//Beginn der Programmschlaufe
void loop()
{

  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (sensor.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    // Lesen aktuelle Sensorwerte
    sensorUpdate();

    pitch = sensor.pitch;
    roll = sensor.roll;
    yaw = sensor.yaw;

    // SYNTAX
    ATOMIC_BLOCK()
    {
    // code here is executed atomically, without task switching or interrupts
      rcThrottleCopy = rcThrottle;

      // RC Ausschläge sind zu stark, durch 10 teilen
      rcPitchCopy = rcPitch / 10;
      rcRollCopy = rcRoll / 10;
      rcYawCopy = rcYaw / 10;
      rcArmCopy = rcArm;
    }

    // Sicherstellen dass arm oder dis-arm nur auferufen wird bei einem Wechsel
    // vom Kippschalter der Fernsteuerung
    if (rcArmCopy > 1500 && rcArmLast < 1500)
    {
      controlMulticopter("on");
      // Abweichnung der Fernsteuerung Kanäle von der Mitte bei Arm
      rcPitchStartwert = rcPitchCopy;
      rcRollStartwert = rcRollCopy;
      rcYawStartwert = rcYawCopy;
    }
    if (rcArmCopy < 1500 && rcArmLast > 1500)
    {
      controlMulticopter("off");
    }
    rcArmLast = rcArmCopy;

    // Rc Anlage liefert keine genauen Mittelwerte bei Mittelstellung der
    // Steuergimbals, deshalb werden die Variabeln rcxxxStartwert der RC
    // Anlage von den Variabel rcxxxCopy subtrahiert.
    rcPitchCopy = rcPitchCopy - rcPitchStartwert;
    rcRollCopy = rcRollCopy - rcRollStartwert;
    rcYawCopy = rcYawCopy - rcYawStartwert;

    // Aufrufen der Funktion zur Fehlerberechnung
    // pitch = ist_wert, startwert.pitch = soll_wert
    errorPitch = berechne_E_t_(pitch, startwert.pitch);
    errorRoll = berechne_E_t_(roll, startwert.roll);
    // errorYaw = berechne_E_t_(yaw, startwert.yaw);
    // für Yaw wird nur die Drehgeschrwindigkeit vom Sensor berücksichtigt
    errorYaw = sensor.gz / 10;

    // Berechnung Zeitdifferenz: dt
    now = micros();
    dt = (now - lastUpdate) / 1000.0;
    lastUpdate = now;

    // Berechnung PID Werte
    // Zum error weden die Werte der Steuerung vom GUI und der Fernsteuerung gezählt
    pidPitch = berechne_PID(errorPitch + guiPitch + rcPitchCopy, dt, PID_PITCH);
    pidRoll = berechne_PID(errorRoll + guiRoll + rcRollCopy, dt, PID_ROLL);
    pidYaw = berechne_PID(errorYaw + guiYaw + rcYawCopy, dt, PID_YAW);

    // Geschwindigkeitswertzuweisung, 990 ist kein Gas
    throttle = 990 + guiThrottle + rcThrottleCopy;

    // Particle variable vom Type double für Tacho Anzeige
    throttleVal = throttle;

    // Berechnung Motordrehzahl
    speedMotor1 = throttle - pidPitch + pidRoll - pidYaw;
    speedMotor2 = throttle + pidPitch + pidRoll + pidYaw;
    speedMotor3 = throttle - pidPitch - pidRoll + pidYaw;
    speedMotor4 = throttle + pidPitch - pidRoll - pidYaw;

    // Aufruf der Funktion um auf Motoren zu schreiben
    updateMotoresAll(speedMotor1, speedMotor2, speedMotor3, speedMotor4);

    // Ausgeabe der Steuerwerte, etwa alle 100ms (5 ms looptime * 20)
    if (printCount >= PRINT_COUNT)
    {
      Serial.print(" dt ");                                     Serial.print(dt, 2);
      Serial.print(" P ");          printPlus(pitch);           Serial.print(pitch, 2);
      Serial.print(" R ");          printPlus(roll);            Serial.print(roll, 2);
      Serial.print(" Y ");          printPlus(yaw);             Serial.print(yaw, 2);

      Serial.print("   ERROR P ");  printPlus(errorPitch);      Serial.print(errorPitch, 2);
      Serial.print(" R ");          printPlus(errorRoll);       Serial.print(errorRoll, 2);
      Serial.print(" Y ");          printPlus(errorYaw);        Serial.print(errorYaw, 2);

      Serial.print(" T ");                                      Serial.print(throttle, 2);

      Serial.print("   PID P ");    printPlus(pidPitch);        Serial.print(pidPitch, 2);
      Serial.print(" R ");          printPlus(pidRoll);         Serial.print(pidRoll, 2);
      Serial.print(" yaw ");        printPlus(pidYaw);          Serial.print(pidYaw, 2);

      Serial.print("   M1 ");                                   Serial.print(speedMotor1, 0);
      Serial.print(" M2 ");                                     Serial.print(speedMotor2, 0);
      Serial.print(" M3 ");                                     Serial.print(speedMotor3, 0);
      Serial.print(" M4 ");                                     Serial.println(speedMotor4, 0);

      Serial.print("   RC P ");                                 Serial.print(rcPitchCopy, 0);
      Serial.print(" R ");                                      Serial.print(rcRollCopy, 0);
      Serial.print(" Y ");                                      Serial.print(rcYawCopy, 0);
      Serial.print(" T ");                                      Serial.print(rcThrottleCopy, 0);
      Serial.print(" A ");                                      Serial.println(rcArmCopy, 0);

      printCount = 0;
    }
    else
    {
      printCount++;
    }
  }
}

// Damit Anzeige von negativen und positiven Werten gleich lang ist
void printPlus(float value)
{
  if (value >=0)
  {
    Serial.print ("+");
  }
}
