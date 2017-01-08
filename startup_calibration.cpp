///////////////////////
// startup_calibration.cpp - Bestimmung der Startwerte für Pitch, Roll und Yaw
// Emmanuel Küpfer
///////////////////////

// Einbindung der benötigten Headerfiles
#include "startup_calibration.h"
#include "MPU9250.h"
#include "sensor.h"

// Berechnung eines Mittelwerts
struct Position kalibrierung()
{
  float summe1 = 0, summe2 = 0, summe3 = 0;
  float mittelwert_pitch, mittelwert_roll, mittelwert_yaw;

  float pitch, roll, yaw;

  struct Position mittelwert;
  int i = 0;

  while (i < 10)
  {
    // Wenn der Sensor bereit ist wird er abgefragt.
    if (sensor.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
    {
      sensorUpdate();
      pitch = sensor.pitch;
      roll = sensor.roll;
      yaw = sensor.yaw;

      summe1 = summe1 + pitch;
      summe2 = summe2 + roll;
      summe3 = summe3 + yaw;
      i++;
    }
  }

  mittelwert.pitch = summe1 / 10;
  mittelwert.roll = summe2 / 10;
  mittelwert.yaw = summe3 / 10;

  //Rückgabe der Strucktur
  return mittelwert;
}
