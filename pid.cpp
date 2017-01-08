///////////////////////////////
// pid.cpp - Berechnung der PID Werte für alle Achsen
// Emmanuel Küpfer
///////////////////////////////

// Einbindung des benötigten Headerfile
#include "application.h"

// Konstante
const float Kp = 2.5;
const float Ki = 0.05;
const float Kd = 30.0;

// Definition Variabeln
#define ERR_1 0
#define ERR_2 1
#define ERR_3 2

// Speicherung pro Achse E(t-1) bis E(t-3) für Berechnung des Differenzialanteil
static int E_t_1[3][3]   = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };

// Speicherung pro Achse I(t-1)
static float I_t_1[3]    = { 0, 0, 0 };

// Integral wird bei jedem aktivieren auf 0 gesetzt.
void initPID ()
{
  I_t_1[0] = 0;
  I_t_1[1] = 0;
  I_t_1[2] = 0;
}

// Berechne PID E_t_ Fehler E(t)
// dt = Zeitdifferenz seit letztem Aufruf
// achse ist PID_ROLL, PID_PITCH oder PID_YAW
float berechne_PID(float E_t_, float dt, int achse)
{
  // Definition PID Variabeln
  float P_t_ = 0;
  float I_t_ = 0;
  float D_t_ = 0;
  float U_t_ = 0;

  // Proportionalanteil
  P_t_ = Kp * E_t_;

  // Integralanteil
  // Multiplikaton mit konstantem Faktor dt wegen konstanter Looptime weggelassen
  I_t_ = I_t_1[achse] + Ki * E_t_; // * dt;
  I_t_1[achse] = I_t_;

  // unklar ob Integralanteil begrenzt werden soll
  // begrene Integralanteil
  if (I_t_1[achse] > 10) {
    I_t_1[achse] = 10;
    I_t_ = 10;
  }
  if (I_t_1[achse] < -10) {
    I_t_1[achse] = -10;
    I_t_ = -10;
  }

  // Differenzialanteil
  // Division durch konstanten Faktor dt wegen konstanter Looptime weggelassen
  // Eifache berechnung des Differenzialanteils mittles Subtraktion E(t) - E(t-1).
  // Aufgrund von allfälligen Sprungwerte wurde eine weniger fehleranfällige Methode verwendet.
  // D_t_ = Kd * (E_t_ - E_t_1[achse]); // / dt;
  // E_t_1[achse] = E_t_;

  // Allternativberechnung vom Differenzialanteil
  // Division durch konstanten Faktor dt wegen konstanter Looptime weggelassen.
  // Mit der Durchschnittsberechnung fallen Abweichungen nicht stark ins Gewicht.
  // Differenzial Berechung gemäss Buch 'Real-time operating system for ARM Corex-M Microcontrollers,
  // J.W Valvano, Seite 381'.
  D_t_ = Kd * (E_t_ + 3 * E_t_1[achse][ERR_1] - 3 * E_t_1[achse][ERR_2] - E_t_1[achse][ERR_3]) / 6;
  E_t_1[achse][ERR_3] = E_t_1[achse][ERR_2];
  E_t_1[achse][ERR_2] = E_t_1[achse][ERR_1];
  E_t_1[achse][ERR_1] = E_t_;

  // PID-Summe
  U_t_ = P_t_ + I_t_ + D_t_;

  // Rückgabewert
  return U_t_;
}
