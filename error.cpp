////////////////////
// error.cpp - Berechnung der Abweichung
// Fehler berechnen//
////////////////////

// Fehlerberechung
float berechne_E_t_(float ist_wert, float soll_wert)
{
  // Fehler E(t) zum Zeitpunkt t
  float E_t_;

  // Fehlerberechung
  E_t_ = soll_wert - ist_wert;

  return E_t_;
}
