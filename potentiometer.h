// potentiometer.h
/*
  ----- HOW TO USE -----
  // Create Potentiometer object using an availble pin
  Potentiometer pot(potPin);

  ----------------
  MAIN CODE 

  pot.makeMeasurement(); to read from the pin
  pot.getValue(); returns latest measurement
*/


// IF potentiometer_h not defined, load in the "library"
#ifndef potentiometer_h
#define potentiometer_h

#include <Arduino.h>

class Potentiometer {
  public:
    Potentiometer(int inputPin);
    int getValue();
    void makeMeasurement();
  private:
    int pin;
    int measurement; // potentiometer value
};

#endif