// potentiometer.cpp
#include "potentiometer.h"

Potentiometer::Potentiometer(int inputPin) {
  pinMode(pin, INPUT);
  pin = inputPin;
}

int Potentiometer::getValue() {
  return measurement;
}

void Potentiometer::makeMeasurement(){
  measurement = analogRead(pin);
}


