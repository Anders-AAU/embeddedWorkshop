// potentiometer.cpp
#include "potentiometer.h"

Potentiometer::Potentiometer(int inputPin, int bitResolution) {
  pinMode(pin, INPUT);
  pin = inputPin;
  bitRes = bitResolution;
}

int Potentiometer::getValue() {
  return measurement;
}

int Potentiometer::getDutycycle(){
  return dutycycle;
}

void Potentiometer::makeMeasurement(){
  measurement = analogRead(pin);
  dutycycle = measurement / pow(2, bitRes); 
  
}


