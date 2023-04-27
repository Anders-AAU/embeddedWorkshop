// IF potentiometer_h not defined, load in the "library"
#ifndef motor_h
#define motor_h

#include <Arduino.h>

class Motor {
  public:
    Motor(int inputPin, int inputPinDirection, int outputA, int InputCountsPerRotation);
    int getRotationSpeed();
    void calcRotationSpeed();
    void readQuadrature();
    void setSpeed(int speed);
    void setDirection(int DIRECTION);
  private:
    int outA;
    int inA;
    int cpr;
    int position;
    int lastPosition;
    int lastTime;
    int rotationSpeed; // potentiometer value

    int direction;
    int directionPin;
};


#endif