#include "Arduino.h"
// motor.cpp

#include "motor.h"

// inputPinA is called quadPinB in .ino file
Motor::Motor(int inputPinA, int inputPinDirection, int outputA, int InputCountsPerRotation) {
	inA = inputPinA;
  outA = outputA;
  directionPin = inputPinDirection;
	pinMode(inA, INPUT);
	position = 0;
	lastPosition = 0;
	cpr = InputCountsPerRotation;
}

int Motor::getRotationSpeed() {
  	return rotationSpeed;
}

void Motor::calcRotationSpeed(){
	/*
	 * Returns RPM. Rotation direction is read from it being positive or negative  
	 */
	int now = millis();
	int moved = position - lastPosition;

	float rotationAmount = moved / cpr;

	rotationSpeed = (rotationAmount / (now-lastTime) ) * 1000; // Note that precision is lost when converted to integer

	lastTime = now;
  lastPosition = position; 
}

void Motor::readQuadrature() {
	// adapted from https://github.com/curiores/ArduinoTutorials/blob/main/encoderControl/part3/part3.ino
	int state = digitalRead(inA);
	switch (state) {
		case HIGH:
			position++;
			break;
		case LOW:
			position--;
			break;  
	}

}

void Motor::setSpeed(int speed) {
	analogWrite(outA, speed);
}

void Motor::setDirection(int DIRECTION) {
  switch (DIRECTION) {
    case HIGH:
      digitalWrite(directionPin, HIGH);
    case LOW:
      digitalWrite(directionPin, LOW);
  }
}



