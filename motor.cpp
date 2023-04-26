#include "Arduino.h"
// motor.cpp

#include "motor.h"

// inputPinA is called quadPinB in .ino file
Motor::Motor(int inputPinA, int outputA, int InputCountsPerRotation) {
	inA = inputPinA;
  outA = outputA;
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

	rotationSpeed = (rotationAmount / (now-lastTime) ) * 1000; // precision is lost when made into integer

	lastTime = now;
}

void Motor::readQuadrature() {
	// adapted from https://github.com/curiores/ArduinoTutorials/blob/main/encoderControl/part3/part3.ino
	int state = digitalRead(inA);
  // I'm pretty sure that this line needs to say "lastPosition = position;"
  // This line should say "if inA > 0" as a criteria for the switch down below
	switch (state) {
		case HIGH: // This needs to be "if inB == 0"
			position++;
			break;
		case LOW: // This needs to be "if inB > 0"
			position--;
			break;
	}

}

void Motor::setSpeed(int speed) {
	analogWrite(outA, speed);
}


