// motor.cpp


Motor::Motor(int inputPinA, int InputCountsPerRotation) {
	pinMode(pin, INPUT);
	pinA = inputPinA;
	position = 0;
	lastPosition = 0;
	countsPerRotation = InputCountsPerRotation;
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

	float rotationAmount = moved / countsPerRotation;

	rotationSpeed = (rotationAmount / (now-lastTime) ) * 1000 // precision is lost when made into integer

	lastTime = now
}

void Motor::readQuardature() {
	// adapted from https://github.com/curiores/ArduinoTutorials/blob/main/encoderControl/part3/part3.ino
	int state = digitalRead(inputPinA)
	switch (state) {
		case HIGH:
			position++;
			break
		case LOW:
			position--;
			break;
	}

}

void Motor::setSpeed() {
	// Not implemented yet
}


