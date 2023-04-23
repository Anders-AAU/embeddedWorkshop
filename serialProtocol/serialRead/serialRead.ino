#include <Arduino_FreeRTOS.h>


// Include semaphore/mutex support
#include <semphr.h>
#include <stdlib.h>

// Buffer size
static const uint8_t buf_len = 20;

// Declaring a global variable of type SemaphoreHandle_t
SemaphoreHandle_t potMutex;

// Declaring a global variable of type SemaphoreHandle_t
SemaphoreHandle_t interruptSemaphore;

// Define pins
#define interruptPin 2
static const int ledPin = LED_BUILTIN;

// Define structs for PID parameters received from serial port
// k in front of p, i, and d is meant to indic
typedef struct pid_struct {
  byte kp; // Proportional variable
  byte ki; // Integral variable
  byte kd; // Derivative variable
} pid_struct;

// Define instance of pid_struct
pid_struct 

// ********************************************************************

// Declaring tasks
 readSerial(void)


void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}






















