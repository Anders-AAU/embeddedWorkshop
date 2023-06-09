
// To have something to start from, we used the mutex example code from the FreeRTOS library
/* 
 * Example of a FreeRTOS mutex used as a starting point
 * https://www.freertos.org/Real-time-embedded-RTOS-mutexes.html
 */


// Include Arduino FreeRTOS library
#include <Arduino_FreeRTOS.h>
// Include semaphore/mutex support
#include <semphr.h>
// Include queue support
#include <queue.h>
// Include PID library https://github.com/br3ttb/Arduino-PID-Library/
#include "PID_v1.h"

#define potentiometerPin A0

#define quadPinA 2
#define quadPinB 9
#define motorOutA 10
#define motorDirection 12
#define countsPerRotation 1024

// ms between info from task is to be printed to serial
#define shortDelay 200
#define mediumDelay 500
#define infoDelay 2000
#define portMAX_DELAY 3 // Redefine portMax_DELAY

// Include homemade class's. 
#include "potentiometer.h"
Potentiometer pot(potentiometerPin, 10); // Interfaced with getValue() and makeMeasurement()
#include "motor.h"
Motor motor(quadPinB, motorDirection, motorOutA, countsPerRotation); //


// Declaring a global variable of type SemaphoreHandle_t
SemaphoreHandle_t potMutex;
SemaphoreHandle_t motorMutex;

SemaphoreHandle_t motorSemaphore; // Motor interrupt


/*
  PID library example code is followed for the PID implementation
*/

#define PID_INPUT 0
#define PID_OUTPUT 3

//Define Variables we'll be connecting to
double Setpoint, PIDInput, PIDOutput;

//Specify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1;
PID myPID(&PIDInput, &PIDOutput, &Setpoint, Kp, Ki, Kd, DIRECT);

// Define some types of messages
enum msgType_t {
  task = 1,
  debug = 2,
  warning = 3
};

// Define a struct for messageing
struct messageStruct {
  String sender;
  int value;
  String optional;
  msgType_t msgType;
};

QueueHandle_t structQueue;
QueueHandle_t PIDPotQueue;
QueueHandle_t PIDMotorQueue;
QueueHandle_t motorQueue;

int POTMAX = 0;
int POTIMAX = 0;
int MOTMAX = 0;
int MOTIMAX = 0;
int PIDMAX = 0;
int SERMAX = 0;
int KEYMAX = 0;


void setup() {
  Serial.begin(9600);
  
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(quadPinA, INPUT);

  //initialize the variables we're linked to
  PIDInput = 0;  
  Setpoint = 100;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);


  structQueue      = xQueueCreate(10, // Queue length
                              sizeof(struct messageStruct) // Queue item size
                            );

  PIDPotQueue      = xQueueCreate(2, // Queue length
                              sizeof(int) // Queue item size
                            );
                            
  PIDMotorQueue    = xQueueCreate(2, // Queue length
                              sizeof(int) // Queue item size
                            );

  motorQueue       = xQueueCreate(2, // Queue length
                              sizeof(int) // Queue item size
                            );



  /*
    Create a mutex.
    https://www.freertos.org/CreateMutex.html
  */
 
  potMutex = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the potentiometer interaction
  if ( ( potMutex ) != NULL )
    xSemaphoreGive( ( potMutex ) );  // Make the mutex available for use, by "Giving" the Semaphore.

  motorMutex = xSemaphoreCreateMutex();  
  if ( ( motorMutex ) != NULL )
    xSemaphoreGive( ( motorMutex ) ); 



  /**
   * Create a binary semaphore.
   * https://www.freertos.org/xSemaphoreCreateBinary.html
   */
  motorSemaphore = xSemaphoreCreateBinary();
  if (motorSemaphore != NULL) {
    // Attach interrupt for Arduino digital pin. 
    attachInterrupt(digitalPinToInterrupt(quadPinA), interruptHandler, RISING);
  }

  /*
     Create tasks
  */

  xTaskCreate(TaskPotentiometer,
              "Task Potentiometer Measurement",
              128,
              NULL, 
              3, // Task priority
              NULL);
#define debugTaskState1 31
pinMode(debugTaskState1, OUTPUT);

  xTaskCreate(TaskPotentiometerInfo,
              "Task Potentiometer info",
              128,
              NULL, 
              1, // Task priority
              NULL);
  #define debugTaskState2 33
  pinMode(debugTaskState2, OUTPUT);

  xTaskCreate(TaskMotorControl,
              "Task motor control",
              128,
              NULL, 
              3, // Task priority
              NULL);
  #define debugTaskState3 35
  pinMode(debugTaskState3, OUTPUT);

  xTaskCreate(TaskMotorControlInfo,
              "Task motor info",
              128,
              NULL, 
              1, // Task priority
              NULL);
  #define debugTaskState4 37
  pinMode(debugTaskState4, OUTPUT);

  xTaskCreate(TaskSerial,
              "Task Serial print", 
              128, 
              NULL,
              1, // Task priority
              NULL);
  #define debugTaskState5 39
  pinMode(debugTaskState5, OUTPUT);

  xTaskCreate(TaskPID,
              "Task PID calc", 
              128, 
              NULL,
              3, // Task priority
              NULL);
  #define debugTaskState6 41
  pinMode(debugTaskState6, OUTPUT);

  
  xTaskCreate(TaskKeyboardControl,
              "Task Keyboard Control",
              128,
              NULL,
              2, // Task priority
              NULL);
  
  #define debugTaskState7 43
  pinMode(debugTaskState7, OUTPUT);


  motor.setDirection(HIGH);
}

void loop() {}

void interruptHandler() {
  // Give semaphore in the interrupt handler. Code block can now run in task
  xSemaphoreGiveFromISR(motorSemaphore, NULL);
  digitalWrite(LED_BUILTIN,!digitalRead(LED_BUILTIN));

  /*   USE THIS TO ACCESS THE SEMAPHORE IN THE TASK
  
    if (xSemaphoreTake(motorSemaphore, portMAX_DELAY) == pdPASS) {
        code that does stuff
      }

  */
}


void TaskPotentiometer(void *pvParameters)
{
  (void) pvParameters;

  int delayTime = shortDelay;
  int speed = 0;

  for (;;)
  {
    int A = millis();
    digitalWrite(debugTaskState1, HIGH);
    
   if (xSemaphoreTake(potMutex, 3) == pdTRUE)
    {
      pot.makeMeasurement();
    
      speed = pot.getValue();
      

      xQueueSend(PIDPotQueue, &speed, portMAX_DELAY);

      xSemaphoreGive(potMutex);
    }
    digitalWrite(debugTaskState1, LOW);
    int endTime = millis()-A;
    if (endTime > POTMAX){
      POTMAX = endTime;
      Serial.print("Potentiometer took: ");Serial.println(POTMAX);

    }
    vTaskDelay(pdMS_TO_TICKS(delayTime));
  
  }
}

void TaskPotentiometerInfo(void *pvParameters)
{
  (void) pvParameters;

  struct messageStruct printInfo;
  printInfo.sender = "Potentiometer";
  printInfo.msgType = task;

  for (;;)
  {
    int A = millis();
    digitalWrite(debugTaskState2, HIGH);
   if (xSemaphoreTake(potMutex, 3) == pdTRUE)
    {
      printInfo.value = pot.getValue();
      
      xQueueSend(structQueue, &printInfo, portMAX_DELAY);
      xSemaphoreGive(potMutex);
      
    }
    
    digitalWrite(debugTaskState2, LOW);
    int endTime = millis() - A;
    if (endTime > POTIMAX) {
      POTIMAX = endTime;
      Serial.print("PotINFO took: ");Serial.println(POTIMAX);
    }
    vTaskDelay(pdMS_TO_TICKS(infoDelay));
  }
}

void TaskMotorControl(void *pvParameters)
{
  (void) pvParameters;
  int delayTime = shortDelay;
  int valueFromQueue;
  int speed;
  for (;;)
  { 
    int A = millis();
    digitalWrite(debugTaskState3, HIGH);
    if (xSemaphoreTake(motorSemaphore, portMAX_DELAY) == pdPASS) // if interupt happened
    {
      if (xSemaphoreTake(motorMutex, portMAX_DELAY) == pdPASS) 
      {
        motor.readQuadrature(); 
        motor.calcRotationSpeed();

        speed = motor.getRotationSpeed();
        xQueueSend(PIDMotorQueue, &speed, portMAX_DELAY);
        xSemaphoreGive(motorMutex);
      }
      xSemaphoreGive(motorSemaphore);
    }    
    if (xQueueReceive(motorQueue, &valueFromQueue, portMAX_DELAY) == pdPASS)
    {
      //Serial.print("Set motor to: ");Serial.println(valueFromQueue);
    }
    if (xSemaphoreTake(motorMutex, portMAX_DELAY) == pdPASS) 
    {
      motor.setSpeed(valueFromQueue);
      xSemaphoreGive(motorMutex);
    }

    digitalWrite(debugTaskState3, LOW);

    int endTime = millis()-A;
    if (endTime > MOTMAX) {
      MOTMAX = endTime;
      Serial.print("MotorControl took: ");Serial.println(MOTMAX);
    }
    vTaskDelay(pdMS_TO_TICKS(delayTime));
  }
}

void TaskMotorControlInfo(void *pvParameters)
{
  (void) pvParameters;
  struct messageStruct printInfo;
  printInfo.sender = "Motor";
  printInfo.msgType = task;
  for (;;)
  {
    int A = millis();
    digitalWrite(debugTaskState4, HIGH);
    if (xSemaphoreTake(motorMutex, portMAX_DELAY) == pdTRUE)
      {
        printInfo.value = motor.getRotationSpeed();
        xQueueSend(structQueue, &printInfo, portMAX_DELAY);
        xSemaphoreGive(motorMutex);
      }

    digitalWrite(debugTaskState4, LOW);

    int endTime = millis()-A;
    if (endTime > MOTIMAX) {
      MOTIMAX = endTime;
      Serial.print("MotorControlInfo took: ");Serial.println(MOTIMAX);
    }
    vTaskDelay(pdMS_TO_TICKS(infoDelay));
  }
}



void TaskKeyboardControl( void *pvParameters)
{
  (void) pvParameters;
  int delayTime = mediumDelay;

  struct messageStruct messageCommand;
  messageCommand.sender = "Keyboard Control";
  messageCommand.msgType = task;
  for (;;)
  {
    int A = millis();
    digitalWrite(debugTaskState7, HIGH);
    if (Serial.available() > 0)
    { 
      String inputCommand;
      while (Serial.available() > 0) {
        inputCommand += (char)Serial.read();
      }
      inputCommand.toLowerCase();

      
      if(inputCommand == "f")
      {
        // forward
        messageCommand.optional = "Forward";
        xQueueSend(structQueue, &messageCommand, portMAX_DELAY);
      }
      else if(inputCommand == "r")
      {
        // Reverse
        messageCommand.optional = "Reverse";
        xQueueSend(structQueue, &messageCommand, portMAX_DELAY);
      }
      
    }

    digitalWrite(debugTaskState7, LOW);
    int endTime = millis()-A;
    if (endTime > KEYMAX) {
      KEYMAX = endTime;
      Serial.print("Keyboard took: ");Serial.println(KEYMAX);
    }
    vTaskDelay(pdMS_TO_TICKS(delayTime));
  }

}


void TaskPID(void *pvParameters)
{
  (void) pvParameters;

  int delayTime = shortDelay;
  struct messageStruct debuging;
  debuging.sender = "PID";
  debuging.msgType = debug;

  int correctedSpeed;
  int tempSetpoint; // 
  int tempInput = 69;

  for (;;)
  { 
    int A = millis();    
    digitalWrite(debugTaskState6, HIGH);
    xQueueReceive(PIDPotQueue, &tempSetpoint, portMAX_DELAY); // speed we aim for
    xQueueReceive(PIDMotorQueue, &tempInput, portMAX_DELAY); // actual speed
    Setpoint = tempSetpoint;
    PIDInput = tempInput;
    
    if ( myPID.Compute() ) // returns boolean with value true if a endTime value has been calculated
    {
      //debuging.value = PIDOutput; xQueueSend(structQueue, &debuging, portMAX_DELAY); //debug line
      xQueueSend(motorQueue, &PIDOutput, portMAX_DELAY);
    }

    digitalWrite(debugTaskState6, LOW);
    int endTime = millis()-A;
    if (endTime > PIDMAX) {
      PIDMAX = endTime;
      Serial.print("PID took: ");Serial.println(PIDMAX);
    }
    vTaskDelay(pdMS_TO_TICKS(delayTime));
  }
}

void TaskSerial( void *pvParameters)
{
  (void) pvParameters;
  int delayTime = mediumDelay;
  struct messageStruct message;
  
  while(!Serial){ // Wait for Serial to be ready
    vTaskDelay(1);
  }
  vTaskDelay(100);

  for (;;)
  {
    int A = millis();
    digitalWrite(debugTaskState5, HIGH);
    if (xQueueReceive(structQueue, &message, 1) == pdPASS) {
      Serial.print("\n \n ");
      switch(message.msgType){        
        case 1:
          Serial.print("Task"); break;
        case 2:
          Serial.print("Debug"); break;
        case 3:
          Serial.print("\n - - - WARNING - - - \n");break;
      }

      Serial.print(" ::: Sender: ");
      Serial.println(message.sender);
      Serial.print(" \t --> ");
      
      if (message.optional != ""){
        // If not an empty string
        Serial.println(message.optional);
      }
      else {
        Serial.print("Value: ");

        /// Make output pretty
        if (message.value < 1000) {
          Serial.print(" ");
        }
        if (message.value < 100) {
          Serial.print(" ");
        }
        if (message.value < 10) {
          Serial.print(" ");
        }

        Serial.print(message.value);

        if (message.sender == "Motor") {
          Serial.print(" RPM \n");
        }
        else {
          Serial.print(" \n");
        }
        
      }

    }
    digitalWrite(debugTaskState5, LOW);
    int endTime = millis()-A;
    if (endTime > SERMAX) {
      SERMAX = endTime;
      Serial.print("SERIAL took: ");Serial.println(SERMAX);
    }
    vTaskDelay(pdMS_TO_TICKS(delayTime));
  }
  
}
