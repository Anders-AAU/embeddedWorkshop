
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

  PIDPotQueue      = xQueueCreate(10, // Queue length
                              sizeof(int) // Queue item size
                            );
                            
  PIDMotorQueue    = xQueueCreate(10, // Queue length
                              sizeof(int) // Queue item size
                            );

  motorQueue       = xQueueCreate(10, // Queue length
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
              1, // Task priority
              NULL);

  xTaskCreate(TaskPotentiometerInfo,
              "Task Potentiometer info",
              128,
              NULL, 
              2, // Task priority
              NULL);

  xTaskCreate(TaskMotorControl,
              "Task motor control",
              128,
              NULL, 
              1, // Task priority
              NULL);

  xTaskCreate(TaskMotorControlInfo,
              "Task motor info",
              128,
              NULL, 
              2, // Task priority
              NULL);

  xTaskCreate(TaskSerial,
              "Task Serial print", 
              128, 
              NULL,
              2, // Task priority
              NULL);

  xTaskCreate(TaskPID,
              "Task PID calc", 
              128, 
              NULL,
              1, // Task priority
              NULL);

  /*
  xTaskCreate(TaskKeyboardControl,
              "Task Keyboard Control",
              128,
              NULL,
              1, // Task priority
              NULL);
  */



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

  int delayTime = 50;
  int speed = 0;

  for (;;)
  {
   if (xSemaphoreTake(potMutex, 3) == pdTRUE)
    {
      pot.makeMeasurement();
    
      speed = pot.getValue();
      

      xQueueSend(PIDPotQueue, &speed, portMAX_DELAY);

      xSemaphoreGive(potMutex);
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
   if (xSemaphoreTake(potMutex, 3) == pdTRUE)
    {
      printInfo.value = pot.getValue();
      
      xQueueSend(structQueue, &printInfo, portMAX_DELAY);
      xSemaphoreGive(potMutex);
      
    }
    
    vTaskDelay(pdMS_TO_TICKS(infoDelay));
  }
}

void TaskMotorControl(void *pvParameters)
{
  (void) pvParameters;
  int delayTime = 50;
  int valueFromQueue;
  int speed;
  for (;;)
  { 
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
  if (xSemaphoreTake(motorMutex, 3) == pdTRUE)
    {
      printInfo.value = motor.getRotationSpeed();
      xQueueSend(structQueue, &printInfo, portMAX_DELAY);
      xSemaphoreGive(motorMutex);
    }


    vTaskDelay(pdMS_TO_TICKS(infoDelay));
  }
}



void TaskKeyboardControl( void *pvParameters)
{
  (void) pvParameters;
  int delayTime = 500;

  struct messageStruct messageCommand;
  messageCommand.sender = "Keyboard Control";
  messageCommand.msgType = task;
  for (;;)
  {
    
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

    vTaskDelay(pdMS_TO_TICKS(delayTime));
  }

}


void TaskPID(void *pvParameters)
{
  (void) pvParameters;

  int delayTime = 50;
  struct messageStruct debuging;
  debuging.sender = "PID";
  debuging.msgType = debug;

  int correctedSpeed;
  int tempSetpoint; // 
  int tempInput = 69;

  for (;;)
  { 
    
    xQueueReceive(PIDPotQueue, &tempSetpoint, portMAX_DELAY); // speed we aim for
    xQueueReceive(PIDMotorQueue, &tempInput, portMAX_DELAY); // actual speed
    Setpoint = tempSetpoint;
    PIDInput = tempInput;
    
    if ( myPID.Compute() ) // returns bool if a new value has been calculated
    {
      //debuging.value = PIDOutput; xQueueSend(structQueue, &debuging, portMAX_DELAY); //debug line
      xQueueSend(motorQueue, &PIDOutput, portMAX_DELAY);
    }

    vTaskDelay(pdMS_TO_TICKS(delayTime));
  }
}

void TaskSerial( void *pvParameters)
{
  (void) pvParameters;
  struct messageStruct message;

  
  
  while(!Serial){
    vTaskDelay(1);
  }

  for (;;)
  {
    
    if (xQueueReceive(structQueue, &message, 1) == pdPASS) {
      Serial.print("\n \n \n \n \n \n \n \n ");
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
  }
}
