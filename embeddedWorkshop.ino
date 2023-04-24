/*
   Example of a FreeRTOS mutex used as a starting point
   https://www.freertos.org/Real-time-embedded-RTOS-mutexes.html
*/
//*****************************************************************************
// Configuration

// Include Arduino FreeRTOS library
#include <Arduino_FreeRTOS.h>
// Include semaphore/mutex support for FreeRTOS
#include <semphr.h>
// Include queue support for FreeRTOS
#include <queue.h>


// Include homemade potentimeter class. Interfaced with getValue() and makeMeasurement()
#include "potentiometer.h"
Potentiometer pot(A0);


// Globals
SemaphoreHandle_t potMutex; // Potentiometer mutex handle
SemaphoreHandle_t interruptSemaphore; // ISR mutex handle
QueueHandle_t structQueue; // Queue handle

// Macros
#define interruptPin 2


//////////////// Message system /////////////////

// Define a struct for messaging
struct messageStruct {
  String sender;
  int value;
  String optional;
};

/////////////////////////////////////////////////

// IKKE NØDVENDIGT AT DEKLARERE TASKS TO GANGE
// Declaring tasks
/* Frederik: Jeg synes vi skal rykke hele tasken herop, da det måske kan give bedre struktur på koden.
void TaskMakeMeasurement(   void *pvParameters );
void TaskDoSomething    (   void *pvParameters );
void TaskSerial         (   void *pvParameters );
void TaskKeyboardControl(   void *pvParameters );
*/


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(interruptPin, INPUT);


  structQueue = xQueueCreate(10, // Queue length
                              sizeof(struct messageStruct) // Queue item size
                            );


  /*
    Create a mutex.
    https://www.freertos.org/CreateMutex.html
  */
 
  if ( potMutex == NULL )  // Check to confirm that the Serial Semaphore has not already been created.
  {
    potMutex = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Serial Port
    if ( ( potMutex ) != NULL )
      xSemaphoreGive( ( potMutex ) );  // Make the Serial Port available for use, by "Giving" the Semaphore.
  }

  /**
   * Create a binary semaphore.
   * https://www.freertos.org/xSemaphoreCreateBinary.html
   */
  interruptSemaphore = xSemaphoreCreateBinary();
  if (interruptSemaphore != NULL) {
    // Attach interrupt for Arduino digital pin. 
    attachInterrupt(digitalPinToInterrupt(interruptPin), interruptHandler, HIGH);
  }

  /*
     Create tasks
  */
  xTaskCreate(TaskMakeMeasurement, // Task function
              "MakePotMeasurement", // Task name for humans
              128, 
              NULL,
              1, // Task priority
              NULL);

  xTaskCreate(TaskDoSomething,
              "DoesStuff",
              128,
              NULL, 
              1, // Task priority
              NULL);

  xTaskCreate(TaskSerial,
              "Task Serial print", 
              128, 
              NULL,
              3, // Task priority
              NULL);

  
  xTaskCreate(TaskKeyboardControl,
              "Task Keyboard Control",
              128,
              NULL,
              2, // Task priority
              NULL);
  

}

void loop() {}

void interruptHandler() {
  // Give semaphore in the interrupt handler. Code block can now run in task
  xSemaphoreGiveFromISR(interruptSemaphore, NULL);
  /* 


  ////////////////////////////////////////////////////////////
  USE THIS TO ACCESS THE SEMAPHORE IN THE TASK
  if (xSemaphoreTake(interruptSemaphore, portMAX_DELAY) == pdPASS) {
      code that does stuff
    }
  ////////////////////////////////////////////////////////////

  */
  
}

void TaskMakeMeasurement(void *pvParameters)
{
  (void) pvParameters;

  int delayTime = 100;
  
  struct messageStruct measurement;
  
  for (;;)
  {
    /**
       Take mutex
       https://www.freertos.org/a00122.html
    */
    if (xSemaphoreTake(potMutex, 3) == pdTRUE)
    { 
      pot.makeMeasurement();
      xSemaphoreGive(potMutex);
      measurement.sender = "Measurement Task";
      measurement.value = pot.getValue();

      /**
     * Post an item on a queue.
     * https://www.freertos.org/a00117.html
     */
      xQueueSend(structQueue, &measurement, portMAX_DELAY);
    }
    else
    {
      Serial.println("Blocked");
    }

    vTaskDelay(pdMS_TO_TICKS(delayTime));
  }
}


void TaskDoSomething(void *pvParameters)
{
  (void) pvParameters;

  int delayTime = 500;

  struct messageStruct motorSpeed;
  int debug = 0;
  for (;;)
  {
   if (xSemaphoreTake(potMutex, 3) == pdTRUE)
    {
      motorSpeed.sender = "TaskDoSomething ";
      motorSpeed.value = debug;
      xQueueSend(structQueue, &motorSpeed, portMAX_DELAY);
      xSemaphoreGive(potMutex);
    }
    else
    {
      Serial.println("Blocked");
    }

    if (xSemaphoreTake(interruptSemaphore, 0) == pdPASS) {
      Serial.println("Interrupted");
    }
    debug++;
    vTaskDelay(pdMS_TO_TICKS(delayTime));
  
  }
}


void TaskKeyboardControl( void *pvParameters)
{
  (void) pvParameters;
  int delayTime = 200;

  struct messageStruct messageCommand;
  messageCommand.sender = "Keyboard Control";
  messageCommand.value = -1;
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



void TaskSerial( void *pvParameters)
{
  (void) pvParameters;

  struct messageStruct message;

  Serial.begin(9600);
  
  while(!Serial){
    vTaskDelay(1);
  }

  for (;;)
  {
    
    if (xQueueReceive(structQueue, &message, 1) == pdPASS) {
      Serial.print("Sender: ");
      Serial.print(message.sender);
      Serial.print(" ---> ");
      
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
        
        Serial.println(message.value);
      }
      
    }
  }
}
