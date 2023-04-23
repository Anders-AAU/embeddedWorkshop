/*
   Example of a FreeRTOS mutex used as a starting point
   https://www.freertos.org/Real-time-embedded-RTOS-mutexes.html
*/

// Include Arduino FreeRTOS library
#include <Arduino_FreeRTOS.h>


// Include semaphore/mutex support
#include <semphr.h>



// Include homemade potentimeter class. Interfaced with getValue() and makeMeasurement()
#include "potentiometer.h"
Potentiometer pot(A0);


// Declaring a global variable of type SemaphoreHandle_t
SemaphoreHandle_t potMutex;

// Declaring a global variable of type SemaphoreHandle_t
SemaphoreHandle_t interruptSemaphore;
#define interruptPin 2

//////////////// Message system /////////////////
// Include queue support
#include <queue.h>
// Define a struct for messageing
struct messageStruct {
  String sender;
  int value;
};

QueueHandle_t structQueue;
/////////////////////////////////////////////////



// Declaring tasks
void TaskMakeMeasurement( void *pvParameters );
void TaskDoSomething( void *pvParameters );
void TaskSerial( void *pvParameters);



void setup() {
  Serial.begin(9600);
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

  /**
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
              1, // Task priority
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
      pot.makeMeasurement()
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

    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

    vTaskDelay(pdMS_TO_TICKS(delayTime));
  }
}


void TaskDoSomething(void *pvParameters)
{
  (void) pvParameters;

  int delayTime = 500;

  for (;;)
  {
   if (xSemaphoreTake(potMutex, 3) == pdTRUE)
    {
      Serial.print("Potentiometer reads: ");
      Serial.println(pot.getValue());
      xSemaphoreGive(potMutex);
    }
    else
    {
      Serial.println("Blocked");
    }

    if (xSemaphoreTake(interruptSemaphore, 0) == pdPASS) {
      Serial.println("Interrupted");
    }

    vTaskDelay(pdMS_TO_TICKS(delayTime));
  
  }
}
void TaskSerial( void *pvParameters)
{
  (void) pvParameters;
  
  while(!Serial){
    vTaskDelay(1);
  }

  for (;;)
  {
    struct messageStruct measurement;
    if (xQueueReceive(structQueue, &measurement, 1) == pdPASS) {
      Serial.print("Sender: ");
      Serial.print(measurement.sender);
      Serial.print(" Value: ");
      Serial.println(measurement.value);
    }
  }
  
}
