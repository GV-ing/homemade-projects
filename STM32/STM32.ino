#include <STM32FreeRTOS.h>

// Configurazione filtro
#define FILTER_WINDOW 5
int filterBuffer[FILTER_WINDOW] = {0};
int filterIndex = 0;

// Coda per i dati filtrati
QueueHandle_t serialQueue;

void micTask(void *pvParameters) {
  while(1) {
    // 1. Lettura analogica
    int rawValue = analogRead(A0);
    
    // 2. Applica filtro media mobile
    filterBuffer[filterIndex] = rawValue;
    filterIndex = (filterIndex + 1) % FILTER_WINDOW;
    
    int filteredValue = 0;
    for(int i=0; i<FILTER_WINDOW; i++) {
      filteredValue += filterBuffer[i];
    }
    filteredValue /= FILTER_WINDOW;

    // 3. Invia valore filtrato alla coda
    xQueueSend(serialQueue, &filteredValue, portMAX_DELAY);
    
    vTaskDelay(pdMS_TO_TICKS(10)); // Campionamento a 100Hz
  }
}

void serialTask(void *pvParameters) {
  while(1) {
    int receivedValue;
    if(xQueueReceive(serialQueue, &receivedValue, portMAX_DELAY) == pdTRUE) {
      // 4. Stampa su seriale in formato plottabile
      Serial.println(receivedValue);
    }
  }
}

void setup() {
  pinMode(PC13, OUTPUT); 
  Serial.begin(115200);
  
  
  analogReadResolution(12); // ADC a 12-bit per STM32
  
  // Crea coda per 10 valori
  serialQueue = xQueueCreate(10, sizeof(int));
  
  // Crea task FreeRTOS
  xTaskCreate(micTask, "Mic", 128, NULL, 2, NULL);
  xTaskCreate(serialTask, "Serial", 128, NULL, 1, NULL);
  
  // Avvia scheduler
  vTaskStartScheduler();
}

void loop() {
  digitalToggle(PC13);
  delay(500);
}