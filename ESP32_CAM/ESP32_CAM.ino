#include "WiFiSetup.h"
#include "CameraSetup.h"
#include "ServerHandler.h"
#include "Config.h"

TaskHandle_t Commmands;
TaskHandle_t Video;

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  connectToWiFi();      // Connessione al Wi-Fi
  startCamera();        // Configura la fotocamera
  startServers();       // Avvia i server TCP e HTTP

  // Creazione dei task su core separati
  //xTaskCreatePinnedToCore(Commmandscode, "Commmands", 2048, NULL, 2, &Commmands, 0);  // Core 0
  xTaskCreatePinnedToCore(Videocode, "Video", 2048, NULL, 1, &Video, 1);  // Core 1

  digitalWrite(LED_PIN, HIGH);
  delay(100);
  digitalWrite(LED_PIN, LOW);
}

/*void Commmandscode(void * pvParameters) {
  for (;;) {
    // Aggiungi un log per verificare quando il task è attivo
    Serial.println("Gestione comandi in esecuzione...");

    handleCommands(NULL);  // Passa NULL come parametro

    // Aggiungi un log per vedere se il semaforo è stato acquisito correttamente
    if (xSemaphoreTake(cameraMutex, pdMS_TO_TICKS(500))) {  // Timeout di 500ms
      Serial.println("Mutex acquisito correttamente");
    } else {
      Serial.println("Timeout nel tentativo di acquisire il mutex");
    }

    vTaskDelay(150 / portTICK_PERIOD_MS);  // Ritardo per non bloccare la CPU
  }
}*/

void Videocode(void * pvParameters) {
  for (;;) {
    // Aggiungi un log per verificare quando il task è attivo
    Serial.println("Streaming video in esecuzione...");

    handleVideoStream(NULL);  // Passa NULL come parametro

    vTaskDelay(150 / portTICK_PERIOD_MS);  // Ritardo per non bloccare la CPU
  }
}

void loop() {
  // Non è necessario fare nulla qui, i task vengono gestiti da FreeRTOS
}

