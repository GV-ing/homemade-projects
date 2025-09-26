#ifndef SERVERHANDLER_H
#define SERVERHANDLER_H

#include <WiFi.h>
#include <esp_camera.h>
#include "Config.h"

WiFiServer commandServer(COMMAND_PORT);
WiFiServer videoServer(VIDEO_PORT);
SemaphoreHandle_t cameraMutex = xSemaphoreCreateMutex();

void startServers() {
  commandServer.begin();
  videoServer.begin();
}

/*void handleCommands(void *pvParameters) {
  while (true) {
    WiFiClient commandClient = commandServer.available();
    if (commandClient) {
      Serial.println("Client connesso per i comandi.");
      String command = "";

      if (xSemaphoreTake(cameraMutex, pdMS_TO_TICKS(200))) {  // Timeout ridotto a 200ms
        if (commandClient.connected()) {
          if (commandClient.available()) {
            char c = commandClient.read();
            command += c;

            if (c == '\n') {
              command.trim();
              Serial.println(command);

              if (command == "L_on") {
                digitalWrite(LED_PIN, HIGH);
              } else if (command == "L_off") {
                digitalWrite(LED_PIN, LOW);
              }

              command = ""; // Resetta il comando
            }
          }
        }

        xSemaphoreGive(cameraMutex); // Rilascia il mutex
      } else {
        Serial.println("Timeout nel tentativo di acquisire il mutex.");
      }

      commandClient.stop();
      Serial.println("Client disconnesso dai comandi.");
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);  // Pausa breve per evitare un uso intensivo della CPU
  }
}*/

// Funzione per gestire lo streaming video
void handleVideoStream(void *pvParameters) {
  while (true) {
    if (xSemaphoreTake(cameraMutex, pdMS_TO_TICKS(200))) {  // Timeout ridotto a 200ms
      WiFiClient videoClient = videoServer.available();
      if (videoClient) {
        Serial.println("Client connesso per lo streaming video.");
        videoClient.println("HTTP/1.1 200 OK");
        videoClient.println("Content-Type: multipart/x-mixed-replace; boundary=frame");
        videoClient.println();

        unsigned long lastFrameTime = millis();
        while (videoClient.connected()) {
          if (millis() - lastFrameTime > 5000) { // Timeout di 5 secondi
            Serial.println("Timeout nello streaming video.");
            break;
          }

          camera_fb_t* fb = esp_camera_fb_get();
          if (!fb) {
            Serial.println("Errore nel frame buffer.");
            vTaskDelay(100 / portTICK_PERIOD_MS); // Attendi prima di ritentare
            continue;
          }

          videoClient.println("--frame");
          videoClient.println("Content-Type: image/jpeg");
          videoClient.printf("Content-Length: %d\r\n\r\n", fb->len);
          videoClient.write(fb->buf, fb->len);
          videoClient.println();

          esp_camera_fb_return(fb);
          lastFrameTime = millis(); // Reset del timeout
          vTaskDelay(50 / portTICK_PERIOD_MS); // Delay non bloccante
        }

        videoClient.stop();
        Serial.println("Client disconnesso dallo streaming video.");
      }
      xSemaphoreGive(cameraMutex);
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);  // Pausa breve per evitare un uso intensivo della CPU
  }
}



#endif
