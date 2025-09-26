#ifndef WIFISETUP_H
#define WIFISETUP_H

#include <WiFi.h>
#include "Config.h"

void connectToWiFi() {
  Serial.println("Connessione al Wi-Fi...");
  WiFi.begin(ssid, password);
  unsigned long startAttemptTime = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
    Serial.print(".");
    delay(500);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnesso al Wi-Fi!");
    Serial.print("Indirizzo IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nErrore: impossibile connettersi al Wi-Fi.");
    esp_restart(); // Riavvia se non riesce a connettersi
  }
}

#endif
