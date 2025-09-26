#include <WiFi.h>
#include <WiFi.h>
#include <esp_system.h>  // Include la libreria per il reset software

#define LED_PIN 4  // Pin del LED (usualmente GPIO 4 sulla ESP32-CAM)

// Configurazione Wi-Fi
const char* ssid = "Vestri_Router"; // Inserisci il nome della tua rete Wi-Fi
const char* password = "Parigi1994!!"; // Inserisci la password della rete

WiFiServer server(5678); // Porta del server TCP

void setup() {
  // Configura la comunicazione seriale
  delay(1000);
  Serial.begin(115200); // Debug seriale

  pinMode(LED_PIN, OUTPUT);  // Configura il pin del LED
  digitalWrite(LED_PIN, LOW); // Assicura che il LED sia spento all'avvio

  // Controlla lo stato del reset
  if (esp_reset_reason() == ESP_RST_POWERON) {
    Serial.println("Avvio normale.");
  } else {
    Serial.printf("Motivo del reset: %d\n", esp_reset_reason());
  }

  // Connessione al Wi-Fi
  WiFi.begin(ssid, password);
  unsigned long startAttemptTime = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) { // Timeout 10s
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Connessione Wi-Fi fallita, riavvio...");
    delay(2000);
    esp_restart();  // Esegui un reset software
  }

  digitalWrite(LED_PIN, HIGH); // Indica che la connessione è avvenuta
  delay(500);
  digitalWrite(LED_PIN, LOW);

  // Avvia il server TCP
  server.begin();
}

void loop() {
  // Controlla se ci sono client connessi
  WiFiClient client = server.available();
  if (client) {
    while (client.connected()) {
      // Leggi i dati dal client
      if (client.available()) {
        String comando = client.readStringUntil('\n');
        comando.trim(); // Rimuove eventuali spazi o newline
        if (comando=="L_on"){
          digitalWrite(LED_PIN, HIGH); // Indica che la connessione è avvenuta
          delay(5);
        }else if (comando=="L_off"){
          digitalWrite(LED_PIN, LOW); // Indica che la connessione è avvenuta
          delay(5);
        }else{
          Serial.println(comando);
          delay(5);
        }
      }
    }
    client.stop();
  }
}
