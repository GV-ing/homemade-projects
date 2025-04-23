#include <WiFi.h>
#include <WebServer.h>
#include "esp_camera.h"

const char* ssid = "Vestri_Router";
const char* password = "Parigi1994!!";
WebServer server(80);

#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

void setup() {
  Serial.begin(115200);
  
  // Configurazione fotocamera a basso consumo
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  // ... (completa con i pin corretti)
  config.frame_size = FRAMESIZE_QVGA;  // 320x240
  config.jpeg_quality = 15;
  config.fb_count = 1;

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Errore fotocamera! Verifica alimentazione/pin.");
    while(1);
  }

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);
  Serial.println("IP: " + WiFi.localIP().toString());

  server.on("/", HTTP_GET, handleRoot);
  server.on("/stream", HTTP_GET, handleStream);
  server.begin();
}

void loop() {
  server.handleClient();
}

// ... (funzioni handleRoot e handleStream come prima)
// Pagina HTML con stream incorporato
void handleRoot() {
  String html = R"rawliteral(
    <html>
      <body>
        <h1>ESP32-CAM Live Stream</h1>
        <img src="/stream" />
      </body>
    </html>
  )rawliteral";
  server.send(200, "text/html", html);
}

// Gestione dello streaming MJPEG
void handleStream() {
  Serial.println("Client connesso allo stream");

  // Imposta gli header per lo streaming MJPEG
  server.sendHeader("Content-Type", "multipart/x-mixed-replace; boundary=frame");
  server.sendHeader("Cache-Control", "no-cache");
  server.sendHeader("Connection", "close");

  while (true) {
    camera_fb_t *fb = esp_camera_fb_get(); // Acquisisci un frame
    if (!fb) {
      Serial.println("Frame non acquisito!");
      break;
    }

    // Invia il frame come parte dello stream
    server.sendContent("--frame\r\n");
    server.sendContent("Content-Type: image/jpeg\r\n\r\n");
    server.sendContent((const char*)fb->buf, fb->len);
    server.sendContent("\r\n");

    esp_camera_fb_return(fb); // Rilascia il frame

    // Interrompi se il client si disconnette
    if (server.client().connected() == false) break;
  }
  Serial.println("Client disconnesso");
}