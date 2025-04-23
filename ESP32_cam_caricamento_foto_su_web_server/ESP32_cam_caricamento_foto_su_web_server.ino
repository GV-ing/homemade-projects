#include <WiFi.h>
#include <WebServer.h>
#include <SD_MMC.h>

const char* ssid = "Vestri_Router";
const char* password = "Parigi1994!!";
WebServer server(80);

void setup() {
  Serial.begin(115200);
  
  // Connessione WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nIP: " + WiFi.localIP().toString());

  // Inizializzazione SD
  if (!SD_MMC.begin()) {
    Serial.println("Errore SD!");
    return;
  }

  // Route del server
  server.on("/", HTTP_GET, handleRoot);
  server.on("/image", HTTP_GET, handleImage);
  server.begin();
}

void loop() {
  server.handleClient();
}

void handleRoot() {
  String html = R"rawliteral(
    <html>
      <body>
        <h1>ESP32-CAM Photo</h1>
        <img src="/image" width="800" />
      </body>
    </html>
  )rawliteral";
  server.send(200, "text/html", html);
}

void handleImage() {
  File file = SD_MMC.open("/latest.jpg");
  if (!file) {
    server.send(404, "text/plain", "Foto non trovata!");
    return;
  }

  server.sendHeader("Content-Type", "image/jpeg");
  server.setContentLength(file.size());  // Evita header duplicati
  server.send(200, "", "");

  uint8_t buffer[1024];
  size_t bytesRead;
  while ((bytesRead = file.read(buffer, sizeof(buffer))) > 0) {
    server.sendContent((const char*)buffer, bytesRead);
  }
  file.close();
}