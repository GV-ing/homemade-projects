#include "esp_camera.h"
#include "SD_MMC.h"

// Configurazione pin fotocamera per ESP32-CAM (AI-Thinker)
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

void setup() {
  Serial.begin(115200);
  
  // Inizializza la fotocamera
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  
  // Risoluzione iniziale
  config.frame_size = FRAMESIZE_SVGA; // 800x600
  config.jpeg_quality = 10; // Qualità JPEG (0-63, più basso = migliore)
  config.fb_count = 1;

  // Avvia la fotocamera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Errore inizializzazione fotocamera: 0x%x", err);
    return;
  }

  // Inizializza SD_MMC
  if (!SD_MMC.begin()) {
    Serial.println("Errore inizializzazione SD!");
    return;
  }
  Serial.println("SD inizializzata correttamente!");
}

void loop() {
  // Acquisisci foto
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Errore acquisizione immagine");
    return;
  }

  // Salva su SD
  String path = "/foto_" + String(millis()) + ".jpg";
  fs::FS &fs = SD_MMC;
  
  File file = fs.open(path.c_str(), FILE_WRITE);
  if (!file) {
    Serial.println("Errore apertura file!");
  } else {
    file.write(fb->buf, fb->len);
    Serial.printf("Foto salvata: %s (dimensione: %d bytes)\n", path.c_str(), fb->len);
    file.close();
  }

  // Rilascia il buffer della fotocamera
  esp_camera_fb_return(fb);

  // Aspetta 10 secondi prima della prossima foto
  delay(10000);
}