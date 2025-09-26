#include <Wire.h>
#include <U8g2lib.h>

#define MQ7_PIN A2  // Pin del sensore MQ-7
#define BUZZER_PIN 6 // Pin del buzzer
//#define RL 10.0      // Resistenza di carico in kΩ
//#define VREF 5.0     // Tensione di riferimento di Arduino
#define MAX_L 1000
#define MIN_L 0
//float R0 = 10.0; // Valore sperimentale di R0 da calibrare!
int wait=0;
// Inizializzazione del display OLED SH1106 128x64
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// Funzione per convertire il valore ADC in PPM usando R0
float readCO() {
    int rawValue = analogRead(MQ7_PIN);  
    /*float voltage = (rawValue / 1023.0) * VREF;  
    float RS = ((VREF - voltage) / voltage) * RL;  
    float ratio = RS/R0; 
 // RS relativo a R0

    // Funzione approssimata per ottenere i PPM di CO (da calibrare meglio!)
    float ppm = pow(10, (log10(ratio) - 0.72) / -0.35); */
    float ppm = map (rawValue, 0, 1023, MAX_L, MIN_L);
    
    return ppm;
}

// Disegna la barra orizzontale in base al valore di CO
void drawCOBar(float ppm) {
    int barWidth = map(ppm,  MIN_L, MAX_L, 0, 120); // Mappa il valore su una larghezza massima di 120px

    u8g2.clearBuffer();  
    u8g2.setFont(u8g2_font_ncenB08_tr);  

    // Disegna il valore numerico di CO
    u8g2.setCursor(10, 20);
    u8g2.print("CO: ");
    u8g2.print(ppm, 1);
    u8g2.print(" PPM");

    // Disegna il contorno della barra
    u8g2.drawFrame(4, 40, 120, 10); 

    // Disegna la barra riempita
    u8g2.drawBox(4, 40, barWidth, 10); 

    u8g2.sendBuffer();
}

// Funzione per il buzzer (suona in base al livello di CO)
void alertBuzzer(float ppm) {
  wait =sqrt(10e5-(ppm*ppm));
  tone(BUZZER_PIN, 1000); // Suona a 1000Hz
  delay(100);
  noTone(BUZZER_PIN);
  delay(wait);
    
}

void setup() {
    u8g2.begin();
    pinMode(BUZZER_PIN, OUTPUT);
}

void loop() {
    float coValue = readCO(); // Legge il valore del sensore
    drawCOBar(coValue); // Mostra il valore e la barra sul display
    alertBuzzer(coValue); // Attiva il buzzer se necessario
    delay(1000);
}
