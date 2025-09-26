#include <Wire.h>
#include <U8g2lib.h>

#define MQ7_PIN A2      // Sensore MQ-7 su A2
#define BUZZER_PIN 6    // Buzzer su D6
#define RL 10.0         // Resistenza di carico in kΩ
#define VREF 5.0        // Tensione di riferimento di Arduino
#define MAX_PPM 500     // Valore massimo per la barra
#define MIN_PPM 0       // Valore minimo per la barra

// Calibrazione del sensore (da misurare sperimentalmente)
float R0 = 5;  // Valore medio misurato in aria pulita (da aggiornare con test)

// Inizializzazione OLED SH1106 128x64
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// Funzione per mappare numeri float
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Funzione per leggere la concentrazione di CO in PPM
float readCO() {
    int rawValue = analogRead(MQ7_PIN);
    float voltage = (rawValue / 1023.0) * VREF;
    float RS = (VREF - voltage) / voltage * RL;  // Calcolo RS
    float ratio = R0 / RS;  // Normalizzazione rispetto a R0

    // Costanti derivate dal datasheet MQ-7
    float a = 27.0;  
    float b = -3.0;

    // Formula per calcolare i PPM
    float ppm =a*pow(ratio, b);
    return ppm;
}

// Funzione per disegnare la barra di livello sul display OLED
void drawCOBar(float ppm) {
    int barWidth = mapFloat(ppm, MIN_PPM, MAX_PPM, 0, 120); // Mappa ppm su 120px

    u8g2.clearBuffer();  
    u8g2.setFont(u8g2_font_ncenB08_tr);

    // Stampa il valore numerico
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

// Funzione per il buzzer (frequenza beep crescente con la CO)
void alertBuzzer(float ppm) {
    if (ppm > 50) { // Soglia minima per attivare il beep
        int beepDelay = mapFloat(ppm, 50, MAX_PPM, 1000, 100); // Più CO, più veloce il beep
        beepDelay = constrain(beepDelay, 1, 2000); // Limita i valori estremi

        tone(BUZZER_PIN, 1000); // Suona a 1000Hz
        delay(50);
        noTone(BUZZER_PIN);
        delay(beepDelay);
    }
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
