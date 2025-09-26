#include <Wire.h>
#include <U8g2lib.h>

#define MQ7_PIN A2
#define RL 10.0  // Resistenza di carico in kΩ
#define VREF 5.0 // Tensione di riferimento di Arduino

// Inizializzazione del display OLED (SH1106 128x64 su I2C)
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

void setup() {
    u8g2.begin();  // Avvia il display
    pinMode(MQ7_PIN, INPUT);
}

void loop() {
    int sensorValue = analogRead(MQ7_PIN);  
    float voltage = (sensorValue / 1023.0) * VREF;  
    float RS = (VREF - voltage) / voltage * RL;  

    // Disegna il valore sul display OLED
    u8g2.clearBuffer();                  // Cancella il buffer dello schermo
    u8g2.setFont(u8g2_font_ncenB08_tr);  // Imposta il font
    u8g2.setCursor(10, 20);              // Posiziona il cursore
    u8g2.print("RS in aria pulita:");
    u8g2.setCursor(10, 40);
    u8g2.print(RS, 2);                   // Stampa il valore con 2 decimali
    u8g2.print(" kOhm");
    u8g2.sendBuffer();                    // Mostra il buffer sul display

    delay(1000);
}
