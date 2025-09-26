
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <Encoder.h>

#define pulsante 9
#define encoderCLK 5 
#define encoderDT 6



Encoder myEnc(encoderCLK,encoderDT);
float t,h;
float T=0;
float U=0;
int pag = 0;
int prev = 0;
float v = 0.0;

LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display



void ENCODER() {
  int newPosition = myEnc.read();
  if (newPosition != prev) {
    if (newPosition < prev ) {
      if (pag == 1) {
      T--;
      }  else if (pag == 2) {
      U--;  
      }
      } else {
      if (pag == 1) {
      T++;
      } else if (pag == 2) {
      U++; 
      }                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
      }
      lcd.clear();
      MENU ();
    prev = newPosition;
  }
  
}


void PULSANTE(){ 
    if (!digitalRead(pulsante)) {
    pag++;
    lcd.clear();
    delay(100);
    if (pag == 3) {
      pag = 0;
    }
     MENU ();
  }
}



void MENU (){
  if (pag == 0) {
    lcd.setCursor(0, 0);
    lcd.print("Temperatura ");
    lcd.print(t);
    lcd.print("C°");

    lcd.setCursor(0, 1);
    lcd.print("Umidita");
    lcd.setCursor(12, 1);
    lcd.print(h);
    lcd.print("%");


  } else if (pag == 1) {
    lcd.setCursor(0, 0);
    lcd.print("Temperatura");
    lcd.setCursor(0, 1);
    lcd.print("Impostata ");
    lcd.setCursor(12, 1);
    lcd.print(T);
    lcd.print(" C");

  } else if (pag == 2) {
    lcd.setCursor(0, 0);
    lcd.print("Umidita");
    lcd.setCursor(0, 1);
    lcd.print("Impostata  ");
    lcd.setCursor(13, 1);
    lcd.print(U);
    lcd.print("%");
  }
}

void setup()
{
  pinMode(encoderCLK, INPUT);
  pinMode(encoderDT, INPUT); 
  pinMode(pulsante, INPUT_PULLUP);
  Serial.begin(9600);
  lcd.init();                     
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("ACCENSIONE");
  delay(1000);
  lcd.clear();
  MENU ();
}


void loop(){
  ENCODER();
  PULSANTE();
}
