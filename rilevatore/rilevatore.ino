#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif
#define BUZZER_PIN 6
U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, 4, 5, U8X8_PIN_NONE);
int sensorPin = A2, value=0;
void screen(){
  u8g2.setFont(u8g_font_9x15B);
  u8g2.drawStr( 20, 20, "Luminosità: ");
  u8g2.setCursor(60,50);
  u8g2.print(value);
  u8g2.drawStr( 90, 50, "%");
}

void setup() {
  pinMode (BUZZER_PIN, OUTPUT) ;
  u8g2_SetI2CAddress(u8g2.getU8g2(), 0x3C * 2);
  u8g2.begin();
  tone(BUZZER_PIN, 5*10e2, 500);

}

void loop() {
  
  u8g2.firstPage();
  do {
      screen();
  } while ( u8g2.nextPage() );
  value = 100-map(analogRead(sensorPin),0 , 1024, 0 , 100); 
  tone(BUZZER_PIN, 5*10e1, 1000-(value*100));
}
