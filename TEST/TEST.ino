#include <Wire.h>
#include <DHT11.h>

//#include <LCD.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
// Addr, En, Rw, Rs, d4, d5, d6, d7, backlighpin, polarity

float temperature[2];

RF24 radio(7, 8);
const uint64_t pipe = 0xE8E8F0F0E1LL;

void setup(void) {
Serial.begin(9600);
radio.begin();
radio.openReadingPipe(1, pipe);
radio.startListening();
lcd.begin(16, 2);
lcd.backlight();
lcd.clear();
lcd.print("Umid & temp");
delay(1000);
lcd.clear();
lcd.print("Start.....");
delay(1000);
}

void loop(void)
{
if ( radio.available() )
{
bool done = false;
while (!done)
{
done = radio.read(temperature, sizeof(temperature));
lcd.clear();
delay(500);
lcd.setCursor(0, 0);
lcd.print("Temp");
lcd.setCursor(0, 1);
lcd.print("Umid");
lcd.setCursor(9, 0);
lcd.print(temperature[0]);
lcd.print(" C");
lcd.setCursor(9, 1);
lcd.print(temperature[1]);
lcd.print(" %");
delay(5000);
}
}

}
