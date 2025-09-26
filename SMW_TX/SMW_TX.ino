//////////////////////////////////////////////////
// ARDUINO TRASMETTITORE DATI DHT11 CON nRF24L01
// Autore Fancello Salvatore
//Per maggiori info visitare: http://www.progettiarduino.com
////////////////////////////////////////////////

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <DHT11.h>

int pin = A0;
DHT11 dht11(pin);
float temperature[2];

double Fahrenheit(double celsius) {
return ((double)(9 / 5) * celsius) + 32;
}

double Kelvin(double celsius) {
return celsius + 273.15;
}

RF24 radio(7, 8);
const uint64_t pipe = 0xE8E8F0F0E1LL;

void setup(void) {
  Serial.begin(9600);
radio.begin();
radio.openWritingPipe(pipe);
}

void loop(void)
{
Serial.println(dht11.readTemperature());
temperature[0] = dht11.readTemperature();
temperature[1] = dht11.readHumidity();
radio.write(temperature, sizeof(temperature));
delay(1000);
}
