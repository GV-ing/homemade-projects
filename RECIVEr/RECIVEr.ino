#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>


char MES;

RF24 radio(9, 10);
const uint64_t pipe = 0xE8E8F0F0E1LL;

void setup(void) {
Serial.begin(9600);
radio.begin();
radio.openReadingPipe(1, pipe);
radio.startListening();
delay(1000);
}

void loop(void)
{
if ( radio.available() ){
radio.read(MES, sizeof(MES));
Serial.write(MES);
delay(500);

}

}
