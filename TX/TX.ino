
#include <Encoder.h>
#include <EEPROM.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define PIN_A_ENCODER1 3
#define PIN_B_ENCODER1 4
#define PIN_BUTTON1    2
#define PIN_A_ENCODER2 5
#define PIN_B_ENCODER2 6
#define PIN_A_ENCODER3 9
#define PIN_B_ENCODER3 10


Encoder Encoder1(PIN_A_ENCODER1, PIN_B_ENCODER1);
Encoder Encoder2(PIN_A_ENCODER2, PIN_B_ENCODER2);
Encoder Encoder3(PIN_A_ENCODER3, PIN_B_ENCODER3);

int v = 5;


int letturaPrecedente1 = 0;
int letturaPrecedente2 = 0;
int letturaPrecedente3 = 0;


RF24 wireless(7, 8);
const byte address[6] = "00001";

int q[3];





void ENCODER1() {
  int n1 = Encoder1.read();
  if (letturaPrecedente1 != n1 ) {
    if (letturaPrecedente1 < n1 ) {
      q[0] = q[0] - v;
      if (q[0] <= 0) {
        q[0] = 0;
      }
      Serial.println(q[0]);
    } else {
      q[0] = q[0] + v;
      if (q[0] >= 180) {
        q[0] = 180;
      }
      Serial.println(q[0]);
    }
    
    letturaPrecedente1 = n1;
  }
}

void ENCODER2() {
  int n2 = Encoder2.read();
  if (letturaPrecedente2 != n2 ) {
    if (letturaPrecedente2 < n2 ) {
      q[1] = q[1] - v;
      if (q[1] <= 0) {
        q[1] = 0;
      }
      Serial.println(q[1]);
    } else {
      q[1] = q[1] + v;
      if (q[1] >= 180) {
        q[1] = 180;
      }
      Serial.println(q[1]);
    }
    
    letturaPrecedente2 = n2;
  }
}



void ENCODER3() {
 int n3 = Encoder3.read();
  if (letturaPrecedente3 != n3 ) {
    if (letturaPrecedente3 < n3 ) {
      q[2] = q[2] - v;
      if (q[2] <= 0) {
        q[2] = 0;
      }
      Serial.println(q[2]);
    } else {
      q[2] = q[2] + v;
      if (q[2] >= 180) {
        q[2] = 180;
      }
      Serial.println(q[2]);
    }
    
    letturaPrecedente3 = n3;
  }
}


void INVIO() {
  wireless.write(&q, sizeof(q));
  delay(1);
}


void VELOCITA() {
  if (PIN_BUTTON1) {
    v = v * 2;
    if (v > 180) {
      v = 1;
    }
  }
}



void setup() {
  pinMode (PIN_BUTTON1, INPUT);


  for (int i = 0; i < 3; i++) {
    q[i] = EEPROM.read(i);
  }
  pinMode (PIN_A_ENCODER1, INPUT);
  pinMode (PIN_B_ENCODER1, INPUT);
  pinMode (PIN_A_ENCODER2, INPUT);
  pinMode (PIN_B_ENCODER2, INPUT);
  pinMode (PIN_A_ENCODER3, INPUT);
  pinMode (PIN_B_ENCODER3, INPUT);
  Serial.begin(250000);
  wireless.begin();
  wireless.setChannel(110);
  wireless.stopListening();
  wireless.openWritingPipe(address);
  wireless.setPALevel(RF24_PA_HIGH);
  wireless.stopListening();
}

void loop() {

  ENCODER1();
  ENCODER2();
  ENCODER3();
  INVIO();

  for (int i = 0; i < 3; i++) {
    EEPROM.write(i, q[i]);
  }

}
