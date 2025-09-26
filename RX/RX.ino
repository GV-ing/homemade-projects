#include <EEPROM.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>


int q[3];
int qn[3];
RF24 wireless(7,8);
const byte address[6] = "00001";


Servo myservo1;
Servo myservo2;
Servo myservo3;


void setup() {
  Serial.begin(9600);
  myservo1.attach(4);
  myservo2.attach(5);
  myservo3.attach(6);
  wireless.begin();
  wireless.setChannel(110);
  wireless.stopListening();
  wireless.openReadingPipe(1, address);
  wireless.setPALevel(RF24_PA_HIGH);
  wireless.startListening();
  myservo1.write(EEPROM.read(0));
  myservo2.write(EEPROM.read(1));
  myservo3.write(EEPROM.read(2));
}
 
void loop() {
  
  if (wireless.available()) {
    wireless.read(&q, sizeof(q));
    }
    /*
      Serial.print(qn[0]);
      Serial.print("-");
      Serial.print(qn[1]);
      Serial.print("-");
      Serial.print(qn[2]);
      Serial.println();
    delay(150);

  myservo1.write(q[0]);
  myservo2.write(q[1]);
  myservo3.write(q[2]);*/
  for(int i=0; i<3; i++){
  if(q[i]>qn[i]){
    for (qn[i]; qn[i] < q[i]; qn[i] += 1) { 
    switch (i){
      case 0:
      myservo1.write(qn[0]);
      case 1:
      myservo2.write(qn[1]);
      case 2:
      myservo3.write(qn[2]);
      
    }
            
    delay(1);                     
  }
  }else if(q[i]<qn[i]){
    for (qn[i]; qn[i] >q[i]; qn[i] -= 1) { 
    switch (i){
      case 0:
      myservo1.write(qn[0]);
      case 1:
      myservo2.write(qn[1]);
      case 2:
      myservo3.write(qn[2]);
    }
    delay(1);                                                                                                                                                                                                                                                                                                                                         
  }
  }EEPROM.write(i,qn[i]);
  }
 delay(1);
}
