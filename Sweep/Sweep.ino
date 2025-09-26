

#include <Servo.h>

Servo myservo;  

int aperto  = 140;    
int chiuso  = -30;

int pulsante = 13;  
int val = 0;

void setup() {
  Serial.begin(9600);
  myservo.attach(9);  
  myservo.write(chiuso);
  pinMode(pulsante, INPUT);
}

void loop() {
val = digitalRead(pulsante); 
Serial.println(val);   

  if (val == 1){                         
    myservo.write(chiuso);
    delay(500);
  }  
  else {                                          
    myservo.write(aperto); 
  }  


 
  
}
