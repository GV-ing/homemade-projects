#include <Servo.h>
#define ServoPin 4
#define keyPin 6
bool state = LOW;

Servo myservo;
void setup() 
{
  myservo.attach(ServoPin);
  pinMode (keyPin, INPUT);
}

void loop()
{                              
  
  if (digitalRead(keyPin))
  {
    myservo.write(0);                 
    delay(5000);
    
  }
  else{
      myservo.write(90);                 
      delay(5000);
    }
}
