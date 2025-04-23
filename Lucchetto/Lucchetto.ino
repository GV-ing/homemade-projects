/*
 Controlling a servo position using a potentiometer (variable resistor)
 by Michal Rinott <http://people.interaction-ivrea.it/m.rinott>

 modified on 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Knob
*/

#include <Servo.h>

Servo myservo;  

int servoPin = 9;  // analog pin used to connect the potentiometer
int magnete = 2;
   // variable to read the value from the analog pin

void setup() {
  myservo.attach(servoPin);  // attaches the servo on pin 9 to the Servo object
}

void loop() {
  if (digitalRead(magnete)){
  myservo.write(90);  
  }else{ 
  myservo.write(0);  
  }                // sets the servo position according to the scaled value
  delay(15);                           // waits for the servo to get there
}
