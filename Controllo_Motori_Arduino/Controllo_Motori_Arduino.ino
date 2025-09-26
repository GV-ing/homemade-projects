#include <Servo.h>

// Pin di controllo motori
#define ML_Ctrl 4
#define ML_PWM 6
#define MR_Ctrl 2
#define MR_PWM 5

#define SERVO1_PIN 10
#define SERVO_PIN 9  // Pin D9 per il servomotore

// Variabili per il movimento
float xPosition = 0.0;  // Posizione del joystick sull'asse X
float yPosition = 0.0;  // Posizione del joystick sull'asse Y
float servoPosition = 0.0;
float leftSpeed = 0.0;    // Velocità motore sinistro
float rightSpeed = 0.0;   // Velocità motore destro
int Speed_Max = 255;      //Velocità massima


int LX_index ;
int RX_index ;
int RY_index ;
int servoAngle = 90, end_eff_relaxed = 120, end_eff_extended= 0;


// Variabili per il servomotore
Servo myServo;  // Crea un'istanza del servomotore
Servo myServo1;

void setup() {
  // Impostazione dei pin
  pinMode(SERVO_PIN, OUTPUT);
  pinMode(ML_Ctrl, OUTPUT);
  pinMode(ML_PWM, OUTPUT);
  pinMode(MR_Ctrl, OUTPUT);
  pinMode(MR_PWM, OUTPUT);

  Serial.begin(115200);  // Comunicazione seriale
  myServo1.attach(SERVO1_PIN);
  myServo.attach(SERVO_PIN);  // Collega il servomotore al pin D9
  myServo1.write(end_eff_relaxed);
  myServo.write(servoAngle);  // Imposta inizialmente il servomotore a 90° (centrato)
  String command = Serial.readStringUntil('\n');

}

void loop() {
  
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    // Debug: stampa il comando ricevuto
    Serial.println("Comando ricevuto: " + command);

    //Modalità di precisione on/off
    if (command == "B") {
      Speed_Max = 255.00;
      delay(5);
    } 
    else if (command == "B") {
      Speed_Max = 155.00;
      delay(5);
    } 
    else if (command == "A") {
      myServo1.write(end_eff_extended);
      delay(5);
    }
    else if (command == "A") {
      myServo1.write(end_eff_relaxed);
      delay(5);
    }
    else {
      // Se il comando non è M_off o M_on, controlla il movimento
      command_analysis(command);
    }
      // Controlla i motori
    controlMotors(leftSpeed, rightSpeed);
      // Controlla il servomotore
    myServo.write(servoAngle);  // Imposta l'angolo del servomotore    
  }
}

void command_analysis(String command){
  // Estrai i valori X, Y e Z dal comando
  // LX:-1.0,RX:-05,RY:-1.0
  LX_index = command.indexOf("LX:") + 3;
  RX_index = command.indexOf("RX:") + 3;
  RY_index = command.indexOf("RY:") + 3;

  // Estrai i valori
  String xStr = command.substring(LX_index, command.indexOf(",", LX_index));
  String yStr = command.substring(RX_index, command.indexOf(",", RX_index));
  String zStr = command.substring(RY_index);

  xPosition = yStr.toFloat();
  yPosition = zStr.toFloat();
  servoPosition = xStr.toFloat();  // Posizione per il servomotore

  // Debug: stampa i valori X, Y e Z ricevuti
  Serial.println("xPosition: " + String(xPosition));
  Serial.println("yPosition: " + String(yPosition));
  Serial.println("servoPosition: " + String(servoPosition));
  // Debug: stampa l'angolo del servomotore
  Serial.println("Angolo servomotore: " + String(servoAngle));

  // Calcola la velocità dei motori
  leftSpeed = (yPosition * Speed_Max) + (-xPosition * Speed_Max);
  rightSpeed = (yPosition * Speed_Max) + (xPosition * Speed_Max);
  if (leftSpeed > Speed_Max) leftSpeed = Speed_Max;
  if (rightSpeed > Speed_Max) rightSpeed = Speed_Max;
  if (leftSpeed < -Speed_Max) leftSpeed = -Speed_Max;
  if (rightSpeed < -Speed_Max) rightSpeed = -Speed_Max;

      
  servoAngle =(-servoPosition*45)+90;  // Mappa la posizione del joystick sull'asse X da -1 a 1 a un angolo da 0 a 180

}


// Funzione per il controllo dei motori
void controlMotors(float leftSpeed, float rightSpeed) {
  int k = 5;

  // Controllo del motore sinistro
  if (leftSpeed - k > 0) {
    digitalWrite(ML_Ctrl, LOW);  // Direzione avanti
    analogWrite(ML_PWM, (int)leftSpeed);
  } else if (leftSpeed + k < 0) {
    digitalWrite(ML_Ctrl, HIGH);  // Direzione indietro
    analogWrite(ML_PWM, (int)leftSpeed);
  } else {
    digitalWrite(ML_Ctrl, LOW);
    analogWrite(ML_PWM, 0);  // Stop motore sinistro
  }

  // Controllo del motore destro
  if (rightSpeed - k > 0) {
    digitalWrite(MR_Ctrl, LOW);  // Direzione avanti
    analogWrite(MR_PWM, (int)rightSpeed);
  } else if (rightSpeed + k < 0) {
    digitalWrite(MR_Ctrl, HIGH);  // Direzione indietro
    analogWrite(MR_PWM, (int)rightSpeed);
  } else {
    digitalWrite(MR_Ctrl, LOW);
    analogWrite(MR_PWM, 0);  // Stop motore destro
  }
}
