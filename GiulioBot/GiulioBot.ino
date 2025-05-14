#include <MPU6050_tockn.h>
#include <Wire.h>
#include "DLPF.h" 
#include "PID.h"
#include <AFMotor.h>

//Parametri PID
#define     SOGLIA          0.2
#define     KP              0.7
#define     KI              0.07
#define     KD              0.06
//PIN
#define     PIN_M_dx        3
#define     PIN_M_sx        4
#define     LED1_PIN        13
//Periodi di attivazione
#define     F_C_MEAS        50.0        //Hz
#define     T_S_LECTURE     1.0/200.0   //s
#define     T_S_STAMP       1.0/15.0    //s
#define     T_S_CTRL        1.0/50.0    //s


//variabili
double z, s_z;
double T_s_lecture_old = 0.00, T_s_stamp_old = 0.00, T_s_ctrl_old = 0.00;
double rif_o =0.00, error = 0.00;
char s_v;

//costanti
const double T_s_lecture = T_S_LECTURE* 1000.00, T_s_stamp = T_S_STAMP* 1000.00, T_s_ctrl = T_S_CTRL* 1000.00;

//Definizione motori
AF_DCMotor motorDX(PIN_M_dx);
AF_DCMotor motorSX(PIN_M_sx);

//Definizione PID
PID PIDs_z (SOGLIA, KP , KI, KD);

//Definizione IMU
MPU6050 mpu6050(Wire);

//Definizione DLPF
LPF LPF_z(F_C_MEAS, T_S_LECTURE);


void setup() {
  Serial.begin(115200);

  pinMode(LED1_PIN, OUTPUT);

  Wire.begin();

  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  mpu6050.update();
  LPF_z.set_initial_condition(mpu6050.getAngleZ());

  motorDX.run(RELEASE);
  motorSX.run(RELEASE);
  
}




void loop() {
//LETTURA IMU
  if (millis() - T_s_lecture_old >= T_s_lecture) {   /// Periodo = 5 ms  Frequenza 200 Hz  
    mpu6050.update();                                   ///Si occupa di leggere i valori dall'imu e calcolare il segnale di controllo uscente dal PID
    z= LPF_z.update(mpu6050.getAngleZ());
    s_z = PIDs_z.signal (rif_o, z, T_S_LECTURE);
    T_s_lecture_old = millis();
  }

//CONTROLLO MOTORI
  if (millis() - T_s_ctrl_old >= T_s_ctrl) {    ///Periodo 20ms   Frequenza 50 Hz
    Controllo_Orientamneto(s_z);                  ///si occupa di inviare il segnale di controllo ai motori
    Controllo_Velocita(s_v);
    T_s_ctrl_old = millis();
  }

//STAMPA VALORI e CAMBIO RIFERIMENTI
  if (millis() - T_s_stamp_old >= T_s_stamp) {     ///Periodo 66.66 ms  Frequenza 15 Hz
    cambio_riferimento();                           /// invia in seriale i valori di controllo, riceve dalla seriale i valori di riferimento 
    stampa();
    T_s_stamp_old = millis();
  }
}



void Controllo_Orientamneto(double signal){
   if(signal> SOGLIA){
      motorDX.run(FORWARD);
      motorSX.run(BACKWARD);
      motorDX.setSpeed(int(s_z*255));
      motorSX.setSpeed(int(s_z*255));
      
    }else if (signal< -SOGLIA){
      motorDX.run(BACKWARD);
      motorSX.run(FORWARD);
      motorDX.setSpeed(int(-s_z*255));
      motorSX.setSpeed(int(-s_z*255));

    }else if (abs(signal)<=SOGLIA){
      motorDX.run(RELEASE);
      motorSX.run(RELEASE);    
    }
}



void Controllo_Velocita(char signal){
   switch(signal){
    case 'f':
      motorDX.run(FORWARD);
      motorSX.run(FORWARD);
      motorDX.setSpeed(255);
      motorSX.setSpeed(255);
      break;
      
    case 'b':
      motorDX.run(BACKWARD);
      motorSX.run(BACKWARD);
      motorDX.setSpeed(255);
      motorSX.setSpeed(255);
      break;

    default :
      s_v ="\0" ;
      break;
   }
}


void cambio_riferimento() {
  static char inputBuffer[32];
  static uint8_t idx = 0;

  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      inputBuffer[idx] = '\0';

      if (idx > 0) {
        if ((inputBuffer[0] >= 'A' && inputBuffer[0] <= 'Z') ||
            (inputBuffer[0] >= 'a' && inputBuffer[0] <= 'z')) {
          s_v = inputBuffer[0];  // salva direzione
        } else {
          char *endptr;
          float value = strtod(inputBuffer, &endptr);
          if (endptr != inputBuffer && *endptr == '\0') {
            if (value >= -180.0f && value <= 180.0f) {
              rif_o += value;  // somma al riferimento attuale
            } else {
              Serial.println(F("Valore fuori range [-180,+180]"));
            }
          } else {
            Serial.println(F("Formato numero non valido"));
          }
        }
      }
      idx = 0;
    } else {
      if (idx < sizeof(inputBuffer) - 1) {
        inputBuffer[idx++] = c;
      }
    }
  }
}


void stampa() {
  error = rif_o - z;
  
  //Serial.print(error); Serial.print(,);
  Serial.print(z); Serial.print(",");
  Serial.print(s_z); Serial.print(",");
  Serial.print(rif_o); Serial.print(",");
  Serial.print(s_v); Serial.println(";");
}
