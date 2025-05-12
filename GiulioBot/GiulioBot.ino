#include <MPU6050_tockn.h>
#include <Wire.h>
#include "DLPF.h" 
#include "PID.h"
#include <AFMotor.h>

#define     GRAD_TO_RAD     PI/180.0 
#define     RAD_TO_GRAD     180.0/PI 

#define     SOGLIA          0.5
#define     KP              0.7
#define     KI              0.07
#define     KD              0.06
#define     PIN_M_dx        3
#define     PIN_M_sx        4
#define     LED1_PIN        13
#define     F_C_MEAS        50.0 //Hz
#define     T_S_LECTURE     1.0/200.0 //s
#define     T_S_STAMP       1.0/15.0 //s
#define     T_S_CTRL        1.0/50.0


static double velX = 0.0; 
static double velY = 0.0;
const double offset= 0.02;
double acc_x_offset;
double acc_y_offset;
const double G = 9.80665;  
double vx, vy;
double acc_x, acc_y, z, z_;
const double T_s_lecture = T_S_LECTURE* 1000, T_s_stamp = T_S_STAMP* 1000, T_s_ctrl = T_S_CTRL* 1000;
double T_s_lecture_old = 0, T_s_stamp_old = 0, T_s_ctrl_old = 0;
double s_z;
char s_v;
double rif_o =0.00;
double rif_v =0.00;

AF_DCMotor motorDX(PIN_M_dx);
AF_DCMotor motorSX(PIN_M_sx);

PID PIDs_z (SOGLIA, KP , KI, KD);
MPU6050 mpu6050(Wire);

LPF LPF_acc_x(F_C_MEAS, T_S_LECTURE);
LPF LPF_acc_y(F_C_MEAS, T_S_LECTURE);
LPF LPF_z(F_C_MEAS, T_S_LECTURE);



void setup() {
  Serial.begin(115200);
  pinMode(LED1_PIN, OUTPUT);

  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  mpu6050.update();

  LPF_acc_x.set_initial_condition(mpu6050.getAccX());
  LPF_acc_y.set_initial_condition(mpu6050.getAccY());
  LPF_z.set_initial_condition(mpu6050.getAngleZ());

  motorDX.run(RELEASE);
  motorSX.run(RELEASE);
  
}




void loop() {
//LETTURA IMU
  if (millis() - T_s_lecture_old >= T_s_lecture) {
    mpu6050.update();
    z= LPF_acc_y.update(mpu6050.getAngleZ());
    s_z = PIDs_z.signal (rif_o, z, T_S_LECTURE);
    T_s_lecture_old = millis();
  }
//CONTROLLO MOTORI
  if (millis() - T_s_ctrl_old >= T_s_ctrl) {
    Controllo_Orientamneto();
    Controllo_Velocita(s_v);
    T_s_ctrl_old = millis();
  }

//STAMPA VALORI e CAMBIO RIFERIMENTI
  if (millis() - T_s_stamp_old >= T_s_stamp) {
    
    char direction = "";
    //s_v=direction;
    static char inputBuffer[32];
    static uint8_t idx = 0;

    while (Serial.available() > 0) {
        char c = Serial.read();
        // Ignora CR
        if (c == '\r') continue;

        // Fine riga: processa il buffer
        if (c == '\n') {
          inputBuffer[idx] = '\0';
          
          // Se il buffer non è vuoto
          if (idx > 0) {
            // Se il primo carattere è una lettera A–Z o a–z
            if ( (inputBuffer[0] >= 'A' && inputBuffer[0] <= 'Z') ||
                (inputBuffer[0] >= 'a' && inputBuffer[0] <= 'z') ) {
              // Prendi solo il primo carattere
              direction = inputBuffer[0];
              s_v=direction;
            } else {
              // Proviamo a convertire in float
              char *endptr;
              float value = strtod(inputBuffer, &endptr);
              // Se endptr punta alla fine, la conversione è OK
              if (endptr != inputBuffer && *endptr == '\0') {
                // Controllo intervallo
                if (value >= -180.0f && value <= 180.0f) {
                  rif_o = value;
                } else {
                  Serial.println(F("Valore fuori range [-180,+180]"));
                }
              } else {
                Serial.println(F("Formato numero non valido"));
              }
            }
          }
          
          // Reset buffer per la prossima linea
          idx = 0;

        } else {
          // Accumula carattere finché c'è spazio
          if (idx < sizeof(inputBuffer) - 1) {
            inputBuffer[idx++] = c;
          }
        }
      }
    
  Serial.print(z);Serial.print(",");Serial.print(s_z);Serial.print(",");Serial.print(rif_o);Serial.print(",");Serial.print(s_v);Serial.println(";");
  //Serial.print(vy);Serial.print(",");Serial.print(s_v);Serial.print(",");Serial.print(rif_v);Serial.println(";");

  T_s_stamp_old = millis();
  }
}



void Controllo_Orientamneto(){
   if(s_z>0+ offset){
      motorDX.run(FORWARD);
      motorSX.run(BACKWARD);
      motorDX.setSpeed(int(s_z*255));
      motorSX.setSpeed(int(s_z*255));
      
    }else if (s_z<0 -offset){
      motorDX.run(BACKWARD);
      motorSX.run(FORWARD);
      motorDX.setSpeed(int(-s_z*255));
      motorSX.setSpeed(int(-s_z*255));
    }else if (abs(s_z)<=offset){
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
    case 's':
      motorDX.run(RELEASE);
      motorSX.run(RELEASE);    
      break;
    default :
      s_v = '';
      break;
   }
}
