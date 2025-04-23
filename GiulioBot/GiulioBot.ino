#include <MPU6050_tockn.h>
#include <Wire.h>
#include <Arduino_FreeRTOS.h>
#include "DLPF.h" 
#include "PID.h"


#define     GRAD_TO_RAD     PI/180.0 
#define     RAD_TO_GRAD     180.0/PI 

#define     SOGLIA          0.001 
#define     KP              0.008
#define     KI              0.001
#define     KD              0.00

#define     LED1_PIN        13
#define     F_C_MEAS        50.0 //Hz
#define     T_S_LECTURE     1.0/100.0 //s
#define     T_S_STAMP       1.0/50.0 //s
#define     T_S_CTRL        1.0/1.0


static double velX = 0.0; 
static double velY = 0.0;

double acc_x_offset;
double acc_y_offset;
const double G = 9.80665;  
double vx, vy;
double acc_x, acc_y, z, z_;
const double T_s_lecture = T_S_LECTURE* 1000, T_s_stamp = T_S_STAMP* 1000, T_s_ctrl = T_S_CTRL* 1000;
double s;
double rif =90.00;

PID PIDs (SOGLIA, KP , KI, KD);
MPU6050 mpu6050(Wire);

LPF LPF_acc_x(F_C_MEAS, T_S_LECTURE);
LPF LPF_acc_y(F_C_MEAS, T_S_LECTURE);
LPF LPF_z(F_C_MEAS, T_S_LECTURE);

void TaskStamp(void *pvParameters);
void TaskLetturaIMU(void *pvParameters);
void Task3(void *pvParameters);

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

  
  xTaskCreate(TaskLetturaIMU, "LetturaIMU", 128, NULL,2, NULL);
  xTaskCreate(Task3, "task3", 64, NULL, 2, NULL);
  xTaskCreate(TaskStamp, "Stampa", 128, NULL, 3, NULL);

  vTaskStartScheduler();
}



void TaskLetturaIMU(void *pvParameters) {
  (void) pvParameters; // Silenzia il warning del parametro non usato
  acc_x_offset=0.6;
  acc_y_offset=0.03;
  while (1) {
    mpu6050.update();
    acc_x= LPF_acc_x.update((mpu6050.getAccX())*G);
    acc_y= LPF_acc_y.update((mpu6050.getAccY())*G);
    updateVelocity(acc_x, acc_y, T_S_LECTURE);

    z= LPF_z.update(mpu6050.getAngleZ());
    s= PIDs.signal (rif, z, T_S_LECTURE);
    
    vTaskDelay(T_s_lecture/ portTICK_PERIOD_MS); // (non bloccante)
  }
}

void TaskStamp(void *pvParameters) {
  (void) pvParameters; // Silenzia il warning del parametro non usato
  
  while (1) {
    //Serial.print(mpu6050.getAccZ()*G);Serial.print(",");Serial.print(acc_x);Serial.print(",");Serial.print(acc_y);Serial.println(";");
    Serial.print(vx);Serial.print(",");Serial.print(vy);Serial.println(";");
    //Serial.print(z);Serial.print(",");Serial.print(acc_x);Serial.print(",");Serial.print(acc_y);Serial.println(";");
    vTaskDelay(T_s_stamp / portTICK_PERIOD_MS); // Aspetta 1000 ms (non bloccante)
  }
}

void Task3(void *pvParameters) {
  (void) pvParameters; // Silenzia il warning del parametro non usato
  
  while (1) {
    digitalWrite(LED1_PIN, !digitalRead(LED1_PIN)); // Inverte lo stato del LED
    vTaskDelay(T_s_ctrl / portTICK_PERIOD_MS); // Aspetta 1000 ms (non bloccante)
  }
}


void updateVelocity(double accX, double accY, double DT) {
    // integrazione semplice: v = v_prev + a * dt
    velX = velX + accX * DT;
    velY = velY + accY * DT;

    // output
    vx = velX;
    vy = velY;
}

void loop() {}