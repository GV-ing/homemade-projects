#include <Servo.h>
#include "Melodia.h"

#define PIN_ANT_DX 11
#define PIN_ANT_SX 10
#define PIN_POST_DX 9
#define PIN_POST_SX 3
#define PIN_BUZZER 6

#define T_S_CONTROL 1.0/200.0
#define T_S_MOTOR 1.0/400.0
#define T_S_VOICE 3.0

Melodia m(PIN_BUZZER);

Servo AntDX;
Servo AntSX;
Servo PostDX;
Servo PostSX;


const double  T_s_motor= T_S_MOTOR*2000, T_s_voice = T_S_VOICE * 2000, T_s_control= T_S_CONTROL * 1000;

int a_dx=0, a_sx=0 , p_dx=0, p_sx=0;
int a_dx_offset=100, a_sx_offset=80 , p_dx_offset=90, p_sx_offset=90;

int state= 1;
int limite_sup=70, limite_inf=-40;

double t_motor_old = 0.00, t_voice_old = 0.00, t_control_old = 0.00;


void setup() {
  AntDX.attach(PIN_ANT_DX);
  AntSX.attach(PIN_ANT_SX);
  PostDX.attach(PIN_POST_DX);
  PostSX.attach(PIN_POST_SX);
  Serial.begin(11200);
  
  AntDX.write(a_dx_offset);
  AntSX.write(a_sx_offset); 
  PostSX.write(p_dx_offset);
  PostDX.write(p_sx_offset);

  delay(5000);
}

void loop() {

  /*if (millis()-t_control_old >= T_s_control && state==1){
    avanti();
    if (millis()>= T_s_control*1000){
      in_piedi();
      state=2;
    }
    t_control_old = millis();
  }
  if (millis()-t_control_old >= T_s_control && state==2){
    seduto();
    if (millis()>= T_s_control*2000){
      in_piedi();
      state=3;
    }
    t_control_old = millis();
  }
  if (millis()-t_control_old >= T_s_control && state==3){
    chiuso();
    if (millis()>= T_s_control*4000){
      state=4;
    }
    t_control_old = millis();
  }*/
  if (millis()-t_control_old >= T_s_control){
    avanti();
    //seduto();
    //chiuso(); 
    t_control_old = millis();
  }
  
  if (millis()-t_motor_old >= T_s_motor){
    AntDX.write(a_dx_offset+a_dx);
    AntSX.write(a_sx_offset-a_sx); 
    PostSX.write(p_dx_offset+p_dx);
    PostDX.write(p_sx_offset-p_sx);
    t_motor_old = millis();
  }

  
  /*if (millis()-t_voice_old >= T_s_voice){
         
    //Serial.println("b");
    switch(emotion()){
    case 0:
        break;
    case 1:
        m.playHappyVerse();
        break;
    case 2:
        m.playAngerVerse();
        break;
    case 3:
        m.playSadVerse();
        break; 
    case 4:
        m.playHappySound();
        break;
    case 5:
        m.playAngerSound();
        break;
    case 6:
        m.playSadSound();
        break;
    }       
    t_voice_old = millis();
  }*/
  
}



int i=0;
int steps=40;
int current_steps =0;
void avanti(){
  switch(i){
    case 0:
        if (current_steps<steps){
           a_dx++;
           current_steps++;
           break;
        }else{
          i=2;
          current_steps=0;
          break;
        }

        
    case 1:
        if (current_steps<steps){
           p_sx++;
           current_steps++;
           break;
        }else{
          i=3;
          current_steps=0;
          break;
        }

       
    case 2:
        if (current_steps<steps){
           p_dx--;
           current_steps++;
           break;
        }else{
          i=1;
          current_steps=0;
          break;
        }
        break;

        
    case 3:
        if (current_steps<steps){
           a_sx--;
           current_steps++;
           break;
        }else{
          i=4;
          current_steps=0;
          break;
        }
        break;
    
    case 4:
        if (current_steps<steps){
           a_dx--;
           p_dx++;
           current_steps++;
           break;
        }else{
          i=5;
          current_steps=0;
          break;
        }
        break; 
    case 5:
        if (current_steps<steps){
           a_sx++;
           p_sx--;
           current_steps++;
           break;
        }else{
          i=6;
          current_steps=0;
          break;
        }
     case 6:
        if (current_steps<steps){
           a_sx++;
           current_steps++;
           break;
        }else{
          i=7;
          current_steps=0;
          break;
        }

        
    case 7:
        if (current_steps<steps){
           p_dx++;
           current_steps++;
           break;
        }else{
          i=8;
          current_steps=0;
          break;
        }

       
    case 8:
        if (current_steps<steps){
           p_sx--;
           current_steps++;
           break;
        }else{
          i=9;
          current_steps=0;
          break;
        }
        break;

        
    case 9:
        if (current_steps<steps){
           a_dx--;
           current_steps++;
           break;
        }else{
          i=10;
          current_steps=0;
          break;
        }
        break;
    
    case 10:
        if (current_steps<steps){
           a_sx--;
           p_sx++;
           current_steps++;
           break;
        }else{
          i=11;
          current_steps=0;
          break;
        }
        break; 
    case 11:
        if (current_steps<steps){
           a_dx++;
           p_dx--;
           current_steps++;
           break;
        }else{
          i=0;
          current_steps=0;
          break;
        }
        break;
    
    }   
}

void in_piedi(){
  a_dx=0;
  a_sx=0;
  p_sx=0;
  p_dx=0;
}

int j=0;
int steps_seduto=70;
void seduto(){
  
  switch(j){
    case 0:
        in_piedi();
        j=1;
        break;
        

        
    case 1:
        if (current_steps<steps_seduto){
           p_sx--;
           p_dx--;
           current_steps++;
           break;
        }else{
          j=2;
          current_steps=0;
          break;
        }
        
  }
}

int k=0;
int steps_chiuso=110;
void chiuso(){
  
  switch(k){
    case 0:
        in_piedi();
        k=1;
        break;
        

        
    case 1:
        if (current_steps<steps_chiuso){
           a_dx--;
           a_sx--;
           current_steps++;
           break;
        }else{
          k=2;
          current_steps=0;
          break;
        }
        
   case 2:
        if (current_steps<steps_chiuso){
           p_sx--;
           p_dx--;
           current_steps++;
           break;
        }else{
          k=3;
          current_steps=0;
          break;
        }
  }
}



int emotion(){
  return random(1,6);
}
