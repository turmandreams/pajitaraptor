#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <esp_task_wdt.h>

TaskHandle_t Task1; 


#include <Adafruit_NeoPixel.h>

#define PIN_WS2812B 14   
#define NUM_PIXELS 1   

Adafruit_NeoPixel ws2812b(NUM_PIXELS,PIN_WS2812B,NEO_GRB + NEO_KHZ800);


#define motorA 16
#define motorB 17
#define fan 18

boolean muevemotores=true;

float posicion=0;

float pid;
float error_anterior;

int sensormax[11];
int sensormin[11];

int tiempo=0;

int estado=-1;
int estadoanterior=-1;

void wdt(){
    esp_task_wdt_init(30,false);
    esp_task_wdt_reset();
    yield(); 
}


void pwm(int canal,int duty){

    if((!muevemotores)&&(duty!=0)) { return; }
    int dutyc=0;

    do{ 
        ledcWrite(canal,duty);        
        dutyc=ledcRead(canal);
    }while(duty!=dutyc);        
       
}


void calibrasensores(){

    int val=0;
    
    val=255-analogRead(32);  if(val>sensormax[0]){ sensormax[0]=val;}else if(val<sensormin[0]){ sensormin[0]=val;}
    val=255-analogRead(35);  if(val>sensormax[1]){ sensormax[1]=val;}else if(val<sensormin[1]){ sensormin[1]=val;}
    val=255-analogRead(34);  if(val>sensormax[2]){ sensormax[2]=val;}else if(val<sensormin[2]){ sensormin[2]=val;}   
    val=255-analogRead(39);  if(val>sensormax[3]){ sensormax[3]=val;}else if(val<sensormin[3]){ sensormin[3]=val;}
    val=255-analogRead(36);  if(val>sensormax[4]){ sensormax[4]=val;}else if(val<sensormin[4]){ sensormin[4]=val;}
    val=255-analogRead(33);  if(val>sensormax[5]){ sensormax[5]=val;}else if(val<sensormin[5]){ sensormin[5]=val;}
    val=255-analogRead(13);  if(val>sensormax[6]){ sensormax[6]=val;}else if(val<sensormin[6]){ sensormin[6]=val;}
    val=255-analogRead(15);  if(val>sensormax[7]){ sensormax[7]=val;}else if(val<sensormin[7]){ sensormin[7]=val;}
    val=255-analogRead(2);   if(val>sensormax[8]){ sensormax[8]=val;}else if(val<sensormin[8]){ sensormin[8]=val;}
    val=255-analogRead(4);   if(val>sensormax[9]){ sensormax[9]=val;}else if(val<sensormin[9]){ sensormin[9]=val;}
    val=255-analogRead(12);  if(val>sensormax[10]){ sensormax[10]=val;}else if(val<sensormin[10]){ sensormin[10]=val;}
  



}

void setup(){

    Serial.begin(115200);

    ws2812b.begin(); 
    
    analogReadResolution(8);
    
    pinMode(12,INPUT);
    pinMode(4,INPUT);
    pinMode(36,INPUT);
    pinMode(39,INPUT);
    pinMode(34,INPUT);
    pinMode(35,INPUT);
    pinMode(32,INPUT);
    pinMode(33,INPUT);
    pinMode(13,INPUT);
    pinMode(15,INPUT);
    pinMode(2,INPUT);


    pinMode(motorA,OUTPUT);
    pinMode(motorB,OUTPUT);

    ledcAttachPin(motorA,0);ledcSetup(0,15000,12);
    ledcAttachPin(motorB,1);ledcSetup(1,15000,12);
    ledcAttachPin(fan,2);ledcSetup(2,15000,12);

    
    ledcWrite(0,0);ledcWrite(1,0);ledcWrite(2,0);
    tiempo=millis();

     xTaskCreatePinnedToCore(
                    Task1code,   /* Task function. */
                    "Task1",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */ 

                    
    estado=0;delay(300);              
    estado=-1;delay(300);              
    estado=0;delay(300);              
    estado=-1;delay(300);              

    estado=2;
    while((millis()-tiempo)<5000){
        calibrasensores();
        yield();      
    }

    estado=-1;delay(300);              
    estado=0;delay(300);              
    estado=-1;delay(300);              
    estado=0;delay(300);              
    
    //Activamos el ventilador de succion
    //ledcWrite(2,1000);
    
}


void Task1code( void * pvParameters ){    // Lectura sensore IR

  int tiempo=millis();
  
  for(;;){  

    int sensor[11];
    int sumaValores = 0;
    int sumaPonderada = 0;
    
    String dato="";
    int val=255-analogRead(32); sensor[0]=map(val,sensormin[0],sensormax[0],0,255); dato+=sensor[0];dato+=",";
    val=255-analogRead(35); sensor[1]=map(val,sensormin[1],sensormax[1],0,255); dato+=sensor[1];dato+=",";
    val=255-analogRead(34); sensor[2]=map(val,sensormin[2],sensormax[2],0,255); dato+=sensor[2];dato+=",";    
    val=255-analogRead(39); sensor[3]=map(val,sensormin[3],sensormax[3],0,255); dato+=sensor[3];dato+=",";
    val=255-analogRead(36); sensor[4]=map(val,sensormin[4],sensormax[4],0,255); dato+=sensor[4];dato+=",";
    val=255-analogRead(33); sensor[5]=map(val,sensormin[5],sensormax[5],0,255); dato+=sensor[5];dato+=",";
    val=255-analogRead(13); sensor[6]=map(val,sensormin[6],sensormax[6],0,255); dato+=sensor[6];dato+=",";
    val=255-analogRead(15); sensor[7]=map(val,sensormin[7],sensormax[7],0,255); dato+=sensor[7];dato+=",";
    val=255-analogRead(2);  sensor[8]=map(val,sensormin[8],sensormax[8],0,255); dato+=sensor[8];dato+=",";
    val=255-analogRead(4);  sensor[9]=map(val,sensormin[9],sensormax[9],0,255); dato+=sensor[9];dato+=",";
    val=255-analogRead(12); sensor[10]=map(val,sensormin[10],sensormax[10],0,255); dato+=sensor[10];
  //  Serial.println(dato);

    int indice=-5;

    int num=0;
    for (int i = 0; i < 11; i++) {    
      sumaValores += sensor[i];
      sumaPonderada += sensor[i] * indice;
      indice++;
      if(sensor[i]>40){ num++; }      
    }
  
    float posicion2 = ((float)(sumaPonderada) / (float)sumaValores)*100;

    if(num!=0){ 
        posicion=posicion2;         
    }else{
       // pwm(0,0);pwm(1,0);
       // delay(5000);
    }


     wdt();    

     if((millis()-tiempo)>100){

        //Serial.println(pid);
        
        if(estado!=estadoanterior){
          
            if(estado==-1){  ws2812b.clear(); }
            else if(estado==0){  ws2812b.setPixelColor(0,ws2812b.Color(0,0,20)); }
            else if(estado==1){  ws2812b.setPixelColor(0,ws2812b.Color(20,0,0)); }
            else if(estado==2){  ws2812b.setPixelColor(0,ws2812b.Color(0,20,0)); }
            else if(estado==3){  ws2812b.setPixelColor(0,ws2812b.Color(20,20,20)); }

            ws2812b.show();
            estadoanterior=estado;
        }

        tiempo=millis();
     }
    
  } 
  
}


void loop(){

    int vlin;    
    float Kp,Kd;    
  
    float error=posicion;          

    ///Se activa el ventilador de succion
    //if(abs(error)>=3){ ledcWrite(2,1200); }
    //else{ ledcWrite(2,0); }
    
    float p=error;  
    float d = error-error_anterior;
    error_anterior = error;     

        
    //vlin=500; 
    int vmax=700;
    int vmin=400;    
    vlin=map(abs(error),0,500,vmax,vmin);     
    vlin=constrain(vlin,vmin,vmax);
    
    ///Hay que implementar valores adaptativos de Kp y Kd , dependiendo de la velocidad
    Kp=1.50;Kd=80.0;   

    pid=(Kp*p)+(Kd*d);  

    if(abs(pid)<=100){ vlin=1000;}

    int pidmax=vlin;
    pid = constrain(pid,-pidmax,pidmax);

       
    pwm(0,vlin-pid);
    pwm(1,vlin+pid); 

    int succionmax=3000;
    int succion=map(abs(error),0,500,0,succionmax);
    succion=constrain(succion,0,succionmax);
    
    pwm(2,succion);
    
    delay(1);    
    
  
}
