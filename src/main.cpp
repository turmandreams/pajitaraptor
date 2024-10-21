#include <Arduino.h>

#include <driver/adc.h>
#include <esp_adc_cal.h>


#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <esp_task_wdt.h>

TaskHandle_t Task1; 


#include <Adafruit_NeoPixel.h>

#define PIN_WS2812B 14   
#define NUM_PIXELS 1   

Adafruit_NeoPixel ws2812b(NUM_PIXELS,PIN_WS2812B,NEO_GRB + NEO_KHZ800);


#define motorA1 19
#define motorA2 18
#define motorB1 17
#define motorB2 16

#define fan 5


boolean muevemotores=true;

float posicion=0;

float pid;
float error_anterior;

int sensormax[11];
int sensormin[11];

int tiempo=0;

int estado=-1;
int estadoanterior=-1;

int num=0;

float Kp,Kd;   
float p,d;

int sensorcentral=0;

int sensorPins[11] = {39,35,32,34,36,33,13,15,2,12,4};


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
    
	for (int i = 0; i < 11; i++) {
		val = 255-analogRead(sensorPins[i]);
		if(val>sensormax[i]){ sensormax[i]=val;}else if(val<sensormin[i]){ sensormin[i]=val;}		
	}

	  
}



void lecturasensores(){
	
  float sensorValues[11];

  float sumaPesada = 0;   
  float sumaValores = 0;  
	
  int val;
  
  int peso=sensorcentral-2;
  peso=constrain(peso,-5,1);
  
  	
  int i=peso+5;
  for (int contador=0;contador<5;contador++) {
	val = 255-analogRead(sensorPins[i]);
    sensorValues[i] = map(val,sensormin[i],sensormax[i],0,4095);
    sumaPesada += sensorValues[i] * peso;
    sumaValores += sensorValues[i];
    peso++;
	i++;
  }

  
  if (sumaValores != 0) {  

	  posicion = sumaPesada / sumaValores;

	  sensorcentral=(int)posicion;

		
	  if(posicion==-5.0){ 
		int val=map(sensorValues[0],0,3400,500,0);
		val=constrain(val,0,500);
		posicion-=(float)val/1000.0;
	  }else if(posicion==5.0){ 
		int val=map(sensorValues[10],0,3400,500,0);
		val=constrain(val,0,500);
		posicion+=(float)val/1000.0;
	  } 	  
	  
	  
	  posicion=posicion*100;
  }

}




void Task1code( void * pvParameters ){    // Lectura sensore IR

  int tiempo=millis();
  
  for(;;){  

	 lecturasensores();   ///Mete un retardo de 500us
	 
	 wdt();

     if((millis()-tiempo)>100){

        //Serial.println(posicion);
        
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




void setup(){

    Serial.begin(115200);
	analogReadResolution(8);

    ws2812b.begin(); 
    
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


	pinMode(motorA1,OUTPUT);
	pinMode(motorA2,OUTPUT);
    pinMode(motorB1,OUTPUT);
    pinMode(motorB2,OUTPUT);
	

	ledcAttachPin(motorA1,0);ledcSetup(0,15000,12);
    ledcAttachPin(motorA2,1);ledcSetup(1,15000,12);
	ledcAttachPin(motorB1,2);ledcSetup(2,15000,12);
    ledcAttachPin(motorB2,3);ledcSetup(3,15000,12);




    ledcAttachPin(fan,2);ledcSetup(2,15000,12);
     
    ledcWrite(0,0);ledcWrite(1,0);
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
    while((millis()-tiempo)<10000){
        calibrasensores();
        yield();      
    }

    estado=-1;delay(300);              
    estado=0;delay(300);              
    estado=-1;delay(300);              
    estado=0;delay(300);              
    
    
    
}




void loop(){

    int vlin;    
    
    float error=posicion;          
    
    p=error;  
    d = error-error_anterior;
    error_anterior = error;     

        
    vlin=540; 
    Kp=0.5;    //0.5 va mas rapido pero mas inseguro
    Kd=100.0;   
		
    

    pid=(Kp*p)+(Kd*d);  

    int pidmax=vlin;

    pid = constrain(pid,-pidmax,pidmax);
    
	if(abs(posicion)>=550){
		if(posicion<0){
			pwm(0,0);pwm(1,vlin);
			pwm(3,0);pwm(2,vlin/16);
		}else{
			pwm(1,0);pwm(0,vlin/16);
			pwm(2,0);pwm(3,vlin);
		}
	}else{
	
		pwm(0,0);pwm(1,vlin-pid);
		pwm(2,0);pwm(3,vlin+pid);

	}


  //  pwm(2,2000);
	
	delay(1);    
  
}
