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
    
	val=adc1_get_raw(ADC1_CHANNEL_4);   //32
	if(val>sensormax[0]){ sensormax[0]=val;}else if(val<sensormin[0]){ sensormin[0]=val;}

    val=adc1_get_raw(ADC1_CHANNEL_7);   //35
	if(val>sensormax[1]){ sensormax[1]=val;}else if(val<sensormin[1]){ sensormin[1]=val;}

    val=adc1_get_raw(ADC1_CHANNEL_6);   //34
	if(val>sensormax[2]){ sensormax[2]=val;}else if(val<sensormin[2]){ sensormin[2]=val;}   
    
    val=adc1_get_raw(ADC1_CHANNEL_3);   //39
	if(val>sensormax[3]){ sensormax[3]=val;}else if(val<sensormin[3]){ sensormin[3]=val;}

	val=adc1_get_raw(ADC1_CHANNEL_0);   //36
	if(val>sensormax[4]){ sensormax[4]=val;}else if(val<sensormin[4]){ sensormin[4]=val;}

    val=adc1_get_raw(ADC1_CHANNEL_5);   //33
	if(val>sensormax[5]){ sensormax[5]=val;}else if(val<sensormin[5]){ sensormin[5]=val;}


	adc2_get_raw( ADC2_CHANNEL_4,ADC_WIDTH_12Bit,&val); val=4095-val; //13
	if(val>sensormax[6]){ sensormax[6]=val;}else if(val<sensormin[6]){ sensormin[6]=val;}

	adc2_get_raw(ADC2_CHANNEL_3,ADC_WIDTH_12Bit,&val); val=4095-val; //15
	if(val>sensormax[7]){ sensormax[7]=val;}else if(val<sensormin[7]){ sensormin[7]=val;}

	adc2_get_raw(ADC2_CHANNEL_2,ADC_WIDTH_12Bit,&val); val=4095-val; //2
	if(val>sensormax[8]){ sensormax[8]=val;}else if(val<sensormin[8]){ sensormin[8]=val;}

	adc2_get_raw(ADC2_CHANNEL_0,ADC_WIDTH_12Bit,&val); val=4095-val; //4
	if(val>sensormax[9]){ sensormax[9]=val;}else if(val<sensormin[9]){ sensormin[9]=val;}

	adc2_get_raw(ADC2_CHANNEL_5,ADC_WIDTH_12Bit,&val); val=4095-val; //12
    if(val>sensormax[10]){ sensormax[10]=val;}else if(val<sensormin[10]){ sensormin[10]=val;}
  
}


void lecturasensores(){

	int sensor[11];
    int sumaValores = 0;
    int sumaPonderada = 0;
    
    sensor[0]=adc1_get_raw(ADC1_CHANNEL_4);   //32
    sensor[1]=adc1_get_raw(ADC1_CHANNEL_7);   //35
    sensor[2]=adc1_get_raw(ADC1_CHANNEL_6);   //34
    sensor[3]=adc1_get_raw(ADC1_CHANNEL_3);   //39
    sensor[4]=adc1_get_raw(ADC1_CHANNEL_0);   //36
    sensor[5]=adc1_get_raw(ADC1_CHANNEL_5);   //33
	
	adc2_get_raw( ADC2_CHANNEL_4,ADC_WIDTH_12Bit,&sensor[6]); sensor[6]=4095-sensor[6]; //13
	adc2_get_raw(ADC2_CHANNEL_3,ADC_WIDTH_12Bit,&sensor[7]); sensor[7]=4095-sensor[7]; //15
	adc2_get_raw(ADC2_CHANNEL_2,ADC_WIDTH_12Bit,&sensor[8]); sensor[8]=4095-sensor[8]; //2
	adc2_get_raw(ADC2_CHANNEL_0,ADC_WIDTH_12Bit,&sensor[9]); sensor[9]=4095-sensor[9]; //4
	adc2_get_raw(ADC2_CHANNEL_5,ADC_WIDTH_12Bit,&sensor[10]); sensor[10]=4095-sensor[10]; //12


  //  Serial.println(dato);

    int indice=-5;

    num=0;
    for (int i = 0; i < 11; i++) {    
      sumaValores += sensor[i];
      sumaPonderada += sensor[i] * indice;
      indice++;
      if(sensor[i]>600){ num++; }      
    }
  
    float posicion2 = ((float)(sumaPonderada) / (float)sumaValores)*100;

    if(num!=0){ 
       // posicion=(posicion*0.8)+(posicion2*0.2);        
		 posicion=posicion2;

    }else{  //En el caso de que se salga se para todo

       // pwm(0,0);pwm(1,0);
	   // pwm(2,0);pwm(3,0);

       // delay(5000);
    }

}


void Task1code( void * pvParameters ){    // Lectura sensore IR

  int tiempo=millis();
  
  for(;;){  

	 lecturasensores();   ///Mete un retardo de 500us
	 
	 wdt();

     if((millis()-tiempo)>100){

        Serial.println(d);
        
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
    while((millis()-tiempo)<5000){
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

        
    vlin=580; 
    Kp=1.0;       
    Kd=120.0;   
    

    pid=(Kp*p)+(Kd*d);  

    int pidmax=vlin;

    pid = constrain(pid,-pidmax,pidmax);
    
	if(abs(posicion)>=500){
		if(posicion<0){
			pwm(0,0);pwm(1,vlin);
			pwm(3,0);pwm(2,vlin/2);
		}else{
			pwm(1,0);pwm(0,vlin/2);
			pwm(2,0);pwm(3,vlin);
		}
	}else{
	
		pwm(0,0);pwm(1,vlin-pid);
		pwm(2,0);pwm(3,vlin+pid);

	}


  //  pwm(2,2000);
	
	delay(1);    
  
}
