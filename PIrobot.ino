#include "PinChangeInterrupt.h"

String  inputString = "";
bool stringComplete = false;
const char separator = ',';
const int dataLength = 4;
float data[dataLength];


//Llanta izquierda 

const byte in1 = 9;
const byte in2 = 10;
const byte enA = 11;

const byte c1_1 = 3;
const byte c2_1 = 2;

volatile long contador_1 = 0;
volatile byte ant_1 = 0;
volatile byte act_1 = 0;

//Llanta derecha

const byte ind1 = 6;
const byte ind2 = 7;
const byte enA2 = 8;

const byte c1_2 = 5;
const byte c2_2 = 4;

volatile long contador_2 = 0;
volatile byte ant_2 = 0;
volatile byte act_2 = 0;


//Tiempo de muestreo 

unsigned long lastTime = 0, currentTime = 0, sampleTime = 0, tt = 0;

//variables PID llanta izquierda 


double WLreal = 0.0, WLd = 0.0, WLdd = 0.0;
double Pterm1 = 0.0, DTerm1 = 0.0, ITerm1  = 0.0, lastWLreal = 0.0;
double kp1 = 0.0, ki1 = 0.0, kd1 = 0.0;
double error1 = 0.0, error1_ant = 0.0, cumerro1 = 0.0, error1p = 0.0;
byte pwm1 = 0;
double u1 = 0.0;  

//Variables de la llanta derecha 

double WRreal = 0.0, WRd = 0.0, WRdd = 0.0;
double Pterm2 = 0.0, DTerm2 = 0.0, ITerm2 = 0.0, lastWRreal = 0.0;
double kp2 = 0.0, ki2 = 0.0, kd2 = 0.0;
double error2 = 0.0, error2_ant = 0.0, cumerro2 = 0.0, error2p = 0.0;
byte pwm2 = 0;
double u2 =0.; 

//Filtros llanta izquierda 

double outMin = 0.0, outMax = 0.0;
const double alfa = 0.7, valueConst1 = 104.71, R = 60;
double t = 0.0;

//Filtros llanta derecha

 double outMin2 = 0.0, outMax2 = 0.0;
 const double beta = 0.0, valueConst2 = 0.0, R2 = 0.0;
 double t2 = 0.0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(38400);

  pinMode(c1_1,INPUT);
  pinMode(c2_1,INPUT);
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);
  pinMode(enA,OUTPUT);

  digitalWrite (in1,false);
  digitalWrite (in2,false);

  pinMode(c1_2,INPUT);
  pinMode(c2_2,INPUT);
  pinMode(ind1,OUTPUT);
  pinMode(ind2,OUTPUT);
  pinMode(enA2,OUTPUT);

  digitalWrite(ind1,false);
  digitalWrite(ind2,false);
  
  //declaramos interrupciones encoders izquierdos 

  attachInterrupt(digitalPinToInterrupt(c1_1),encoder_1,CHANGE );
  attachInterrupt(digitalPinToInterrupt(c2_1),encoder_1,CHANGE );

  //encoders derechos 

  attachInterrupt(digitalPinToInterrupt(c1_2),encoder_2,CHANGE );
  attachInterrupt(digitalPinToInterrupt(c2_2),encoder_2,CHANGE );

  //asignamos tiempo de muestreo
  sampleTime = 100;

  //Limites del controlador PID
  outMax = 10000;
  outMin = -10000;

  outMax2 = 1000;
  outMin2 = -1000;

  //Ganancias de PID
  //Llanta izquierda
  kp1 = 1;
  kd1 = 2;
  ki1 = 0.0099;

  //Lanta derecha 
  kp2 = 1;
  kd2 = 2;
  ki2 = 0.0099;

  WLd = 0.0;
  WRd = 0.0;
}


void loop() {
  // put your main code here, to run repeatedly:
  if(stringComplete){

    for(int i = 0;i < dataLength ; i++){
      int index = inputString.indexOf(separator);
      data[i] = inputString.substring(0,index).toFloat();
      inputString = inputString.substring(index +1);
    }
    WLd = data [0];
    WLdd = data [1];

    WRd = data [0];
    WRdd = data [1];

    Serial.println(WLreal,3);
    Serial.println(WRreal,3);

    inputString = "";
    stringComplete = false;
  }

  ControlPIDiz();
  ControlPIDde();

}

double ControlPIDiz(void){
  if (millis()-lastTime <= sampleTime){
    noInterrupts();
    
    WLreal = (valueConst1*contador_1)/(millis()-lastTime);
    lastTime = millis();
   contador_1 = 0;
    
    interrupts();

    //calculamos error de seguimiento 
    error1 = (WLd-WLreal);

    //termino proporcional
    Pterm1 = error1;
    
    //termino integral
    ITerm1 += error1*sampleTime;

  if(ITerm1 > 10000) ITerm1= 10000; else if (ITerm1 < -10000) ITerm1=-10000;//amti-windup
  
    //termino derivativo
    DTerm1 = (error1-error1_ant)/(sampleTime);

    //controlador PID
    
    u1 = kp1*Pterm1+ki1*ITerm1+kd1*DTerm1;
    
    //u1 = 100;

    if (u1 > 0) anticlockwise(in2,in1,enA,abs(u1));else clockwise(in2,in1,enA,abs(u1));
  }
  Serial.print("WLdeseado: "); Serial.print(WLd); Serial.print(",");
  Serial.print("WLreal: "); Serial.print(contador_1); Serial.print(",");
  Serial.println();
}

double ControlPIDde(void){
  if (millis()-lastTime <= sampleTime){
    noInterrupts();
    
   WRreal = (valueConst2*contador_2)/(millis()-lastTime);
   lastTime = millis();
   contador_2 = 0;
    
    interrupts();

    //calculamos error de seguimiento 
    error2 = (WRd-WRreal);

    //termino proporcional
    Pterm2 = error2;
    
    //termino integral
    ITerm2 += error2*sampleTime;

  if(ITerm2 > 10000) ITerm2 = 10000; else if (ITerm2 < -10000) ITerm2=-10000;//amti-windup
  
    //termino derivativo
    DTerm2 = (error2-error2_ant)/(sampleTime);

    //controlador PID
    
    u2 = kp2*Pterm2+ki2*ITerm2+kd2*DTerm2;
    
    //u1 = 100;

    if (u2 > 0) anticlockwise(ind2,ind1,enA2,abs(u2));else clockwise(ind2,ind1,enA2,abs(u2));
  }
  Serial.print("WRdeseado: "); Serial.print(WRd); Serial.print(",");
  Serial.print("WRreal: "); Serial.print(contador_2); Serial.print(",");
  Serial.println();
}

void encoder_1(){

  ant_1 = act_1;

  if (digitalRead(c1_1)==1) bitSet(act_1,0); else bitClear(act_1,0); 
  if (digitalRead(c2_1)==1) bitSet(act_1,1); else bitClear(act_1,1);

  if(ant_1==3 && act_1==1) contador_1--;
  if(ant_1==1 && act_1==0) contador_1--;
  if(ant_1==0 && act_1==2) contador_1--;
  if(ant_1==2 && act_1==3) contador_1--;

  if(ant_1==1 && act_1==3) contador_1++;
  if(ant_1==0 && act_1==1) contador_1++;
  if(ant_1==2 && act_1==0) contador_1++;
  if(ant_1==3 && act_1==2) contador_1++;
}

void encoder_2(){

  ant_2 = act_2;

  if (digitalRead(c1_2)==1) bitSet(act_2,0); else bitClear(act_2,0); 
  if (digitalRead(c2_2)==1) bitSet(act_2,1); else bitClear(act_2,1);

  if(ant_2==3 && act_2==1) contador_2--;
  if(ant_2==1 && act_2==0) contador_2--;
  if(ant_2==0 && act_2==2) contador_2--;
  if(ant_2==2 && act_2==3) contador_2--;

  if(ant_2==1 && act_2==3) contador_2++;
  if(ant_2==0 && act_2==1) contador_2++;
  if(ant_2==2 && act_2==0) contador_2++;
  if(ant_2==3 && act_2==2) contador_2++;
}


void clockwise(int pin1, int pin2, int analogPin, int pwm){
  digitalWrite (pin1,LOW);
  digitalWrite (pin2,HIGH);
  analogWrite(analogPin,pwm);
}

void anticlockwise(int pin1, int pin2, int analogPin, int pwm){
  digitalWrite (pin1,HIGH);
  digitalWrite (pin2,LOW);
  analogWrite(analogPin,pwm);
}

void serialEvent(){
  while (Serial.available()){
    char inChar = (char)Serial.read();
    inputString += inChar;
    if(inChar = '\n'){
      stringComplete = true;
    }
  }
}
