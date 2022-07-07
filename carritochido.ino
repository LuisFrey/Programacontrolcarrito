#include "PinChangeInterrupt.h"

String inputString = "";         // String para contener datos entrantes
bool stringComplete = false;     //bandera en falso de comunicacion
const char separator = ',';   //separador de datos
const int dataLength = 4;    //numero de datos
float data[dataLength];    //guardamos datos de un vector "data"
  
////////////////////// LINK 1 ///////////////////////////////

const byte    C1_1 = 3;                  // Entrada de la señal A del encoder.  (Ojo invertir canales)
const byte    C2_1 = 2;                  // Entrada de la señal B del encoder.
const byte    in1  = 9;                 
const byte    in2  = 10;         
const byte    enA = 11;                

volatile long contador_1 = 0; //interrupcion 1
volatile byte ant_1      = 0; //tick anterior 
volatile byte act_1      = 0; //tick actual

//Lanta derecha

const byte c1_2 = 4;
const byte c2_2 = 5;
const byte ind1 = 6;
const byte ind2 = 7;
const byte enA2 = 8;

volatile long contador_2 = 0;
volatile byte ant_2 = 0;
volatile byte act_2 = 0;

////////////////////// Tiempo de muestreo ///////////////////////////////

unsigned long lastTime = 0, currentTime = 0, sampleTime = 0;   

////////////////////// VARIABLES PID LINK 1 ///////////////////////////////
double        WLreal    = 0.0, WLd   = 0.0,  WLdd   = 0.0;             
double        Pterm1    = 0.0, DTerm1    = 0.0, ITerm1    = 0.0, lastq1real = 0.0; 
double        kp1       = 0.0, ki1         = 0.0, kd1         = 0.0;              
double        error1    = 0.0, error1_ant    = 0.0,  cumerror1   = 0.0, error1p   = 0.0;                                  
byte          pwm1      = 0;
double        u1    = 0.0;

////////////////////// VARIABLES PID LINK 2 ///////////////////////////////
double        WRd = 0.0, WRreal    = 0.0, WRdd = 0.0;
double        Pterm2 = 0.0, Dterm2 = 0.0, Iterm2 = 0.0, lastqrreal = 0.0;
double        kp2 = 0.0, ki2 = 0.0, kd2 = 0.0;
double        error2 = 0.0, error2_ant = 0.0, cumerror2 = 0.0, error2p = 0.0;
byte          pwm2 = 0;
double        u2 =0.0;  


////////////////////// FILTROS ///////////////////////////////
double       outMin   = 0.0, outMax     = 0.0; 
const double alfa = 0.7, valueConst1=104.71, R=60;   
double t = 0.0;


////////////////////// FILTROS ///////////////////////////////
double       outMin2   = 0.0, outMax2     = 0.0; 
const double alfa2 = 0.7, valueConst2 = 104.71, R2 = 60;   
double t2 = 0.0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(38400);
  
  pinMode(C1_1, INPUT); //entrada canal a
  pinMode(C2_1, INPUT);  //entrada canal b
  pinMode(in1, OUTPUT); //puente h derecho
  pinMode(in2, OUTPUT); //puente h izquierdo
  pinMode(enA, OUTPUT); //puente h izquierdo
  
  digitalWrite(in1, false);     //aseguramos motor en reposo inicial   
  digitalWrite(in2, false); 
  //digitalWrite(enA,100);

  pinMode(c1_2, INPUT);
  pinMode(c2_2, INPUT);
  pinMode(ind1,OUTPUT);
  pinMode(ind2,OUTPUT);
  pinMode(enA2,OUTPUT);

  digitalWrite(ind1,false);
  digitalWrite(ind2,false);
  
 //declaramos interrupciones
attachInterrupt(digitalPinToInterrupt(C1_1), encoder_1, CHANGE);
attachInterrupt(digitalPinToInterrupt(C2_1), encoder_1, CHANGE);

attachInterrupt(digitalPinToInterrupt(c1_2), encoder_2, CHANGE);
attachInterrupt(digitalPinToInterrupt(c2_2), encoder_2, CHANGE);

//asignamos tiempo de muestreo 0.1s

sampleTime = 100;                      // Se le asigna el tiempo de muestreo en milisegundos.
// limites de control PID

outMax =  10000;                      // Límite máximo del controlador PID.
outMin = -10000;                     // Límite mínimo del controlador PID.

outMax2 =  10000;                      
outMin2 = -10000;  

//ganancias del PID

  kp1 = 0.4;
  kd1 = 1.9;
  ki1 = 0.0099;

  kp2 = 0.4;
  kd2 = 1.9;
  ki2 = 0.0099;

WLd = 0.0;
WRd=0.0;
}

void loop() {
  if (stringComplete) 
  {
    for (int i = 0; i < dataLength ; i++)
    {
      int index = inputString.indexOf(separator);  //pasa dato por dato hasta llegar al separador
      data[i] = inputString.substring(0, index).toFloat(); //guarda todos los datos del index en data i
      inputString = inputString.substring(index + 1); //
  
    } //finaliza for
    
    WLd=data[0]; //guardamos valores recibidos 
    WLdd=data[1]; 
  
    WRd = data[2];
    WRdd = data [3];

    Serial.println(WLreal,3);
    Serial.println(WRreal,3);
    
    inputString = "";
    stringComplete = false;
    
  }//finaliza if
  
ComputePID();
ControlPID();
  
} //finaliza voidloop
  
double ComputePID(void){
  
if (millis() - lastTime >= sampleTime){  

noInterrupts(); // Desconectamos la interrupción para que no actué en esta parte del programa.

WLreal= (valueConst1*contador_1)/(millis()-lastTime);
lastTime = millis();
contador_1=0;

interrupts ();

//calcular errror de seguimiento
error1 =  (WLd - WLreal);
 //termino proporcional
Pterm1  = error1;

 //termino integral
ITerm1 += error1 * sampleTime; // compute integral
if(ITerm1 > 10000) ITerm1= 10000; else if (ITerm1 < -10000) ITerm1=-10000;//amti-windup

 //termino derivativo
DTerm1 = (error1-error1_ant)/(sampleTime); 
///controlador ¨PID

u1 =  kp1*Pterm1 + ki1*ITerm1 + kd1*DTerm1;


if (u1 > 0) anticlockwise(in1,in2,enA,abs(u1));else clockwise(in1,in2,enA,abs(u1));
 
}//finaliza if
Serial.print("WLd : ");Serial.print(WLd); Serial.print(",");
Serial.print("WLreal : ");Serial.print(WLreal); Serial.print(",");
Serial.println();
} //finaliza control pid

double ControlPID(void){
  
if (millis() - lastTime >= sampleTime){  

noInterrupts(); // Desconectamos la interrupción para que no actué en esta parte del programa.

WRreal= (valueConst2*contador_2)/(millis()-lastTime);
lastTime = millis();
contador_2=0;

interrupts ();

//calcular errror de seguimiento
error2 =  (WRd - WRreal);
 //termino proporcional
Pterm2  = error2;

 //termino integral
Iterm2 += error2 * sampleTime; // compute integral
if(Iterm2 > 10000) Iterm2= 10000; else if (Iterm2 < -10000) Iterm2 = -10000;//amti-windup

 //termino derivativo
Dterm2 = (error2-error2_ant)/(sampleTime); 
///controlador ¨PID

u2 =  kp2*Pterm2 + ki2*Iterm2 + kd2*Dterm2;


if (u2 > 0) anticlockwise2(ind1,ind2,enA2,abs(u2));else clockwise2(ind1,ind2,enA2,abs(u2));
 
}//finaliza if
Serial.print("WRd : ");Serial.print(WRd); Serial.print(",");
Serial.print("WRreal : ");Serial.print(WRreal); Serial.print(",");
Serial.println();
} //finaliza control pid

void encoder_1(void)
{
 ant_1=act_1;   

if(digitalRead(C1_1)==1) bitSet(act_1,0); else bitClear(act_1,0);
if(digitalRead(C2_1)==1) bitSet(act_1,1); else bitClear(act_1,1);

if(ant_1==3 && act_1==1) contador_1--; //Cuadratura x1
if(ant_1==1 && act_1==0) contador_1--;//Cuadratura x2
if(ant_1==0 && act_1==2) contador_1--; //Cuadratura x4
if(ant_1==2 && act_1==3) contador_1--;//Cuadratura x4

if(ant_1==1 && act_1==3) contador_1++; //Cuadratura x1
if(ant_1==0 && act_1==1) contador_1++;//Cuadratura x2
if(ant_1==2 && act_1==0) contador_1++; //Cuadratura x4
if(ant_1==3 && act_1==2) contador_1++;//Cuadratura x4
}

void encoder_2(void)
{
 ant_2=act_2;   

if(digitalRead(c1_2)==1) bitSet(act_2,0); else bitClear(act_2,0);
if(digitalRead(c2_2)==1) bitSet(act_2,1); else bitClear(act_2,1);

if(ant_2==3 && act_2==1) contador_2--; //Cuadratura x1
if(ant_2==1 && act_2==0) contador_2--;//Cuadratura x2
if(ant_2==0 && act_2==2) contador_2--; //Cuadratura x4
if(ant_2==2 && act_2==3) contador_2--;//Cuadratura x4

if(ant_2==1 && act_2==3) contador_2++; //Cuadratura x1
if(ant_2==0 && act_2==1) contador_2++;//Cuadratura x2
if(ant_2==2 && act_2==0) contador_2++; //Cuadratura x4
if(ant_2==3 && act_2==2) contador_2++;//Cuadratura x4
}


void clockwise (int pin1, int pin2, int analogPin, int pwm){
digitalWrite(pin1,LOW);
digitalWrite(pin2,HIGH);
analogWrite (analogPin, pwm);}

void clockwise2 (int pind1, int pind2, int analogPin, int pwm){
digitalWrite(pind1,LOW);
digitalWrite(pind2,HIGH);
analogWrite (analogPin, pwm);}

void anticlockwise (int pin1, int pin2, int analogPin, int pwm){
digitalWrite(pin1,HIGH);
digitalWrite(pin2,LOW);
analogWrite (analogPin, pwm);}

void anticlockwise2 (int pind1, int pind2, int analogPin, int pwm){
digitalWrite(pind1,HIGH);
digitalWrite(pind2,LOW);
analogWrite (analogPin, pwm);}


void serialEvent(){
      while(Serial.available()){
        char inChar = (char)Serial.read();
        inputString +=inChar;
        if(inChar =='\n'){
          stringComplete = true;
        }
      }
 }
