#include <Arduino.h>

//#include "../include/Motores.cpp"
//#include "../include/Sen_Linea.cpp"
//#include "../include/Sen_Oponente.cpp"
//

#define SWEEP_SPEED 50                        // Cambio de velocidad
#define EQUIVALENT_12V 100                    // 12 V
#define EQUIVALENT_24V 255                    // 24 V
#define INTERRUPTION_DELAY_MS 500             // Delay para interrupción
#define MOTOR_IMPULSE_DELAY 60                // Delay motor
#define SENSOR_DISTANCE_CM 40                 // Distancia del sensor a comparar
#define ULTRASONIC_SENSOR_TIMEOUT 8000        //

bool antiBounce = false;

const byte pinRF = 0;            // D0 - Control RF

bool enableSignalReceived = false;
bool targetFound = false;
bool isRobotOn = false;

const byte pinSensorDD = 11;      // B3 - Sensor Delantero
const byte pinSensorDI = 8;       // B0 - Sensor Delantero
const byte pinSensorLD = 12;      // B4 - Sensor Lateral
const byte pinSensorLI = 13;      // B5 - Sensor lateral  

//-------------------------------------------------------------------------------//

const byte pinMDG1 = A5/*5*/;       // Entrada C5 ATMega328p - Giro motor 
const byte pinMDG2 = A4/*6*/;       // Entrada C4 ATMega328p - Giro motor
const byte pinMDE = 10/*9*/;        // B2 - Enable 1

const byte pinMIG1 = 5/*A5*/;       // D5 - Giro motor
const byte pinMIG2 = 6/*A4*/;       // D6 - Giro motor
const byte pinMIE = 9/*10*/;        // B1 - Enable 2

//-------------------------------------------------------------------------------//

const byte pinLineaDD = 3;        // D1 - Sensor de linea 
const byte pinLineaDI = 2;        // D2 - Sensor de linea 
const byte pinLineaTD = 1;        // D3 - Sensor de linea 
const byte pinLineaTI = 4;        // D4 - Sensor de linea 

const int Vb = 50;
const int Vf = 140;
const int Vg = 100;
const int Vga = 110;
const int Va = 150; 

const float tiempoFrenado = 400; //Tiempo de frenado
const float tiempoGiroAtaque = 200; //Tiempo giro ataque
const int tiempoLimite = 300; //tiempo Giro
unsigned long millisViejo = 0;


/*
 * ------------------------------------------
 *                FUNCIONES
 *-------------------------------------------
 */

void pinSensorLin(){
  pinMode(pinLineaDD, INPUT);
  pinMode(pinLineaDI, INPUT);
  pinMode(pinLineaTD, INPUT);
  pinMode(pinLineaTI, INPUT);
}

void pinSensorOp(){                        // Inicialización de los pines 
  pinMode(pinSensorDD, INPUT);
  pinMode(pinSensorDI, INPUT);
  pinMode(pinSensorLD, INPUT);
  pinMode(pinSensorLI, INPUT);
}

void pinesMotor(){
  pinMode(pinMDG1, OUTPUT);
  pinMode(pinMDG2, OUTPUT);
  pinMode(pinMIG1, OUTPUT);
  pinMode(pinMIG2, OUTPUT);
  pinMode(pinMDE, OUTPUT);
  pinMode(pinMIE, OUTPUT);
}

/*
 * ------------------------------------------
 *           ACCIONES DE MOTORES
 *-------------------------------------------
 */

void pararMotores() {
  digitalWrite(pinMDG1, LOW);
  digitalWrite(pinMDG2, LOW);
  analogWrite(pinMDE, 0);
  digitalWrite(pinMIG1, LOW);
  digitalWrite(pinMIG2, LOW);
  analogWrite(pinMIE, 0);
}

void atrasDerecha(int velocidadDerecho) {
  digitalWrite(pinMDG1, HIGH);
  digitalWrite(pinMDG2, LOW);
  analogWrite(pinMDE, velocidadDerecho);
}

void atrasIzquierda(int velocidadIzquierdo) {
  digitalWrite(pinMIG1, HIGH);
  digitalWrite(pinMIG2, LOW);
  analogWrite(pinMIE, velocidadIzquierdo);
}

void adelanteDerecha(int velocidadDerecho) {
  digitalWrite(pinMDG1, LOW);
  digitalWrite(pinMDG2, HIGH);
  analogWrite(pinMDE, velocidadDerecho);
}

void adelanteIzquierda(int velocidadIzquierdo) {
  digitalWrite(pinMIG1, LOW);
  digitalWrite(pinMIG2, HIGH);
  analogWrite(pinMIE, velocidadIzquierdo);
}

/*
 * ------------------------------------------
 *              DECLARACIONES 
 *-------------------------------------------
 */

void setup() {
  pinSensorLin();            // Pines Linea 
  pinSensorOp();             // Pines Oponente     
  pinesMotor();              // Pines Motores
}

/*
 * ------------------------------------------
 *                  LOOP 
 *-------------------------------------------
 */

void loop() {
  if(millis()>5000){
    if (digitalRead(pinLineaDD) == LOW  && digitalRead(pinLineaDI) == LOW) 
    {
      //Serial.println("Giro Ambos"); 
      millisViejo=millis();
      const float tiempoFrenado2 = 700;
      while((millis()-millisViejo)<tiempoFrenado2){atrasDerecha(Vf); atrasIzquierda(Vf);}//vamos para atras 
      millisViejo=millis();
      while((millis()-millisViejo)<tiempoLimite){
      adelanteDerecha(Vg); atrasIzquierda(Vg);
      //while(SensorOponente()==1){Ataque();}
      }
    }
    if (digitalRead(pinLineaDD) == LOW)
    {
      millisViejo=millis();
      while((millis()-millisViejo)<tiempoFrenado){atrasDerecha(Vf); atrasIzquierda(Vf);}//vamos para atras 
      millisViejo=millis();
      while((millis()-millisViejo)<tiempoLimite){
      adelanteDerecha(Vg); atrasIzquierda(Vg);
      //while(SensorOponente()==1){Ataque();}
      }
    } 
    else if (digitalRead(pinLineaDI) == LOW)
    {
      millisViejo=millis();
      while((millis()-millisViejo)<tiempoFrenado){atrasDerecha(Vf); atrasIzquierda(Vf);}//vamos para atras 
      millisViejo=millis();
      while((millis()-millisViejo)<tiempoLimite){
      atrasDerecha(Vg); adelanteIzquierda(Vg);
      //while(SensorOponente()==1){Ataque();}
      }
    }
    else{
      if(digitalRead(pinSensorDD) == LOW || digitalRead(pinSensorDI) == LOW){
        adelanteIzquierda(Va);adelanteDerecha(Va); 
      }
      else if(digitalRead(pinSensorLD) == LOW){                                             //Giro derecho de Ataque
        millisViejo = millis();
        while((millis()-millisViejo)<tiempoGiroAtaque){
          atrasIzquierda(Vga);adelanteDerecha(Vga);
        }
      }
      else if(digitalRead(pinSensorLI) == LOW){                                             //Giro izquierdo de Ataque
        millisViejo = millis();
        while((millis()-millisViejo)<tiempoGiroAtaque){
          adelanteIzquierda(Vga); atrasDerecha(Vga); 
        }
      }
      else{//adelanteIzquierda(0);adelanteDerecha(0);
        //pararMotores();
        if(millis()<=5500){
            adelanteIzquierda(70);adelanteDerecha(70);
        }
        adelanteIzquierda(Vb);adelanteDerecha(Vb);
      }  
    }
  }
}