/*************************************************************
Ventilador V3

 *************************************************************
 v0 volumen             pines virtuales para blynk app
 v1 presion
 v2 temperatura
 v3 oxigeno
 v4 led 
 v5 temp
 v6 aire      
                        
/
/* Inicio programa */
//****************************************************************************************
#define BLYNK_PRINT Serial                  //lib Blynk

#include <Blynk.h>                          //lib Blynk
#include <SoftwareSerial.h>                 //lib sensor
#include <BlynkSimpleSerialBLE.h>           //lib Blynk
#include <DHT.h>                            //lib sensor

#define DHTPIN 6                            // pin digital donde el sensor de temp está conectado 
#define DHTTYPE DHT11                       // tipo del sensor

SoftwareSerial SerialBT1(10, 11); // establece la comuicación Bluetooth por pines RX, TX

BlynkTimer timerEnvio;           // Timer que controla el proceso envio datos Led y Dth
BlynkTimer timerCiclo;           // Timer para control del tiempo de cada ciclo de respiración

DHT dht(DHTPIN, DHTTYPE);

// You should get Auth Token in the Blynk App.
char auth[] = "aqui va el token que generaste desde la app Blynk y te envían a tu correo";



//Leds ---------------------- const refiere variable de solo lectura, valor fijo
const int Ld1 = 2;        // comunicacion establecida visualmente
const int Ld2 = 4;        // valvula de inhalacion activa
const int Ld3 = 5;        // valvula de inhalacion inactiva
const int Ld4 = 12;       // valvula de exhalacion activa
const int Ld5 = 13;       // valvula de exhalacion inactiva

//Pines motor
int IN3 = 7;          //Giro del motor
int IN4 = 8;          //Giro del motor
int ENB = 9;          //control de Pwm del motor 
int Vol = 0;          //par de constantes tipo entero 
int Presion = 0;

long varExala = 2000;     // variable para control de timer default intervalo de 1 segundo, tiempo de exhalación 
long varInala = 1000;     // variable para control del tiempo de inhalacion 
long CicloResp = 0;       // variable para control del ciclo de respiracion 
int varTimer;             // control Timeout para el timerCiclo

//Pin Valvula oxigeno
const int Voxg = 3;        // pin valvula de oxigeno 
int POxg;                  // declara variable Porcentaje de oxigeno


//Pines virtuales         para uso de la app blink
WidgetLED led1(V4);



//*****************************************************************************
//***************************************SETUP        *************************/


void setup()
{
   
  Serial.begin(9600);                               //  Inicia puerto serie a 9600
  Serial.println("Waiting for connections...");     //  Despliega mensaje en monitor serie
  
  SerialBT1.begin(9600);                            // Inicia segunda comunicación serie a 9600  
  Blynk.begin(SerialBT1, auth);                     // Inicia la comunicación con la app


                            // Declara pines del microprocesador como salidas 
  pinMode(Voxg, OUTPUT);    // intencidad de valvula oxigeno 
  pinMode(IN3, OUTPUT);     // entrada motor 1
  pinMode(IN4, OUTPUT);     // entrada motor 2
  pinMode(ENB, OUTPUT);     // ENB control de vel
  
  pinMode(Ld1, OUTPUT);     //
  pinMode(Ld2, OUTPUT);     //
  pinMode(Ld3, OUTPUT);     //
  pinMode(Ld4, OUTPUT);     //
  pinMode(Ld5, OUTPUT);     //
  pinMode(Voxg, OUTPUT);     //

  
  timerEnvio.setInterval (300L, controlLeds); // inicia timer para lectura de leds 300L = .3 seg, 1000L = 1 seg

  dht.begin(); // inicia sensor de temperatura y humedad 

  digitalWrite (IN3, HIGH);   //Establece el sentido del giro del motor 
  digitalWrite (IN4, LOW);    //Como la turbina solo gira en un sentido se puede conectar directamente a 
                              //+5 v y comun cada pin respectivamente desde la salida del puente l298N
  
}

//****************************************************************/

// Subrutinas 

//****************************************************************/
//****************************************************************/
// Funcion led on-off

void controlLeds()
{
  
  // solo funciona con arquitectura AVR
    // Control de estados del led en el celular, envio de señal cada intervalo definido por timerLEDDTH 
  if (digitalRead(Ld1) == HIGH){      // si la lectura del puerto virtual esta en alto 
    led1.on();                        // el led en el puerto v4 enciende 
  }
  else {                              // de lo contrario
    led1.off();                       // el led en el puerto v4 estara apagado
  }
  
  // Sensor de temperatura


  float h = dht.readHumidity();         // almacena en variable h el valor leido del sensor para humedad por el pin 6
  float t = dht.readTemperature();      // almacena en variable t el valor leido del sensar para temperatura por el pin 6

  if (isnan(h) || isnan(t)){                          // Si el valor de h no es un numero ó el valor de t no es un numero
    SerialBT1.println("Error al leer el sensor");     // Escribe este mensaje en el monitor serie
    return;                                           // regresa al if 
  }
  Blynk.virtualWrite(V6,h);                           // envia el valor de h al pin virtual V6 de la app
  Blynk.virtualWrite(V5,t);                           // envia el valor de t al pin virtual V5 de la app 
  
}



//****************************************************************/
// Funcion control velvula oxigeno  -  ESTE VALOR CON RANGO DE 0-100%  NO DEBE SER MAYOR A 60% CONFIRMAR  

BLYNK_WRITE(V3)
{
  // El valor cambia cada vez que algo cambia en la app del android 
  POxg = param.asInt();                // Esto recibe el valor del widget V3 como un numero entero y lo almacena en POxg (porcentaje oxg)
  analogWrite(Voxg, POxg);            // Envia señal de PWM basada en el valor recibido por V3
}


//****************************************************************/
// Funcion control presión de motor
BLYNK_WRITE(V1)
{
  // El valor cambia cada vez que algo cambia en la app del android 
  Presion = param.asInt();        // Esto recibe el valor del widget V1 como un numero entero y lo almacena en var Presion 
//  analogWrite(ENB,Presion);         // asigna valor de volumen a variable vol 
}



//****************************************************************/

// Funcion control tiempo del motor

// Tiempo inalación //****************************************************************/



BLYNK_WRITE(V2) 

{  // El valor cambia cada vez que algo cambia en la app del android 
  varInala = param.asInt();       // Esto recibe el valor del widget V2 como un numero entero y lo almacena en var varInhala
                                  // Control de tiempo minimo de inhalación en este caso cero MODIFICAR CUANDO SE INVESTIGE 
  if (varInala <= 0) {            // Si el valor de Inhalación es menor o igual que cero
    varInala = 1;                 // el valor de la variable varInhala será 1 
  }
}

// Tiempo de exalación y tiempo total del retardo //****************************************************************/



BLYNK_WRITE(V0) 

{ // El valor cambia cada vez que algo cambia en la app del android 
  varExala = param.asInt();      // Esto recibe el valor del widget V0 como un numero entero y lo almacena en var varExala
                                 // Control de tiempo minimo de exhalación en este caso cero MODIFICAR CUANDO SE INVESTIGE 
  if (varExala <= 0) {           // Si el valor de exhalación es menor o igual que cero
    varExala = 1;                // el valor de la variable varExala será 1 
  }

  CicloResp = varInala + varExala;

  timerCiclo.deleteTimer(varTimer);  // Cancela el valor anterior del timer 
  varTimer = timerCiclo.setInterval(CicloResp, timerLoop);  // ajusta el Timeout del timer y ejecuta timerLoop cada unidad de T
}



// Subrutina timerloop **************************************************************************************************

void timerLoop() {

  //  secuencia inhalación
  
  analogWrite(ENB,Presion);    // control de velocidad del motor por PWM destino pin ENB

  digitalWrite (Ld2, LOW);     // valvula inhalacion
  digitalWrite (Ld3, HIGH);    // activa
  
  digitalWrite (Ld4, HIGH);    // valvula exalacion
  digitalWrite (Ld5, LOW);     // inactiva

  delay (varInala);            // Espera el tiempo de inhalación  

  analogWrite(ENB,0);          // Detiene el motor conectado el pin ENB

  digitalWrite (Ld2, HIGH);     // valvula inhalacion
  digitalWrite (Ld3, LOW);      // inactiva
  
  digitalWrite (Ld4, LOW);      // valvula exalacion
  digitalWrite (Ld5, HIGH);     // activa


}


// **************************************************************************************************
// Loop del programa

void loop()
{
  Blynk.run();
  timerEnvio.run();
  timerCiclo.run();


}
