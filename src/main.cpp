#include <Arduino.h>

#include <cstring>

#include "datos_t.h"

using namespace std;

// PIDs: Kd,Kp,Ki,QPPS
/*
const float PID_M1[4] = {0.0, 0.31516, 0.04771, 84750.0};
const float PID_M2[4] = {0.0, 0.30119, 0.04643, 82125.0};
const float PID_M3[4] = {0.0, 0.20926, 0.03613, 78000.0};
const float PID_M4[4] = {0.0, 0.20380, 0.03661, 79875.0};
*/

//String cadena;

// On Roboclaw set switch 1 and 6 on.

// declaracion puertos serie roboblaws

#define RXD1 26
#define TXD1 27

#define RXD2 16
#define TXD2 17

#define BAUDRATE 38400

#define address 0x80
// declaracion robovlaws

#define acceleration 1.0f // tiempo de acerlacion en segudnos
#define mul_speed 20.0f
#define mul_speed_giro 20.0f

#define VMin 10
#define REDUCCION 40.0f

#define RADIO 0.076f
#define Length 0.68181f
#define Width 0.68181f

datos_t Dato(acceleration, mul_speed, mul_speed_giro);

int pos = 0; // variable to store the servo position

// paquete de mensaje

// segundo nucleo
void segundoNucleo(void * pV);

// Interrupción ante evento del mando
void notify()
{
  Dato.modoManual();
}

void onConnect()
{
  // si se conecta al mando, se para
  Dato.pararTodo();
}

void OnDisconnect()
{
  // si se desconecta al mando, se para
  Dato.pararTodo();
}

void setup()
{
  Dato.controlador->attach(notify);
  Dato.controlador->attachOnConnect(onConnect);
  Dato.controlador->attachOnDisconnect(OnDisconnect);

  Serial.begin(BAUDRATE);

  Dato.begin(TXD1, RXD1, TXD2, RXD2);

  xTaskCreate(segundoNucleo, "mi_tarea", 10000, NULL, 1, NULL);

  // Serial.println("Ready.");
}

void segundoNucleo( void *pV)
{
  for(;;)
  {
    Dato.actualizarVelocidad();
  }
  
}

void loop()
{
  // Sale del bucle si no haysegundoNucleo un mando conectado
  if (!Dato.controlador->isConnected())
  {
    Dato.recibirMensaje();
  }

 

}

