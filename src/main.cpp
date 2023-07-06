#include <Arduino.h>

#include <Ps3Controller.h>
#include <RoboClaw.h>

#include <cstring>

int player = 0;
int battery = 0;

int parar = 0;
int habilitado = 0;

int LeftX = 0, LeftY = 0, RightX = 0, RightY = 0;
//definicion velocidades 
int VM1 = 0, VM2 = 0, VM3 = 0, VM4 = 0;
int VM1_i = 0, VM2_i = 0, VM3_i = 0, VM4_i = 0;

//PIDs: Kd,Kp,Ki,QPPS
const float PID_M1[4] = {0.0,0.31516,0.04771,84750.0};
const float PID_M2[4] = {0.0,0.30119,0.04643,82125.0};
const float PID_M3[4] = {0.0,0.20926,0.03613,78000.0};
const float PID_M4[4] = {0.0,0.20380,0.03661,79875.0};

String cadena;

//On Roboclaw set switch 1 and 6 on.

//declaracion puertos serie roboblaws

#define RXD1 26
#define TXD1 27

#define RXD2 16
#define TXD2 17

#define BAUDRATE 38400

#define address 0x80
//declaracion robovlaws
 
RoboClaw roboclaw_IZQUIERDO(&Serial1,10000);
RoboClaw roboclaw_DERECHO(&Serial2,10000);

#define acceleration 20000
#define mul_speed 250

const int VMin = 10;
 
int pos = 0;    // variable to store the servo position 


//Interrupción ante evento del mando
void notify()
{

  if(habilitado == 1){

    if(Ps3.event.button_down.ps){
    Ps3.setPlayer(1);
    //parada robot
    VM1 = 0;
    VM2 = 0;

    VM3 = 0;
    VM4 = 0;

    habilitado = 0;
    }

  //parada de motores
    
    if( Ps3.event.button_down.cross )
    {
      VM1 = 0;
      VM2 = 0;
    
      VM3 = 0;
      VM4 = 0;
    }

    //obtención velocidades
    
    LeftX = Ps3.data.analog.stick.lx;
    LeftY = Ps3.data.analog.stick.ly;

    RightX = Ps3.data.analog.stick.rx;
    RightY = Ps3.data.analog.stick.ry;

  }
  else{

    if(Ps3.event.button_down.ps){
    Ps3.setPlayer(5);
    habilitado = 1;
    }

  }
}

void onConnect()
{
    
}

void OnDisconnect()
{
  
}
 
void setup() 
{ 
  Serial.begin(BAUDRATE);
  Serial2.begin(BAUDRATE, SERIAL_8N1, RXD2,TXD2);
  Serial1.begin(BAUDRATE, SERIAL_8N1, RXD1,TXD1);

  Ps3.attach(notify);
  Ps3.attachOnConnect(onConnect);
  Ps3.attachOnDisconnect(OnDisconnect);
  Ps3.begin(/*"01:02:03:04:05:06"*/);
/*
  roboclaw_IZQUIERDO.SetM1VelocityPID(address,PID_M1[0],PID_M1[1],PID_M1[2],PID_M1[3]);
  roboclaw_IZQUIERDO.SetM2VelocityPID(address,PID_M2[0],PID_M2[1],PID_M2[2],PID_M2[3]);  

  roboclaw_DERECHO.SetM2VelocityPID(address,PID_M3[0],PID_M3[1],PID_M3[2],PID_M3[3]);  
  roboclaw_DERECHO.SetM2VelocityPID(address,PID_M4[0],PID_M4[1],PID_M4[2],PID_M4[3]);  
*/
  Serial.println("Ready.");


} 
 
void loop() 
{ 
   if(!Ps3.isConnected())
      return;
      
        
    
    
    //paro
    if(abs(LeftX) < VMin & abs(LeftY) < VMin & abs(RightX) < VMin & abs(RightY) < VMin )
    {
      VM1 = 0;
      VM2 = 0;

      VM3 = 0;
      VM4 = 0;
    }
    else{
      VM1 = (-1) * mul_speed * (LeftY - RightX - LeftX + RightY);
      VM2 = (-1) * mul_speed * (LeftY - RightX + LeftX + RightY);

      VM3 = (-1) * mul_speed * (LeftY + RightX - LeftX + RightY);
      VM4 = (-1) * mul_speed * (LeftY + RightX + LeftX + RightY);

    }

    //velocidades
    roboclaw_IZQUIERDO.SpeedAccelM1(address,acceleration,VM1);
    roboclaw_IZQUIERDO.SpeedAccelM2(address,acceleration,VM2);

    roboclaw_DERECHO.SpeedAccelM1(address,acceleration,VM3);
    roboclaw_DERECHO.SpeedAccelM2(address,acceleration,VM4);

      

    //Imprime valores joysticks
    cadena = String(LeftX) + ',' + String(LeftY) + ',' + String(RightX) + ',' + String(RightY) + '\n';
    Serial.print(cadena);

  

} 
