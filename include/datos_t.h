#define LINKEDSTACKTYPE_H
#ifndef LINKEDSTACKTYPE_H
#endif

#include <stdint.h>
#include <RoboClaw.h>
#include <Ps3Controller.h>
#include <Arduino.h>
#include "spslib.h"

#include "Mando_t.h"
#include "Velocidad_t.h"

using namespace SPS;
using namespace std;

typedef Message<> Mensaje;

class datos_t
{

    int baudRate;

    uint8_t address1, address2;

    HardwareSerial *HS1, *HS2, *HS0;

    float tiempo;
    float mul_speed;
    float mul_speed_giro;
    int VMin;

    float RADIO;
    float Length;
    float Width;

    float reduccion;
    float Velocidad_Ojetivo[3] = {0.0f, 0.0f, 0.0f};
    float Vel_Ang[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float Accel_Ang[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    Velocidad_t Velocidad_Angular, Velocidad_Angular_Anterior;
    Velocidad_t Aceleracion;
    Mando_t Mando;

    Mensaje Vel_Mess;
    Mensaje Vel_Obj;
    int velocidadesMotores[4] = {0, 0, 0, 0};
    float miVelocidad[3] = {0.0f, 0.0f, 0.0f};
    
    Mensaje::MsgReader reader;

    bool calcularAceleracion();
    bool calcularVelocidad();

    bool clavar();

public:
    RoboClaw roboclaw_IZQUERDO, roboclaw_DERECHO;

    Ps3Controller *controlador;

    datos_t(float tmp, float mul_s, float mul_sg, int vmin = 10, float reducir = 40.0f, float L = 0.68181f, float W = 0.68181f, float R = 0.076f, Ps3Controller *ctrl = &Ps3, int br = 38400, uint8_t a1 = 0x80, uint8_t a2 = 0x80, HardwareSerial *hs1 = &Serial1, HardwareSerial *hs2 = &Serial2, HardwareSerial *hs0 = &Serial) : tiempo(tmp), mul_speed(mul_s), mul_speed_giro(mul_sg), VMin(vmin), reduccion(reducir), Length(L), Width(W), RADIO(R), controlador(ctrl), baudRate(br), address1(a1), address2(a2), HS1(hs1), HS2(hs2), HS0(hs0), roboclaw_IZQUERDO(hs1, 1000), roboclaw_DERECHO(hs2, 1000), Vel_Mess(10), Vel_Obj(11)
    {
    }

    void begin(int TX1, int RX1, int TX2, int RX2);
    void begin();

    bool actualizarVelocidad(int vm1, int vm2, int vm3, int vm4);
    bool actualizarVelocidad(float VX, float VY, float WZ);
    bool actualizarVelocidad(const Velocidad_t &V);

    bool eviarVelocidadActual(float *VX, float *VY, float *WZ);

    bool clavar(int vm1, int vm2, int vm3, int vm4);
    bool clavar(const Velocidad_t &V);

    Velocidad_t obtenerVelocidad();

    bool pararTodo();

    bool modoManual();

    bool enviarVelocidad();
    bool enviarVelocidad(const Velocidad_t &V);
    bool enviarVelocidad_Objetivo();
    bool enviarVelocidad_Objetivo(float VX, float VY, float WZ);
    bool recibirMensaje();

    bool moverMotores();
};
