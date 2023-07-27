#include <stdint.h>
#include <RoboClaw.h>
#include <Ps3Controller.h>
#include <Arduino.h>

#include "Mando_t.h"
#include "Velocidad_t.h"


class datos_t
{

    int baudRate;

    uint8_t address1, address2;

    HardwareSerial *HS1, *HS2;

    int acceleration;
    int mul_speed;
    int VMin;

    Velocidad_t Velocidad;
    Mando_t Mando;

    public:
    

    RoboClaw roboclaw_IZQUERDO, roboclaw_DERECHO;

    Ps3Controller* controlador;

    

    datos_t(int accel, int mul_s, int vmin, Ps3Controller * ctrl = &Ps3, int br = 38400, uint8_t a1 = 0x80, uint8_t a2 = 0x80, HardwareSerial * hs1 = &Serial1, HardwareSerial * hs2 = &Serial2) : acceleration(accel), mul_speed(mul_s),VMin(vmin), controlador(ctrl), baudRate(br), address1(a1), address2(a2),HS1(hs1), HS2(hs2), roboclaw_IZQUERDO(hs1,1000), roboclaw_DERECHO(hs2,1000)
    {

    }

    void begin(int TX1, int RX1, int TX2, int RX2);
    void begin();

    bool actualizarVelocidad();
    bool actualizarVelocidad(int vm1, int vm2, int vm3, int vm4);
    bool actualizarVelocidad(const Velocidad_t& V);

    bool clavar();
    bool clavar(int vm1, int vm2, int vm3, int vm4);

    Velocidad_t& obtenerVelocidad();

    void pararTodo();

    void modoManual();

};
