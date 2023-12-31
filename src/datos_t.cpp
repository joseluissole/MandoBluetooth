#include "datos_t.h"
#include <math.h>

bool datos_t::calcularAceleracion()
{
    // leo la velocidad actual en el motor
    Velocidad_Angular_Anterior.VM1 = roboclaw_DERECHO.ReadSpeedM1(address2);
    Velocidad_Angular_Anterior.VM2 = roboclaw_IZQUERDO.ReadSpeedM2(address1);
    Velocidad_Angular_Anterior.VM3 = roboclaw_IZQUERDO.ReadSpeedM1(address1);
    Velocidad_Angular_Anterior.VM4 = roboclaw_DERECHO.ReadSpeedM2(address2);

    // calcula la aceleración que debe tener
    Accel_Ang[0] = (float)abs(Velocidad_Angular.VM1 - Velocidad_Angular_Anterior.VM1) / tiempo;
    Accel_Ang[1] = (float)abs(Velocidad_Angular.VM2 - Velocidad_Angular_Anterior.VM2) / tiempo;
    Accel_Ang[2] = (float)abs(Velocidad_Angular.VM3 - Velocidad_Angular_Anterior.VM3) / tiempo;
    Accel_Ang[3] = (float)abs(Velocidad_Angular.VM4 - Velocidad_Angular_Anterior.VM4) / tiempo;

    Aceleracion.VM1 = trunc(Accel_Ang[0]);
    Aceleracion.VM2 = trunc(Accel_Ang[1]);
    Aceleracion.VM3 = trunc(Accel_Ang[2]);
    Aceleracion.VM4 = trunc(Accel_Ang[3]);

    return true;
}

bool datos_t::calcularVelocidad()
{
    // Velocidades angulares en pusos/s de 90 grados en el motor
    Vel_Ang[0] = (0.0f - Velocidad_Ojetivo[0] + Velocidad_Ojetivo[1] + 0.5f * (Length + Width) * Velocidad_Ojetivo[2]) * reduccion * 2 / PI;
    Vel_Ang[1] = (Velocidad_Ojetivo[0] + Velocidad_Ojetivo[1] - 0.5f * (Length + Width) * Velocidad_Ojetivo[2]) * reduccion * 2 / PI;
    Vel_Ang[2] = (0.0f - Velocidad_Ojetivo[0] + Velocidad_Ojetivo[1] - 0.5f * (Length + Width) * Velocidad_Ojetivo[2]) * reduccion * 2 / PI;
    Vel_Ang[3] = (Velocidad_Ojetivo[0] + Velocidad_Ojetivo[1] + 0.5f * (Length + Width) * Velocidad_Ojetivo[2]) * reduccion * 2 / PI;

    Velocidad_Angular.VM1 = trunc(Vel_Ang[0]);
    Velocidad_Angular.VM2 = trunc(Vel_Ang[1]);
    Velocidad_Angular.VM3 = trunc(Vel_Ang[2]);
    Velocidad_Angular.VM4 = trunc(Vel_Ang[3]);

    return calcularAceleracion();
}

void datos_t::begin(int TX1, int RX1, int TX2, int RX2)
{

    HS1->begin(baudRate, SERIAL_8N1, RX1, TX1);
    HS2->begin(baudRate, SERIAL_8N1, RX2, TX2);

    controlador->begin();
}

void datos_t::begin()
{

    HS1->begin(baudRate);
    HS2->begin(baudRate);

    HS0->begin(baudRate);

    controlador->begin();
}

bool datos_t::pararTodo()
{

    Velocidad_Angular.reset();
    Aceleracion.reset();
    
    Velocidad_Ojetivo[0] = 0.0f;
    Velocidad_Ojetivo[1] = 0.0f;
    Velocidad_Ojetivo[2] = 0.0f;
    return roboclaw_IZQUERDO.SpeedM1M2(address1, 0, 0) && roboclaw_DERECHO.SpeedM1M2(address2, 0, 0);
}

bool datos_t::actualizarVelocidad(int vm1, int vm2, int vm3, int vm4)
{

    // Velocidades angulares en pusos/s de 90 grados
    Velocidad_Angular.VM1 = vm1;
    Velocidad_Angular.VM2 = vm2;
    Velocidad_Angular.VM3 = vm3;
    Velocidad_Angular.VM4 = vm4;

    return calcularAceleracion();
}

bool datos_t::actualizarVelocidad(const Velocidad_t &V)
{

    Velocidad_Angular = V;

    return calcularAceleracion();
}

bool datos_t::eviarVelocidadActual(float *VX, float *VY, float *WZ)
{
    Velocidad_Angular_Anterior.VM1 = roboclaw_DERECHO.ReadSpeedM1(address2);
    Velocidad_Angular_Anterior.VM2 = roboclaw_IZQUERDO.ReadSpeedM2(address1);
    Velocidad_Angular_Anterior.VM3 = roboclaw_IZQUERDO.ReadSpeedM1(address1);
    Velocidad_Angular_Anterior.VM4 = roboclaw_DERECHO.ReadSpeedM2(address2);

    *VX = (0 - Velocidad_Angular_Anterior.VM1 + Velocidad_Angular_Anterior.VM2 - Velocidad_Angular_Anterior.VM3 + Velocidad_Angular_Anterior.VM4) * (PI * RADIO) / (8 * reduccion);
    *VY = (Velocidad_Angular_Anterior.VM1 + Velocidad_Angular_Anterior.VM2 + Velocidad_Angular_Anterior.VM3 + Velocidad_Angular_Anterior.VM4) * (PI * RADIO) / (8 * reduccion);
    *WZ = (Velocidad_Angular_Anterior.VM1 - Velocidad_Angular_Anterior.VM2 - Velocidad_Angular_Anterior.VM3 + Velocidad_Angular_Anterior.VM4) * (PI * RADIO) / (4 * reduccion * (Length + Width));

    return true;
}

bool datos_t::actualizarVelocidad(float VX, float VY, float WZ)
{
    Velocidad_Ojetivo[1] = VY;
    Velocidad_Ojetivo[0] = VX;
    Velocidad_Ojetivo[2] = WZ;

    return calcularVelocidad();
}

bool datos_t::clavar()
{
    return roboclaw_IZQUERDO.SpeedM1M2(address1, Velocidad_Angular.VM1, Velocidad_Angular.VM2) && roboclaw_DERECHO.SpeedM1M2(address2, Velocidad_Angular.VM3, Velocidad_Angular.VM4);
}

bool datos_t::clavar(int vm1, int vm2, int vm3, int vm4)
{
    Velocidad_Angular.VM1 = vm1;
    Velocidad_Angular.VM2 = vm2;
    Velocidad_Angular.VM3 = vm3;
    Velocidad_Angular.VM4 = vm4;

    return clavar();
}

bool datos_t::clavar(const Velocidad_t &V)
{
    Velocidad_Angular = V;

    return clavar();
}

Velocidad_t datos_t::obtenerVelocidad()
{
    return Velocidad_Angular;
}

bool datos_t::modoManual()
{
    

    // clavada instantanea
    if (controlador->data.button.cross)
    {
        return pararTodo();
    }
    else if (controlador->data.button.ps)
    {
        pararTodo();
        ESP.restart();
        return true;
    }
    else
    {
        // paro
        if (abs(controlador->data.analog.stick.lx) < VMin & abs(controlador->data.analog.stick.ly) < VMin & abs(controlador->data.analog.stick.rx) < VMin & abs(controlador->data.analog.stick.ry) < VMin)
        {
            Mando.LeftX = 0;
            Mando.LeftY = 0;

            Mando.RightX = 0;
            Mando.RightY = 0;
            return pararTodo();
        }
        else
        {
            Velocidad_Ojetivo[0] = (float)Mando.LeftX * mul_speed;
            Velocidad_Ojetivo[1] = (-1.0f) * (float)Mando.LeftY * mul_speed;
            Velocidad_Ojetivo[2] = (-1.0f) * (float)Mando.RightX * mul_speed_giro;

            return calcularVelocidad();
        }
    }
}

bool datos_t::recibirMando()
{
    Mando.LeftX = controlador->data.analog.stick.lx;
    Mando.LeftY = controlador->data.analog.stick.ly;

    Mando.RightX = controlador->data.analog.stick.rx;
    Mando.RightY = controlador->data.analog.stick.ry;
    return true;
}

bool datos_t::moverMotores()
{
    return roboclaw_IZQUERDO.SpeedAccelM1M2_2(address1, Aceleracion.VM3, Velocidad_Angular.VM3, Aceleracion.VM2, Velocidad_Angular.VM2) &&
           roboclaw_DERECHO.SpeedAccelM1M2_2(address2, Aceleracion.VM1, Velocidad_Angular.VM1, Aceleracion.VM4, Velocidad_Angular.VM4);
}

bool datos_t::enviarVelocidad()
{
    velocidadesMotores[0] = roboclaw_DERECHO.ReadSpeedM1(address2);
    velocidadesMotores[1] = roboclaw_IZQUERDO.ReadSpeedM2(address1);
    velocidadesMotores[2] = roboclaw_IZQUERDO.ReadSpeedM1(address1);
    velocidadesMotores[3] = roboclaw_DERECHO.ReadSpeedM2(address2);

    Vel_Mess.write_array<int>(velocidadesMotores, 4);

    for (int i = 0; i < Vel_Mess.datagram_size(); i++)
        HS0->write(Vel_Mess[i]);

    return true;
}

bool datos_t::enviarVelocidad(const Velocidad_t &V)
{

    velocidadesMotores[0] = V.VM1;
    velocidadesMotores[1] = V.VM2;
    velocidadesMotores[2] = V.VM3;
    velocidadesMotores[3] = V.VM4;

    Vel_Mess.write_array<int>(velocidadesMotores, 4);

    for (int i = 0; i < Vel_Mess.datagram_size(); i++)
        HS0->write(Vel_Mess[i]);

    return true;
}

bool datos_t::enviarVelocidad_Objetivo()
{
    eviarVelocidadActual(&miVelocidad[0],&miVelocidad[1],&miVelocidad[2]);

    Vel_Obj.write_array<float>(miVelocidad, 3);

    for (int i = 0; i < Vel_Obj.datagram_size(); i++)
        HS0->write(Vel_Obj[i]);

    return true;
}

bool datos_t::enviarVelocidad_Objetivo(float VX, float VY, float WZ)
{
    miVelocidad[0] = VX;
    miVelocidad[1] = VY;
    miVelocidad[2] = WZ;

    Vel_Obj.write_array<float>(miVelocidad, 3);

    for (int i = 0; i < Vel_Obj.datagram_size(); i++)
        HS0->write(Vel_Obj[i]);

    return true;
}

bool datos_t::recibirMensaje()
{
    while (HS0->available())
    {
        if (reader.add_uchar(HS0->read()))
        {
            auto m = reader.getMessage();

            switch (m.id)
            {
            case 10:
            {
                m.read_array<int>(velocidadesMotores, 4);

                return actualizarVelocidad(velocidadesMotores[0], velocidadesMotores[1], velocidadesMotores[2], velocidadesMotores[3]);
            }
            break;

            case 11:
            {
                m.read_array<float>(miVelocidad, 3);

                return actualizarVelocidad(miVelocidad[0], miVelocidad[1], miVelocidad[2]);
            }
            break;

            default:
                return false;
            }
        }
    }
    return false;
}
