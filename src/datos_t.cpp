#include "datos_t.h"
#include <math.h>

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
    return clavar();
}

bool datos_t::actualizarVelocidad()
{
    Velocidad_Angular_Anterior.VM1 = roboclaw_DERECHO.ReadSpeedM1(address2);
    Velocidad_Angular_Anterior.VM2 = roboclaw_IZQUERDO.ReadSpeedM2(address1);
    Velocidad_Angular_Anterior.VM3 = roboclaw_IZQUERDO.ReadSpeedM1(address1);
    Velocidad_Angular_Anterior.VM4 = roboclaw_DERECHO.ReadSpeedM2(address2);
    // Velocidades angulares en pusos/s de 90 grados
    Vel_Ang[0] = (0.0f - Velocidad_Ojetivo[0] + Velocidad_Ojetivo[1] + 0.5f * (Length + Width) * Velocidad_Ojetivo[2]) * 2 / PI;
    Vel_Ang[1] = (Velocidad_Ojetivo[0] + Velocidad_Ojetivo[1] - 0.5f * (Length + Width) * Velocidad_Ojetivo[2]) * 2 / PI;
    Vel_Ang[2] = (0.0f - Velocidad_Ojetivo[0] + Velocidad_Ojetivo[1] - 0.5f * (Length + Width) * Velocidad_Ojetivo[2]) * 2 / PI;
    Vel_Ang[3] = (Velocidad_Ojetivo[0] + Velocidad_Ojetivo[1] + 0.5f * (Length + Width) * Velocidad_Ojetivo[2]) * 2 / PI;

    Velocidad_Angular.VM1 = trunc(Vel_Ang[0]);
    Velocidad_Angular.VM2 = trunc(Vel_Ang[1]);
    Velocidad_Angular.VM3 = trunc(Vel_Ang[2]);
    Velocidad_Angular.VM4 = trunc(Vel_Ang[3]);

    Accel_Ang[0] = (float)abs(Velocidad_Angular.VM1 - Velocidad_Angular_Anterior.VM1) / tiempo;
    Accel_Ang[1] = (float)abs(Velocidad_Angular.VM2 - Velocidad_Angular_Anterior.VM2) / tiempo;
    Accel_Ang[2] = (float)abs(Velocidad_Angular.VM3 - Velocidad_Angular_Anterior.VM3) / tiempo;
    Accel_Ang[3] = (float)abs(Velocidad_Angular.VM4 - Velocidad_Angular_Anterior.VM4) / tiempo;

    Aceleracion.VM1 = trunc(Accel_Ang[0]);
    Aceleracion.VM2 = trunc(Accel_Ang[1]);
    Aceleracion.VM3 = trunc(Accel_Ang[2]);
    Aceleracion.VM4 = trunc(Accel_Ang[3]);

    return roboclaw_IZQUERDO.SpeedAccelM1M2_2(address1, Aceleracion.VM3, Velocidad_Angular.VM3, Aceleracion.VM2, Velocidad_Angular.VM2) &&
           roboclaw_DERECHO.SpeedAccelM1M2_2(address2, Aceleracion.VM1, Velocidad_Angular.VM1, Aceleracion.VM4, Velocidad_Angular.VM4);
}

bool datos_t::actualizarVelocidad(int vm1, int vm2, int vm3, int vm4)
{

    Velocidad_Angular_Anterior.VM1 = roboclaw_DERECHO.ReadSpeedM1(address2);
    Velocidad_Angular_Anterior.VM2 = roboclaw_IZQUERDO.ReadSpeedM2(address1);
    Velocidad_Angular_Anterior.VM3 = roboclaw_IZQUERDO.ReadSpeedM1(address1);
    Velocidad_Angular_Anterior.VM4 = roboclaw_DERECHO.ReadSpeedM2(address2);
    // Velocidades angulares en pusos/s de 90 grados
    Velocidad_Angular.VM1 = vm1;
    Velocidad_Angular.VM2 = vm2;
    Velocidad_Angular.VM3 = vm3;
    Velocidad_Angular.VM4 = vm4;

    Accel_Ang[0] = (float)abs(Velocidad_Angular.VM1 - Velocidad_Angular_Anterior.VM1) / tiempo;
    Accel_Ang[1] = (float)abs(Velocidad_Angular.VM2 - Velocidad_Angular_Anterior.VM2) / tiempo;
    Accel_Ang[2] = (float)abs(Velocidad_Angular.VM3 - Velocidad_Angular_Anterior.VM3) / tiempo;
    Accel_Ang[3] = (float)abs(Velocidad_Angular.VM4 - Velocidad_Angular_Anterior.VM4) / tiempo;

    Aceleracion.VM1 = trunc(Accel_Ang[0]);
    Aceleracion.VM2 = trunc(Accel_Ang[1]);
    Aceleracion.VM3 = trunc(Accel_Ang[2]);
    Aceleracion.VM4 = trunc(Accel_Ang[3]);

    return roboclaw_IZQUERDO.SpeedAccelM1M2_2(address1, Aceleracion.VM3, Velocidad_Angular.VM3, Aceleracion.VM2, Velocidad_Angular.VM2) &&
           roboclaw_DERECHO.SpeedAccelM1M2_2(address2, Aceleracion.VM1, Velocidad_Angular.VM1, Aceleracion.VM4, Velocidad_Angular.VM4);
}

bool datos_t::actualizarVelocidad(const Velocidad_t &V)
{
    Velocidad_Angular_Anterior.VM1 = roboclaw_DERECHO.ReadSpeedM1(address2);
    Velocidad_Angular_Anterior.VM2 = roboclaw_IZQUERDO.ReadSpeedM2(address1);
    Velocidad_Angular_Anterior.VM3 = roboclaw_IZQUERDO.ReadSpeedM1(address1);
    Velocidad_Angular_Anterior.VM4 = roboclaw_DERECHO.ReadSpeedM2(address2);

    Velocidad_Angular = V;

    Accel_Ang[0] = (float)abs(Velocidad_Angular.VM1 - Velocidad_Angular_Anterior.VM1) / tiempo;
    Accel_Ang[1] = (float)abs(Velocidad_Angular.VM2 - Velocidad_Angular_Anterior.VM2) / tiempo;
    Accel_Ang[2] = (float)abs(Velocidad_Angular.VM3 - Velocidad_Angular_Anterior.VM3) / tiempo;
    Accel_Ang[3] = (float)abs(Velocidad_Angular.VM4 - Velocidad_Angular_Anterior.VM4) / tiempo;

    Aceleracion.VM1 = trunc(Accel_Ang[0]);
    Aceleracion.VM2 = trunc(Accel_Ang[1]);
    Aceleracion.VM3 = trunc(Accel_Ang[2]);
    Aceleracion.VM4 = trunc(Accel_Ang[3]);

    return roboclaw_IZQUERDO.SpeedAccelM1M2_2(address1, Aceleracion.VM3, Velocidad_Angular.VM3, Aceleracion.VM2, Velocidad_Angular.VM2) &&
           roboclaw_DERECHO.SpeedAccelM1M2_2(address2, Aceleracion.VM1, Velocidad_Angular.VM1, Aceleracion.VM4, Velocidad_Angular.VM4);
}

bool datos_t::actualizarVelocidad(float VX, float VY, float WZ)
{
    Velocidad_Ojetivo[1] = VY;
    Velocidad_Ojetivo[0] = VX;
    Velocidad_Ojetivo[2] = WZ;

    return actualizarVelocidad();
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

Velocidad_t &datos_t::obtenerVelocidad()
{
    return Velocidad_Angular;
}

bool datos_t::modoManual()
{
    Mando.LeftX = controlador->data.analog.stick.lx;
    Mando.LeftY = controlador->data.analog.stick.ly;

    Mando.RightX = controlador->data.analog.stick.rx;
    Mando.RightY = controlador->data.analog.stick.ry;

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
        if (abs(Mando.LeftX) < VMin & abs(Mando.LeftY) < VMin & abs(Mando.RightX) < VMin & abs(Mando.RightY) < VMin)
        {
            return pararTodo();
        }
        else
        {
            return actualizarVelocidad((float)Mando.LeftX * mul_speed,(-1.0f) * (float)Mando.LeftY * mul_speed,(-1.0f) * (float)Mando.RightX * mul_speed_giro);
        }

        
    }
}

bool datos_t::enviarVelocidad()
{
    Velocidad_Angular_Anterior.VM1 = roboclaw_DERECHO.ReadSpeedM1(address2);
    Velocidad_Angular_Anterior.VM2 = roboclaw_IZQUERDO.ReadSpeedM2(address1);
    Velocidad_Angular_Anterior.VM3 = roboclaw_IZQUERDO.ReadSpeedM1(address1);
    Velocidad_Angular_Anterior.VM4 = roboclaw_DERECHO.ReadSpeedM2(address2);

    Vel_Mess.write_array<int>(velocidadesMotores, 4);

    for (int i = 0; i < Vel_Mess.datagram_size(); i++)
        HS0->write(Vel_Mess[i]);

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

                actualizarVelocidad(velocidadesMotores[0], velocidadesMotores[1], velocidadesMotores[2], velocidadesMotores[3]);

                return enviarVelocidad();
            }
            break;

            default:
                return false;
            }
        }
    }
    return false;
}
