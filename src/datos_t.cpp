#include "datos_t.h"

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

    controlador->begin();
}

void datos_t::pararTodo()
{

    Velocidad.reset();
    actualizarVelocidad();

}

bool datos_t::actualizarVelocidad()
{

    return roboclaw_IZQUERDO.SpeedAccelM1M2(address1, acceleration, Velocidad.VM1, Velocidad.VM2) && roboclaw_DERECHO.SpeedAccelM1M2(address2, acceleration, Velocidad.VM3, Velocidad.VM4);
}

void datos_t::modoManual()
{
    Mando.LeftX = controlador->data.analog.stick.lx;
    Mando.LeftY = controlador->data.analog.stick.ly;

    Mando.RightX = controlador->data.analog.stick.rx;
    Mando.RightY = controlador->data.analog.stick.ry;

    // clavada instantanea
    if (controlador->data.button.cross)
    {
        pararTodo();
    }
    else if (controlador->data.button.ps)
    {
        pararTodo();
        ESP.restart();
    }
    else
    {
        // paro
        if (abs(Mando.LeftX) < VMin & abs(Mando.LeftY) < VMin & abs(Mando.RightX) < VMin & abs(Mando.RightY) < VMin)
        {
            Velocidad.reset();
        }
        else
        {
            Velocidad.VM1 = (-1) * mul_speed * (2 * Mando.LeftY - Mando.RightX + Mando.LeftX);
            Velocidad.VM2 = (-1) * mul_speed * (2 * Mando.LeftY - Mando.RightX - Mando.LeftX);

            Velocidad.VM3 = (-1) * mul_speed * (2 * Mando.LeftY + Mando.RightX + Mando.LeftX);
            Velocidad.VM4 = (-1) * mul_speed * (2 * Mando.LeftY + Mando.RightX - Mando.LeftX);
        }
    }
}
