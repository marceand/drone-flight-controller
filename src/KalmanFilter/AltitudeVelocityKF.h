#pragma once

#include <BasicLinearAlgebra.h>

using namespace BLA;

class AltitudeVelocityKF
{
public:
    void setParameters();
    void calculate_altitude_velocity(float barometer_altitude, float vertical_acceleration);
    float getAltitude()
    {
        return _altitude;
    }
    float getVerticalVelocity()
    {
        return _vertical_velocity;
    }

    float getGainAltitude()
    {
        return _gain_altitude;
    }
    float getGainVelocity()
    {
        return _gain_velocity;
    }

private:
    float _altitude;
    float _vertical_velocity;
    float _gain_altitude;
    float _gain_velocity;
    BLA::Matrix<2, 2> F;
    BLA::Matrix<2, 1> G;
    BLA::Matrix<2, 2> P;
    BLA::Matrix<2, 2> Q;
    BLA::Matrix<2, 1> S;
    BLA::Matrix<1, 2> H;
    BLA::Matrix<2, 2> I;
    BLA::Matrix<1, 1> Acc;
    BLA::Matrix<2, 1> K;
    BLA::Matrix<1, 1> R;
    BLA::Matrix<1, 1> L;
    BLA::Matrix<1, 1> M;
};
