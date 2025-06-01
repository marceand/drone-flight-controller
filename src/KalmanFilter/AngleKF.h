#pragma once

#include <BasicLinearAlgebra.h>

using namespace BLA;

class AngleKF
{
public:
    void setParameters();
    float getAngle()
    {
        return _angle;
    }
    float getGain()
    {
        return _gain;
    }
    float calculateAngle(float angularRate, float angleMeasurement);

private:
    BLA::Matrix<1, 1> F;
    BLA::Matrix<1, 1> G;
    BLA::Matrix<1, 1> P;
    BLA::Matrix<1, 1> Q;
    BLA::Matrix<1, 1> S;
    BLA::Matrix<1, 1> H;
    BLA::Matrix<1, 1> I;
    BLA::Matrix<1, 1> U;
    BLA::Matrix<1, 1> K;
    BLA::Matrix<1, 1> R;
    BLA::Matrix<1, 1> L;
    BLA::Matrix<1, 1> M;

    float _gain;
    float _angle;
};