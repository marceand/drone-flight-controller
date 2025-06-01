#include "AngleKF.h"

void AngleKF::setParameters()
{
    F = {1.0f};
    G = {0.004};
    H = {1.0f};
    I = {1.0f};
    Q = {0.004 * 0.004 * 4.0 * 4.0};
    R = {3.0 * 3.0};
    P = {2.0 * 2.0};
    S = {0.0f};
}

float AngleKF::calculateAngle(float angularRate, float angleMeasurement)
{
    U = {angularRate};
    S = F * S + G * U;
    P = F * P * ~F + Q;
    L = H * P * ~H + R;
    K = P * (~H) * Inverse(L);
    M = {angleMeasurement};
    S = S + K * (M - H * S);
    P = (I - K * H) * P;

    _gain = K(0, 0);
    _angle = S(0, 0);

    return _angle;
}
