#include "AltitudeVelocityKF.h"

void AltitudeVelocityKF::setParameters()
{
      F = {1, 0.004,
           0, 1};
      G = {0.5 * 0.004 * 0.004,
           0.004};
      H = {1, 0};
      I = {1, 0,
           0, 1};
      Q = G * ~G * 10.0f * 10.0f;
      R = {30.0f * 30.0f};
      P = {0, 0,
           0, 0};
      S = {0,
           0};
}

void AltitudeVelocityKF::calculate_altitude_velocity(float barometer_altitude, float vertical_acceleration)
{
      Acc = {vertical_acceleration};
      S = F * S + G * Acc;
      P = F * P * ~F + Q;
      L = H * P * ~H + R;
      K = P * (~H) * Inverse(L);
      M = {barometer_altitude};
      S = S + K * (M - H * S);
      P = (I - K * H) * P;
      _altitude = S(0, 0);
      _vertical_velocity = S(1, 0);
}