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

float AltitudeVelocityKF::calculateVerticalVelocity(float barometer_altitude, float vertical_acceleration)
{
      U = {vertical_acceleration};
      S = F * S + G * U;
      P = F * P * ~F + Q;
      L = H * P * ~H + R;
      K = P * (~H) * Inverse(L);
      M = {barometer_altitude};
      S = S + K * (M - H * S);
      P = (I - K * H) * P;

      _gain_altitude = K(0, 0);
      _gain_vertical_velocity = K(1, 0);
      _altitude = S(0, 0);
      _vertical_velocity = S(1, 0);

      return _vertical_velocity;
}