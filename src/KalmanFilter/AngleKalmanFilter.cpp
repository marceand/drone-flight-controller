#include "AngleKalmanFilter.h"

float AngleKalmanFilter::calculate(float angularRate, float angleMeasurement)
{
    float anglePredicted = calculateAnglePrediction(angularRate);
    float uncertaintyPredicted = calculateUncertaintyPrediction();
    float gain = calculateGain(uncertaintyPredicted);
    float angleEstimated = calculateAngleEstimation(anglePredicted, angleMeasurement, gain);
    float uncertaintyEstimated  = calculateUncertaintyEstimation(uncertaintyPredicted, gain);

    _previous_angle = angleEstimated;
    _previous_uncertainty = uncertaintyEstimated;

    return angleEstimated;
}

float AngleKalmanFilter::calculateAnglePrediction(float angularRate)
{
    return _previous_angle + (_dt * angularRate);
}

float AngleKalmanFilter::calculateUncertaintyPrediction()
{
    return _previous_uncertainty + _process_uncertainty;
}

float AngleKalmanFilter::calculateGain(float uncertaintyPredicted)
{
    return uncertaintyPredicted / (uncertaintyPredicted + _measurement_uncertainty);
}

float AngleKalmanFilter::calculateAngleEstimation(float anglePredicted, float angleMeasurement, float gain)
{
    return anglePredicted + gain * (angleMeasurement - anglePredicted);
}

float AngleKalmanFilter::calculateUncertaintyEstimation(float uncertaintyPredicted, float gain)
{
    return uncertaintyPredicted - (gain *  uncertaintyPredicted);
}
