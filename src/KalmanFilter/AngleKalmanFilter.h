#pragma once

class AngleKalmanFilter
{
public:
    void setParams(float dt, float processUncertainty, float measurementUncertainty){
        _dt = dt;
        _process_uncertainty = processUncertainty;
        _measurement_uncertainty = measurementUncertainty;
    }

    float calculate(float angularRate, float angleMeasurement);

private:
    float _previous_angle;
    float _previous_uncertainty;
    float _process_uncertainty;
    float _measurement_uncertainty;
    float _dt;
    float calculateAnglePrediction(float angularRate);
    float calculateUncertaintyPrediction();
    float calculateGain(float uncertaintyPredicted);
    float calculateAngleEstimation(float anglePredicted, float angleMeasurement, float gain);
    float calculateUncertaintyEstimation(float uncertaintyPredicted, float gain);
};