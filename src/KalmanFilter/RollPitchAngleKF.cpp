#include "RollPitchAngleKF.h"
#include "../Parameters/Parameters.h"
#include "../Parameters/ParamNames.h"

void RollPitchAngleKF::setParameters()
{
    // float time_step;
    // Parameters::getFloat(ParamNames::PID_TIME_STEP, time_step);

    // float roll_process_uncertainty;
    // float roll_measurement_uncertainty;

    // Parameters::getFloat(ParamNames::KF_ANGLE_ROLL_PROCESS_UNCERTAINTY, roll_process_uncertainty);
    // Parameters::getFloat(ParamNames::KF_ANGLE_ROLL_MEASUREMENT_UNCERTAINTY, roll_measurement_uncertainty);

    // _rollAngleKF.setParameters(time_step, roll_process_uncertainty, roll_measurement_uncertainty);

    // float pitch_process_uncertainty;
    // float pitch_measurement_uncertainty;

    // Parameters::getFloat(ParamNames::KF_ANGLE_PITCH_PROCESS_UNCERTAINTY, pitch_process_uncertainty);
    // Parameters::getFloat(ParamNames::KF_ANGLE_PITCH_MEASUREMENT_UNCERTAINTY, pitch_measurement_uncertainty);

    // _pitchAngleKF.setParameters(time_step, pitch_process_uncertainty, pitch_measurement_uncertainty);
}

float RollPitchAngleKF::calculateRoll(float angularRate, float angleMeasurement)
{
    return _rollAngleKF.calculate(angularRate, angleMeasurement);
}

float RollPitchAngleKF::calculatePitch(float angularRate, float angleMeasurement)
{
    return _pitchAngleKF.calculate(angularRate, angleMeasurement);
}
