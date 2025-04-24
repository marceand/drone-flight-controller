#include "AngleController.h"
#include "../Parameters/Parameters.h"
#include "../Parameters/ParamNames.h"

void AngleController::setParameters()
{
    float roll_p_gain;
    float roll_i_gain;
    float roll_d_gain;
    Parameters::getFloat(ParamNames::CONTROLLER_ANGLE_ROLL_P_GAIN, roll_p_gain);
    Parameters::getFloat(ParamNames::CONTROLLER_ANGLE_ROLL_I_GAIN, roll_i_gain);
    Parameters::getFloat(ParamNames::CONTROLLER_ANGLE_ROLL_D_GAIN, roll_d_gain);
    _rollPID.setGains(roll_p_gain, roll_i_gain, roll_d_gain);

    float pitch_p_gain;
    float pitch_i_gain;
    float pitch_d_gain;
    Parameters::getFloat(ParamNames::CONTROLLER_ANGLE_PITCH_P_GAIN, pitch_p_gain);
    Parameters::getFloat(ParamNames::CONTROLLER_ANGLE_PITCH_I_GAIN, pitch_i_gain);
    Parameters::getFloat(ParamNames::CONTROLLER_ANGLE_PITCH_D_GAIN, pitch_d_gain);
    _pitchPID.setGains(pitch_p_gain, pitch_i_gain, pitch_d_gain);

    float time_step;
    Parameters::getFloat(ParamNames::PID_TIME_STEP, time_step);
    _rollPID.setTimeStep(time_step);
    _pitchPID.setTimeStep(time_step);

    float output_limit;
    Parameters::getFloat(ParamNames::PID_OUTPUT_LIMIT, output_limit);
    _rollPID.setOutputLimit(output_limit);
    _pitchPID.setOutputLimit(output_limit);

    float integral_limit;
    Parameters::getFloat(ParamNames::PID_INTEGRAL_LIMIT, integral_limit);
    _rollPID.setIntegralLimit(integral_limit);
    _pitchPID.setIntegralLimit(integral_limit);
}

float AngleController::computeRollPID(float desired, float actual)
{
    return _rollPID.computePID(desired, actual);
}

float AngleController::computePitchPID(float desired, float actual)
{
    return _pitchPID.computePID(desired, actual);
}
