#include "Parameters.h"
#include <string.h>
#include "ParamNames.h"

const Parameters::Parameter Parameters::params[] = {
    {Parameters::PARAM_FLOAT, ParamNames::CONTROLLER_RATE_ROLL_P_GAIN, {.f = 0.4f}},
    {Parameters::PARAM_FLOAT, ParamNames::CONTROLLER_RATE_ROLL_I_GAIN, {.f = 0.4f}},
    {Parameters::PARAM_FLOAT, ParamNames::CONTROLLER_RATE_ROLL_D_GAIN, {.f = 0.4f}},
    {Parameters::PARAM_FLOAT, ParamNames::CONTROLLER_RATE_PITCH_P_GAIN, {.f = 0.4f}},
    {Parameters::PARAM_FLOAT, ParamNames::CONTROLLER_RATE_PITCH_I_GAIN, {.f = 0.4f}},
    {Parameters::PARAM_FLOAT, ParamNames::CONTROLLER_RATE_PITCH_D_GAIN, {.f = 0.4f}},
    {Parameters::PARAM_FLOAT, ParamNames::CONTROLLER_RATE_YAW_P_GAIN, {.f = 0.4f}},
    {Parameters::PARAM_FLOAT, ParamNames::CONTROLLER_RATE_YAW_I_GAIN, {.f = 0.4f}},
    {Parameters::PARAM_FLOAT, ParamNames::CONTROLLER_RATE_YAW_D_GAIN, {.f = 0.4f}},
    {Parameters::PARAM_FLOAT, ParamNames::PID_TIME_STEP, {.f = 0.4f}},
    {Parameters::PARAM_FLOAT, ParamNames::PID_OUTPUT_LIMIT, {.f = 0.4f}},
    {Parameters::PARAM_FLOAT, ParamNames::PID_INTEGRAL_LIMIT, {.f = 0.4f}},
    {Parameters::PARAM_FLOAT, ParamNames::CONTROLLER_ANGLE_ROLL_P_GAIN, {.f = 0.4f}},
    {Parameters::PARAM_FLOAT, ParamNames::CONTROLLER_ANGLE_ROLL_I_GAIN, {.f = 0.4f}},
    {Parameters::PARAM_FLOAT, ParamNames::CONTROLLER_ANGLE_ROLL_D_GAIN, {.f = 0.4f}},
    {Parameters::PARAM_FLOAT, ParamNames::CONTROLLER_ANGLE_PITCH_P_GAIN, {.f = 0.4f}},
    {Parameters::PARAM_FLOAT, ParamNames::CONTROLLER_ANGLE_PITCH_I_GAIN, {.f = 0.4f}},
    {Parameters::PARAM_FLOAT, ParamNames::CONTROLLER_ANGLE_PITCH_D_GAIN, {.f = 0.4f}},
    {Parameters::PARAM_FLOAT, ParamNames::KF_ANGLE_ROLL_PROCESS_UNCERTAINTY, {.f = 0.4f}},
    {Parameters::PARAM_FLOAT, ParamNames::KF_ANGLE_ROLL_MEASUREMENT_UNCERTAINTY, {.f = 0.4f}},
    {Parameters::PARAM_FLOAT, ParamNames::KF_ANGLE_PITCH_PROCESS_UNCERTAINTY, {.f = 0.4f}},
    {Parameters::PARAM_FLOAT, ParamNames::KF_ANGLE_PITCH_MEASUREMENT_UNCERTAINTY, {.f = 0.4f}},
};

const Parameters::Parameter *Parameters::getParamByName(const char *name)
{
    for (int i = 0; i < sizeof(params) / sizeof(params[0]); ++i)
    {
        if (strcmp(params[i].name, name) == 0)
        {
            return &params[i];
        }
    }
    return nullptr;
}

bool Parameters::getFloat(const char *name, float &out)
{
    const Parameter *p = getParamByName(name);
    if (p && p->type == PARAM_FLOAT)
    {
        out = p->value.f;
        return true;
    }

    return false;
}

bool Parameters::getInt(const char *name, int &out)
{
    const Parameter *p = getParamByName(name);
    if (p && p->type == PARAM_INT)
    {
        out = p->value.i;
        return true;
    }
    return false;
}
