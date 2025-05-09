#include "ESC_Calibration.h"
#define ESC_CALIBRATION_HIGH_THROTTLE 1800

void ESC_Calibration::calibrate(float throttle, bool isCalibrationMode)
{
    if (isCalibrationMode)
    {
        if (throttle >= ESC_CALIBRATION_HIGH_THROTTLE)
        {
        }
    }
}