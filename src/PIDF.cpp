#include "PIDF.h"
#include <cmath>


PIDF::error_t PIDF::getError() const
{
    return error_t {
        .P = _errorPrevious*_pid.kp,
        .I = _errorIntegral, // _erroIntegral is already multiplied by _pid.ki
        .D = _errorDerivative*_pid.kd
    };
}

PIDF::error_t PIDF::getErrorRaw() const
{
    return error_t {
        .P = _errorPrevious,
        .I = (_pid.ki == 0.0F) ? 0.0F : _errorIntegral / _pid.ki,
        .D = _errorDerivative
    };
}

/*!
Calculate PID output using the provided measurementRate.
This allows the measurementRate to be filtered before the PID update is called.
*/
float PIDF::updateDelta(float measurement, float measurementDelta, float deltaT) // NOLINT(bugprone-easily-swappable-parameters)
{
    const float error = _setpoint - measurement;

    if (fabsf(error) > _integralThreshold) {
        // "integrate" the error
        //_errorIntegral += _pid.ki*_error*deltaT; // Euler integration
        _errorIntegral += _pid.ki*0.5F*(error + _errorPrevious)*deltaT; // integration using trapezoid rule
        // Anti-windup via integral clamping
        if (_integralMax > 0.0F) {
            _errorIntegral = clip(_errorIntegral, -_integralMax, _integralMax);
        }
    }
    const float pValue = _pid.kp*error;
    if (_outputSaturationValue != 0.0F && fabsf(pValue) > _outputSaturationValue) {
        // The output is saturated by the pValue, so the integral value will not increase output and will result in overshoot when the pValue eventually comes down,
        // so set the integral to zero.
        _errorIntegral = 0.0F;
    }

    _errorDerivative = -measurementDelta / deltaT; // note minus sign, error delta has reverse polarity to measurement delta
    _errorPrevious = error;
    _measurementPrevious = measurement;

    // The PID calculation with additional feedforward
    //                   P        I                D                          F
    const float output = pValue + _errorIntegral + _pid.kd*_errorDerivative + _pid.kf*_setpoint;

    return output;
}
