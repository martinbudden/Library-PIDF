#include "PIDF.h"
#include <cmath>


PIDF::error_t PIDF::getError() const
{
    return error_t {
        .P = _errorPrevious*_pid.kp,
        .I = _errorIntegral, // _erroIntegral is already multiplied by _pid.ki
        .D = _errorDerivative*_pid.kd,
        .F = _setpoint*_pid.kf
    };
}

PIDF::error_t PIDF::getErrorRaw() const
{
    return error_t {
        .P = _errorPrevious,
        .I = (_pid.ki == 0.0F) ? 0.0F : _errorIntegral / _pid.ki,
        .D = _errorDerivative,
        .F = _setpoint
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
    const float pfSum = _pid.kp*error + _pid.kf*_setpoint;
    if (_outputSaturationValue != 0.0F) {
        // Anti-windup by avoiding output saturation.
        // Check if the  output is saturated by P + I + F.
        // If so, the excess value above saturation does not help convergence to the setpoint and will result in overshoot when the P value eventually comes down.
        // So limit the integral to a value that avoids saturation.
        if (pfSum + _errorIntegral > _outputSaturationValue) {
            _errorIntegral = std::fmax(_outputSaturationValue - pfSum, 0.0F);
        } else if (pfSum  + _errorIntegral < -_outputSaturationValue) {
            _errorIntegral = std::fmin(-_outputSaturationValue - pfSum, 0.0F);
        }
    }

    _errorDerivative = -measurementDelta / deltaT; // note minus sign, error delta has reverse polarity to measurement delta

    _errorPrevious = error;
    _measurementPrevious = measurement;

    // The PID calculation with additional feedforward
    //                   P + F + I              +  D
    const float output = pfSum + _errorIntegral + _pid.kd*_errorDerivative ;

    return output;
}
