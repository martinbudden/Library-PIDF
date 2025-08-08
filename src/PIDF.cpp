#include "PIDF.h"
#include <cmath>


PIDF::error_t PIDF::getError() const
{
    return error_t {
        .P = _errorPrevious*_pid.kp,
        .I = _errorIntegral, // _erroIntegral is already multiplied by _pid.ki
        .D = _errorDerivative*_pid.kd,
        .F = _setpointDerivative*_pid.kf,
        .S = _setpoint*_pid.ks
    };
}

PIDF::error_t PIDF::getErrorRaw() const
{
    return error_t {
        .P = _errorPrevious,
        .I = (_pid.ki == 0.0F) ? 0.0F : _errorIntegral / _pid.ki,
        .D = _errorDerivative,
        .F = _setpointDerivative,
        .S = _setpoint
    };
}

void PIDF::resetAll()
{
    _setpoint = 0.0F;
    _setpointPrevious = 0.0F;
    _setpointDerivative = 0.0F;
    _errorDerivative = 0.0F;
    _errorIntegral = 0.0F;
    _errorPrevious = 0.0F;
    _measurementPrevious = 0.0F;
}

/*!
Calculate PID output using the provided measurementRate.
This allows the measurementRate to be filtered before the PID update is called.
*/
float PIDF::updateDelta(float measurement, float measurementDelta, float setpointDelta, float deltaT) // NOLINT(bugprone-easily-swappable-parameters)
{
    const float error = _setpoint - measurement;
    _measurementPrevious = measurement;

    if (_integralThreshold == 0.0F || fabsf(error) >= _integralThreshold) {
        // "integrate" the error
        //_errorIntegral += _pid.ki*_error*deltaT; // Euler integration
        _errorIntegral += _pid.ki*0.5F*(error + _errorPrevious)*deltaT; // integration using trapezoid rule
        // Anti-windup via integral clamping
        if (_integralMax > 0.0F && _errorIntegral > _integralMax) {
            _errorIntegral = _integralMax;
        } else if (_integralMin < 0.0F && _errorIntegral < _integralMin) {
            _errorIntegral = _integralMin;
        }
    }
    _errorPrevious = error;

    const float psSum = _pid.kp*error + _pid.ks*_setpoint;
    if (_outputSaturationValue > 0.0F) {
        // Anti-windup by avoiding output saturation.
        // Check if the  output is saturated by P + I + F.
        // If so, the excess value above saturation does not help convergence to the setpoint and will result in
        // overshoot when the P value eventually comes down.
        // So limit the integral to a value that avoids saturation.
        if (psSum + _errorIntegral > _outputSaturationValue) {
            _errorIntegral = std::fmax(_outputSaturationValue - psSum, 0.0F);
        } else if (psSum  + _errorIntegral < -_outputSaturationValue) {
            _errorIntegral = std::fmin(-_outputSaturationValue - psSum, 0.0F);
        }
    }

    _errorDerivative = -measurementDelta / deltaT; // note minus sign, error delta has reverse polarity to measurement delta
    _setpointDerivative = setpointDelta / deltaT;

    // The PID calculation with additional setpoint(openloop) and feedforward(setpoint derivative)
    //                   P + S + I              +  D                       + F
    const float output = psSum + _errorIntegral + _pid.kd*_errorDerivative + _pid.kf*_setpointDerivative;

    return output;
}
