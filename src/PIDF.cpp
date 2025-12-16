#include "PIDF.h"
#include <cmath>


PIDF::error_t PIDF::getError() const
{
    return error_t {
        .P = _errorPrevious*_pid.kp,
        .I = _errorIntegral, // _erroIntegral is already multiplied by _pid.ki
        .D = _errorDerivative*_pid.kd,
        .S = _setpoint*_pid.ks,
        .K = _setpointDerivative*_pid.kk
    };
}

PIDF::error_t PIDF::getErrorRaw() const
{
    return error_t {
        .P = _errorPrevious,
        .I = (_pid.ki == 0.0F) ? 0.0F : _errorIntegral / _pid.ki,
        .D = _errorDerivative,
        .S = _setpoint,
        .K = _setpointDerivative
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
Calculate PID output using the provided measurementRate and ITerm error.
This allows the measurementRate to be filtered and the ITerm error to be attenuated 
before the PID update is called.
*/
float PIDF::updateDeltaITerm(float measurement, float measurementDelta, float iTermError, float deltaT) // NOLINT(bugprone-easily-swappable-parameters)
{
    _measurementPrevious = measurement;
    const float error = _setpoint - measurement;
    _errorDerivative = -measurementDelta / deltaT; // note minus sign, error delta has reverse polarity to measurement delta
    // Partial PID sum, excludes ITerm
    // has additional S setpoint(openloop) and F feedforward(setpoint derivative) terms
    //                       P             +  D                       + S                 + K (no ITerm)
    const float partialSum = _pid.kp*error + _pid.kd*_errorDerivative + _pid.ks*_setpoint + _pid.kk*_setpointDerivative;

    if (_integralThreshold == 0.0F || fabsf(error) >= _integralThreshold) {
        // "integrate" the error
        _errorIntegral += _pid.ki*iTermError*deltaT; // Euler integration
        //_errorIntegral += _pid.ki*0.5F*(iTermError + _errorPrevious)*deltaT; // integration using trapezoid rule
        // Anti-windup via integral clamping
        if (_integralMax > 0.0F && _errorIntegral > _integralMax) {
            _errorIntegral = _integralMax;
        } else if (_integralMin < 0.0F && _errorIntegral < _integralMin) {
            _errorIntegral = _integralMin;
        }
    }
    _errorPrevious = error;

    if (_outputSaturationValue > 0.0F) {
        // Anti-windup by avoiding output saturation.
        // Check if partialSum + _errorIntegral saturates the output
        // If so, the excess value above saturation does not help convergence to the setpoint and will result in
        // overshoot when the P value eventually comes down.
        // So limit the _errorIntegral to a value that avoids output saturation.
        if (_errorIntegral > _outputSaturationValue - partialSum) {
            _errorIntegral = std::fmax(_outputSaturationValue - partialSum, 0.0F);
        } else if (_errorIntegral < -_outputSaturationValue - partialSum) {
            _errorIntegral = std::fmin(-_outputSaturationValue - partialSum, 0.0F);
        }
    }


    // The PID calculation with additional S setpoint(openloop) and F feedforward(setpoint derivative) terms
    //                   P+D+S+F    +  I
    const float output = partialSum + _errorIntegral;

    return output;
}

/*
Optimized update of S and P terms only (P controller).
*/
float PIDF::updateSP(float measurement) // NOLINT(bugprone-easily-swappable-parameters)
{
    _measurementPrevious = measurement;
    const float error = _setpoint - measurement;
    _errorPrevious = error;

    // The P (no I, no D) calculation with additional S setpoint(openloop) term
    //                   P             + S
    const float output = _pid.kp*error + _pid.ks*_setpoint;

    return output;
}

/*
Optimized update of S, P, and I terms only (PI controller)
*/
float PIDF::updateSPI(float measurement, float deltaT) // NOLINT(bugprone-easily-swappable-parameters)
{
    _measurementPrevious = measurement;
    const float error = _setpoint - measurement;
    const float partialSum = _pid.kp*error + _pid.ks*_setpoint;

    if (_integralThreshold == 0.0F || fabsf(error) >= _integralThreshold) {
        // "integrate" the error
        _errorIntegral += _pid.ki*error*deltaT; // Euler integration
        //_errorIntegral += _pid.ki*0.5F*(error + _errorPrevious)*deltaT; // integration using trapezoid rule
        // Anti-windup via integral clamping
        if (_integralMax > 0.0F && _errorIntegral > _integralMax) {
            _errorIntegral = _integralMax;
        } else if (_integralMin < 0.0F && _errorIntegral < _integralMin) {
            _errorIntegral = _integralMin;
        }
    }
    _errorPrevious = error;

    if (_outputSaturationValue > 0.0F) {
        // Anti-windup by avoiding output saturation.
        // Check if partialSum + _errorIntegral saturates the output
        // If so, the excess value above saturation does not help convergence to the setpoint and will result in
        // overshoot when the P value eventually comes down.
        // So limit the _errorIntegral to a value that avoids output saturation.
        if (_errorIntegral > _outputSaturationValue - partialSum) {
            _errorIntegral = std::fmax(_outputSaturationValue - partialSum, 0.0F);
        } else if (_errorIntegral < -_outputSaturationValue - partialSum) {
            _errorIntegral = std::fmin(-_outputSaturationValue - partialSum, 0.0F);
        }
    }

    // The PI (no D) calculation with additional S setpoint(openloop) term
    //                   P + S      +  I
    const float output = partialSum + _errorIntegral;

    return output;
}

/*
Optimized update of S, P, and D terms only (PD controller).
*/
float PIDF::updateSPD(float measurement, float measurementDelta, float deltaT) // NOLINT(bugprone-easily-swappable-parameters)
{
    _measurementPrevious = measurement;
    const float error = _setpoint - measurement;

    _errorPrevious = error;

    _errorDerivative = -measurementDelta / deltaT; // note minus sign, error delta has reverse polarity to measurement delta

    // The PD (no I) calculation with additional S setpoint(openloop) term
    //                   P             + D                        + S
    const float output = _pid.kp*error + _pid.kd*_errorDerivative + _pid.ks*_setpoint;

    return output;
}
