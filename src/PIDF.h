# pragma once

/*!
PID controller with additional open loop control.

Uses "independent PID" notation where the gains are denoted as kp, ki, kd etc.

(In the "dependent PID" notation Kc, tauI, and tauD parameters are used, where kp = Kc, ki = Kc/tauI, kd = Kc*tauD)
*/
class PIDF {
public:
    struct PIDF_t {
        float kp; // proportional gain
        float ki; // integral gain
        float kd; // derivative gain
        float kf; // setpoint derivative gain
        float ks; // setpoint gain
    };
    struct error_t {
        float P;
        float I;
        float D;
        float F;
        float S;
    };
public:
    explicit inline PIDF(const PIDF_t& pid) : _pid {pid.kp, pid.ki, pid.kd, pid.kf, pid.ks}, _kiSaved(pid.ki)  {}
    inline PIDF() : PIDF({0.0F, 0.0F, 0.0F, 0.0F, 0.0F}) {}
public:
    inline void setP(float p) { _pid.kp = p; }
    inline void setI(float i) { _pid.ki = i; _kiSaved = _pid.ki; }
    inline void setD(float d) { _pid.kd = d; }
    inline void setF(float f) { _pid.kf = f; }
    inline void setS(float s) { _pid.ks = s; }
    inline void setPID(const PIDF_t& pid) { _pid = pid; _kiSaved = _pid.ki; }
    inline float getP() const { return _pid.kp; }
    inline float getI() const { return _kiSaved; } // returns the set value of ki, whether integration is turned on or not
    inline float getD() const { return _pid.kd; }
    inline float getF() const { return _pid.kf; }
    inline float getS() const { return _pid.ks; }
    inline const PIDF_t getPID() const { return PIDF_t { _pid.kp, _kiSaved, _pid.kd, _pid.kf, _pid.ks }; }  // returns the set value of ki, whether integration is turned on or not

    inline void resetIntegral() { _errorIntegral = 0.0F; }
    inline void switchIntegrationOff() { _kiSaved = _pid.ki; _pid.ki = 0.0F; _errorIntegral = 0.0F; }
    inline void switchIntegrationOn() { _pid.ki = _kiSaved; _errorIntegral = 0.0F; }

    inline void setIntegralMax(float integralMax) { _integralMax = integralMax; }
    inline void setIntegralMin(float integralMin) { _integralMin = integralMin; }
    inline void setIntegralLimit(float integralLimit) { _integralMax = integralLimit; _integralMin = -integralLimit; }
    inline void setIntegralThreshold(float integralThreshold) { _integralThreshold = integralThreshold; }
    inline void setOutputSaturationValue(float outputSaturationValue) { _outputSaturationValue = outputSaturationValue; }

    inline void setSetpoint(float setpoint) { _setpointPrevious = _setpoint; _setpoint = setpoint; }
    inline void setSetpoint(float setpoint, float deltaT) {
        _setpointPrevious = _setpoint;
        _setpoint = setpoint;
        _setpointDerivative = (_setpoint - _setpointPrevious)/deltaT;
    }
    inline void setSetpointDerivative(float setpointDerivative) { _setpointDerivative = setpointDerivative; }

    inline float getSetpoint() const { return _setpoint; }
    inline float getPreviousSetpoint() const { return _setpointPrevious; }
    inline float getSetpointDelta() const { return _setpoint - _setpointPrevious; }

    inline float getPreviousMeasurement() const { return _measurementPrevious; } //!< get previous measurement, useful for DTerm filtering

    inline float update(float measurement, float deltaT) {
        return updateDelta(measurement, measurement - _measurementPrevious, deltaT);
    }
    inline float updateDelta(float measurement, float measurementDelta, float deltaT) {
        return updateDeltaITerm(measurement, measurementDelta, _setpoint - measurement, deltaT);
    }

    float updateDeltaITerm(float measurement, float measurementDelta, float iTermError, float deltaT);

    float updatePS(float measurement);
    float updatePIS(float measurement, float deltaT);
    float updatePDS(float measurement, float measurementDelta, float deltaT);

    // accessor functions to obtain error values
    error_t getError() const;
    error_t getErrorRaw() const;
    inline float getErrorP() const { return _errorPrevious*_pid.kp; }
    inline float getErrorI() const { return _errorIntegral; } // _erroIntegral is already multiplied by _pid.ki
    inline float getErrorD() const { return _errorDerivative*_pid.kd; }
    inline float getErrorF() const { return _setpointDerivative*_pid.kf; }
    inline float getErrorS() const { return _setpoint*_pid.ks; }

    inline float getErrorRawP() const { return _errorPrevious; }
    inline float getErrorRawI() const { return (_pid.ki == 0.0F) ? 0.0F : _errorIntegral / _pid.ki; }
    inline float getErrorRawD() const { return _errorDerivative; }
    inline float getErrorRawF() const { return _setpointDerivative; }
    inline float getErrorRawS() const { return _setpoint; }

    inline float getPreviousError() const { return _errorPrevious; } //!< get previous error, for test code

    void resetAll(); //!< reset all, for test code
    static inline float clip(float value, float min, float max) { return value < min ? min : value > max ? max : value; }
private:
    PIDF_t _pid;
    float _kiSaved; //!< saved value of _pid.ki, so integration can be switched on and off
    float _measurementPrevious {0.0F};

    float _setpoint {0.0F};
    float _setpointPrevious {0.0F};
    float _setpointDerivative {0.0F};

    float _errorDerivative {0.0F};
    float _errorIntegral {0.0F};
    float _errorPrevious {0.0F};

    // integral anti-windup parameters
    float _integralMax {0.0F}; //!< Integral windup limit for positive integral
    float _integralMin {0.0F}; //!< Integral windup limit for negative integral
    float _integralThreshold {0.0F}; //!< Threshold for PID integration. Can be set to avoid integral wind-up due to movement in motor's backlash zone.
    float _outputSaturationValue {0.0F}; //!< For integral windup control
};
