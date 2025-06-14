# pragma once

class PIDF {
public:
    struct PIDF_t {
        float kp;
        float ki;
        float kd;
        float kf;
    };
    struct error_t {
        float P;
        float I;
        float D;
    };
public:
    explicit inline PIDF(const PIDF_t& pid) : _pid {pid.kp, pid.ki, pid.kd, pid.kf}, _kiSaved(pid.ki)  {}
    inline PIDF() : PIDF({0.0F, 0.0F, 0.0F, 0.0F}) {}
public:
    inline void setP(float p) { _pid.kp = p; }
    inline void setI(float i) { _pid.ki = i; _kiSaved = _pid.ki; }
    inline void setD(float d) { _pid.kd = d; }
    inline void setF(float f) { _pid.kf = f; }
    inline void setPID(const PIDF_t& pid) { _pid = pid; _kiSaved = _pid.ki; }
    inline float getP() const { return _pid.kp; }
    inline float getI() const { return _kiSaved; } // returns the set value of ki, whether integration is turned on or not
    inline float getD() const { return _pid.kd; }
    inline float getF() const { return _pid.kf; }
    inline const PIDF_t getPID() const { return PIDF_t { _pid.kp, _kiSaved, _pid.kd, _pid.kf }; }  // returns the set value of ki, whether integration is turned on or not

    inline void resetIntegral() { _errorIntegral = 0.0F; }
    //! switch off integration, to avoid integral windup
    inline void switchIntegrationOff() { _kiSaved = _pid.ki; _pid.ki = 0.0F; _errorIntegral = 0.0F; }
    inline void switchIntegrationOn() { _pid.ki = _kiSaved; _errorIntegral = 0.0F; }

    inline void setIntegralMax(float integralMax) { _integralMax = integralMax; }
    inline void setIntegralThreshold(float integralThreshold) { _integralThreshold = integralThreshold; }
    inline void setOutputSaturationValue(float outputSaturationValue) { _outputSaturationValue = outputSaturationValue; }

    inline void setSetpoint(float setpoint) { _setpoint = setpoint; }
    inline float getSetpoint() const { return _setpoint; }

    inline float getPreviousMeasurement() const { return _measurementPrevious; } //!< get previous measurement, useful for DTerm filtering

    inline float update(float measurement, float deltaT) { return updateDelta(measurement, measurement - _measurementPrevious, deltaT); }
    float updateDelta(float measurement, float measurementDelta, float deltaT);

    // accessor functions to obtain error values for instrumentation
    error_t getError() const;
    error_t getErrorRaw() const;
private:
    static float clip(float value, float min, float max) { return value < min ? min : value > max ? max : value; }
private:
    PIDF_t _pid;
    float _kiSaved; //!< saved value of _pid.ki, so integration can be switched on and off
    float _setpoint {0.0F};
    float _errorDerivative {0.0F}; // stored for instrumentation and telemetry
    float _errorIntegral {0.0F};
    float _errorPrevious {0.0F};
    float _measurementPrevious {0.0F};
    float _integralMax {0.0F};
    float _integralThreshold {0.0F}; //!< Threshold for PID integration. Can be set to avoid integral wind-up due to movement in motor's backlash zone.
    float _outputSaturationValue {0.0F};
};
