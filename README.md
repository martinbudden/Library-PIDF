![license](https://img.shields.io/badge/license-MIT-green) ![open source](https://badgen.net/badge/open/source/blue?icon=github)

# PID Library

This library contains a [PID controller](https://en.wikipedia.org/wiki/Proportional-integral-derivative_controller) with
additional [feed forward](https://en.wikipedia.org/wiki/Feed_forward_(control)).

The PID controller has the following features:

1. Addition of optional feed forward component. The PID calculation pseudocode is:<br>
   `output = kp*error + ki*errorIntegral + kd*errorDerivative + kf*setpoint`<br>
   Setting `kf` to zero gives a traditional PID controller.
2. Calculation of derivative on measurement, avoiding "derivative kick" when the setpoint changes.
3. Two forms of the update function `update` and `updateDelta` with a `measurementDelta` parameter. Providing this parameter
   allows filtering `measurementDelta` before the PID calculation.
4. _delta-t_ input parameter to PID `update` function. This allows for jitter in the timing of the call to the `update` function.
5. A choice of two methods of controlling integral windup. Either the integral term can be limited to a maximum value,
   or it can be set to zero when the output saturates. Both methods can be used together, if desired.
6. Ability to switch integration off. Useful, for example, for a vehicle that has its motors turned on, but has not yet started moving.
7. Functions to return the current error terms. These can be used for PID tuning, telemetry, and test code.

The PID controller deliberately does not implement these features:

1. Output clipping. Usually it is desirable to clip the PID output before it is fed to the device being controlled.
   However often several PID outputs are combined, when this is the case it is the combined output that needs to be clipped.
2. PID "negation". Some PID controllers provide a function to negate all the PID constants. If this is required,
   just subtract the PID output, rather than add it.
3. Option to calculate derivative on input. This can be achieved by calculating `inputDelta` rather than `measurementDelta`
   and using it as input to the `updateDelta` function.
   Note this value must be negated, ie `updateDelta(measurement, -inputDelta, deltaT)` should be called.
4. Filtering of the D-term. Providing a D-term filter limits flexibility - the user no choice in the type of filter used.
   Instead the `updateDelta` function can be used, with `measurementDelta` filtered by a filter provided by the user.
