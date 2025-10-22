![license](https://img.shields.io/badge/license-MIT-green) ![open source](https://badgen.net/badge/open/source/blue?icon=github)

# PID Library

This library contains a [PID controller](https://en.wikipedia.org/wiki/Proportional-integral-derivative_controller) with
additional [feed forward](https://en.wikipedia.org/wiki/Feed_forward_(control)) and setpoint components.

The PID controller has the following features:

1. Addition of optional feed forward components (ie components base solely on the setpoint).
   The PID calculation pseudocode is:<br>
   `output = kp*error + ki*errorIntegral + kd*errorDerivative + ks*setpoint + kk*setpointDerivative`<br>
   Setting `ks` and `kk` to zero gives a traditional PID controller.
   Setting `kp`, `ki`, `kd`, and `ks` to zero gives pure open-loop control.
2. Calculation of derivative on measurement, avoiding "derivative kick" when the setpoint changes. If derivative kick is desired,
   then it can be added by setting the K-term (`kk`) and calling `setSetpointDerivative` when the setpoint changes.
3. _delta-t_ input parameter to PID `update` function. This allows for jitter in the timing of the call to the `update` function.
4. A choice of two methods of controlling integral windup. Either the integral term can be limited to a maximum value,
   or it can be limited when the output saturates. Both methods can be used together, if desired.
5. Additional update function, `updateDelta`, with a `measurementDelta` parameter. Providing this parameter
   allows filtering of `measurementDelta` before the PID calculation.
6. Support for dynamic PID control, where the PID constants are changed at runtime, in particular:
    1. Ability to switch integration off. Useful, for example, for a vehicle that has its motors turned on, but has not yet started moving.
    2. Additional update function `updateDeltaITerm`, with a `iTermError` parameter. This allows the user to calculate the I-term
       error. This can be used, for example, to implement ITerm relaxation.
    3. Functions to return the current error terms. (These can also be used for PID tuning, telemetry, and test)
7. Optimized forms of the `update` function, `updateSP`, `updateSPI`, and `updateSPD` that avoid unnecessary calculations
   for a P-controller, a PI-controller, and a PD-controller. These can be used when performance is critical (ie when very
   short loop times are used).

The PID controller deliberately does not implement these features:

1. PID "negation". Some PID controllers provide a function to negate all the PID constants. If this is required,
   just subtract the PID output, rather than add it.
2. Option to calculate derivative on input. This can be achieved by calculating `inputDelta` rather than `measurementDelta`
   and using it as input to the `updateDelta` function.
   Note this value must be negated, ie `updateDelta(measurement, -inputDelta, deltaT)` should be called.
3. Filtering of the D-term. Providing a D-term filter limits flexibility - the user no choice in the type of filter used.
   Instead the `updateDelta` function can be used, with `measurementDelta` filtered by a filter provided by the user.
