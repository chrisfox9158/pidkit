import time

class PID:
    """Simple PID controller for closed-loop error correction.

    Computes a correction signal from the difference between a setpoint
    and a measured value. Uses Proportional, Integral, Derivative control strategy.

    Example:
        >>> pid = PID(kp=1.0, ki=0.1, kd=0.05, setpoint=100)
        >>> correction = pid.compute(pv=95, dt=0.1)
    
    Args:
        kp: Proportional gain tuner
        ki: Integral gain tuner
        kd: Derivative gain tuner
        setpoint: Target value
        output_limits: Optional (min, max) tuple for output bounding. Either side may be None for unbounded; defaults to fully unbounded.
    """

    def __init__(self, *, kp, ki, kd, setpoint, output_limits=(None, None)):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.setpoint = setpoint

        min_output, max_output = output_limits
        self.min_output = min_output if min_output is not None else float('-inf')
        self.max_output = max_output if max_output is not None else float('inf')

        self._integral = 0
        self._prev_error = None
        self._last_call_time = None

    def compute(self, *, pv, dt=None):
        """Compute the correction signal for one control step via PID.

        Args:
            pv: Current measured process value.
            dt: Elapsed time since the last call, in seconds. If None,
                elapsed time is measured automatically from the system clock.
                Pass explicit dt for deterministic behavior, e.g. in simulations
                where wall-clock time encounters conflicts with simulated time.

        Returns computed control output, clamped to output_limits if set.
        """

        auto = dt is None
        error = self.setpoint - pv

        if auto:
            current_time = time.monotonic()
            if self._last_call_time is None:
                dt = 0
            else:
                dt = current_time - self._last_call_time
            self._last_call_time = current_time

        if self._prev_error is None or dt == 0:
            derivative = 0
        else:
            derivative = (error - self._prev_error) / dt

        output = (self.kp * error) + (self.ki * self._integral) + (self.kd *  derivative)

        output, accumulate = self._clamp(output, error)
        if accumulate:
            self._integral += error * dt

        self._prev_error = error
        return output

    def _clamp(self, output, error):
        if output > self.max_output:
            if error > 0:
                return self.max_output, False
            else:
                return self.max_output, True
        elif output < self.min_output:
            if error < 0:
                return self.min_output, False
            else:
                return self.min_output, True
        else:
            return output, True