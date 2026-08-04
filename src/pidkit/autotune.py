import math
from .pid import PID

def _run_trial(*, plant_factory, kp, ki, kd, setpoint, dt, steps, output_limits):
    plant = plant_factory()
    pid = PID(kp=kp, ki=ki, kd=kd, setpoint=setpoint, output_limits=output_limits)

    times, errors = [], []
    t = 0
    for step in range(steps):
        pv = plant.get_state()
        u = pid.compute(pv=pv, dt=dt)
        error = setpoint - pv
        plant.step(u, dt)

        times.append(t)
        errors.append(error)

        t += dt

    return times, errors

def _itae(times, errors, dt):
    """Integral of time-weighted absolute error.
    
    Assumes uniform timestep spacing (constant dt);
    designed for use with simulation plants where
    dt is not affected by outside variability (such as
    relay-based hardware irregularity).
    """
    cost = sum(t * abs(error) for t, error in zip(times, errors)) * dt
    return cost

def _count_zero_crossings(errors):
    def _check_cross(num1, num2):
        if (num1 > 0 and num2 < 0) or (num1 < 0 and num2 > 0):
            return True
        return False

    count = 0
    for i in range(1, len(errors)):
        if _check_cross(errors[i], errors[i-1]):
            count += 1

    return count

def _max_overshoot_ratio(errors):
    initial = errors[0]
    initial_sign = initial > 0

    overshoot_errors = [
        error for error in errors
        if (error > 0) != initial_sign and error != 0
        ]
    if not overshoot_errors:
        return 0.0

    peak = max(abs(error) for error in overshoot_errors)
    return peak / abs(initial)

def _check_reject(errors, crossing_threshold=2, overshoot_threshold=0.5):
    crossings = _count_zero_crossings(errors)
    overshoot_ratio = _max_overshoot_ratio(errors)
    return crossings >= crossing_threshold or overshoot_ratio >= overshoot_threshold

