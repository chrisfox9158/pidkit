import math
from dataclasses import dataclass
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

def _find_kp(*, plant_factory, setpoint, dt, steps, output_limits,
            crossing_threshold, overshoot_threshold,
            doubling_cap, refinement_cap, tolerance):

    # Doubling phase for bracket discovery
    candidate = 1e-6
    low = 0
    high = float('inf')
    for i in range(doubling_cap):
        trace_times, trace_errors = _run_trial(kp=candidate, ki=0, kd=0, plant_factory=plant_factory, setpoint=setpoint, dt=dt, steps=steps, output_limits=output_limits)
        if _check_reject(trace_errors, crossing_threshold, overshoot_threshold):
            high = candidate
            break
        low = candidate
        candidate *= 2
    if high == float('inf'):
        raise RuntimeError("kp search exceeded doubling_cap without finding unstable boundary. Please increase doubling_cap")

    # Halving phase for refinement
    for i in range(refinement_cap):
        mid = (low + high) / 2
        trace_times, trace_errors = _run_trial(kp=mid, ki=0, kd=0, plant_factory=plant_factory, setpoint=setpoint, dt=dt, steps=steps, output_limits=output_limits)
        if _check_reject(trace_errors, crossing_threshold, overshoot_threshold):
            high = mid
        else:
            low = mid
        if (high - low) / (low + math.ulp(0.0)) < tolerance:
            break

    return low

def _find_ki(*, plant_factory, kp, setpoint, dt, steps, output_limits,
            doubling_cap, refinement_cap, tolerance):

    # Candidate bracket discovery
    def _cost(ki):
        times, errors = _run_trial(plant_factory=plant_factory, kp=kp, ki=ki, kd=0, setpoint=setpoint, dt=dt, steps=steps, output_limits=output_limits)
        return _itae(times, errors, dt)

    prev_ki = 1e-6
    prev_cost = _cost(prev_ki)
    ki = prev_ki * 2

    bracketed = False
    for i in range(doubling_cap):
        cost = _cost(ki)
        if cost >= prev_cost:
            bracketed = True
            break
        prev_ki, prev_cost = ki, cost
        ki *= 2
    if not bracketed:
        raise RuntimeError("ki search exceeded doubling_cap without finding cost minimum. Please increase doubling_cap")

    # Golden-section refinement
    low = prev_ki
    high = ki

    phi = (1 + 5 ** 0.5) / 2 # golden ratio, approx. 1.618
    phi_complement = (1 / phi) ** 2

    for i in range(refinement_cap):
        test1 = low + phi_complement * (high - low)
        test2 = high - phi_complement * (high - low)
        cost1, cost2 = _cost(test1), _cost(test2)

        if cost1 >= cost2:
            low = test1
        elif cost1 < cost2:
            high = test2

        if (high - low) / (low + math.ulp(0.0)) < tolerance:
            break

    mid = (high + low) / 2
    return mid

def _find_kd(*, plant_factory, kp, ki, setpoint, dt, steps, output_limits,
            doubling_cap, refinement_cap, tolerance):

    # Candidate bracket discovery
    def _cost(kd):
        times, errors = _run_trial(plant_factory=plant_factory, kp=kp, ki=ki, kd=kd, setpoint=setpoint, dt=dt, steps=steps, output_limits=output_limits)
        return _itae(times, errors, dt)

    prev_kd = 1e-6
    prev_cost = _cost(prev_kd)
    kd = prev_kd * 2

    bracketed = False
    for i in range(doubling_cap):
        cost = _cost(kd)
        if cost >= prev_cost:
            bracketed = True
            break
        prev_kd, prev_cost = kd, cost
        kd *= 2
    if not bracketed:
        raise RuntimeError("kd search exceeded doubling_cap without finding cost minimum. Please increase doubling_cap")

    # Golden-section refinement
    low = prev_kd
    high = kd

    phi = (1 + 5 ** 0.5) / 2 # golden ratio, approx. 1.618
    phi_complement = (1 / phi) ** 2

    for i in range(refinement_cap):
        test1 = low + phi_complement * (high - low)
        test2 = high - phi_complement * (high - low)
        cost1, cost2 = _cost(test1), _cost(test2)

        if cost1 >= cost2:
            low = test1
        elif cost1 < cost2:
            high = test2

        if (high - low) / (low + math.ulp(0.0)) < tolerance:
            break

    mid = (high + low) / 2
    return mid

def _run_final_trial(*, plant_factory, kp, ki, kd, setpoint, dt, steps, output_limits):

    plant = plant_factory()
    pid = PID(kp=kp, ki=ki, kd=kd, setpoint=setpoint, output_limits=output_limits)

    times, errors, pv_values, control_outputs = [], [], [], []
    t = 0
    for step in range(steps):
        pv = plant.get_state()
        u = pid.compute(pv=pv, dt=dt)
        error = setpoint - pv
        plant.step(u, dt)

        times.append(t)
        errors.append(error)
        pv_values.append(pv)
        control_outputs.append(u)

        t += dt

    return times, errors, pv_values, control_outputs

def _find_tolerance_margin(times, errors, tolerance_scale):
    tolerance = tolerance_scale * abs(errors[0])
    stop_time, steady_error = None, None

    for i, error in reversed(list(enumerate(errors))):
        if abs(error) >= tolerance:
            stop_time = times[i + 1]
            steady_error = sum(error for error in errors[i + 1:]) / len(errors[i + 1:])
            break

    return stop_time, steady_error

@dataclass
class TuneResult:
    kp: float
    ki: float
    kd: float

    times: list[float]
    errors: list[float]
    pv_values: list[float]
    control_outputs: list[float]

    peak_overshoot: float
    zero_crossings: int
    score: float
    stop_time: float
    steady_error: float

