import math
from typing import Protocol, runtime_checkable
from dataclasses import dataclass
from .pid import PID

@runtime_checkable
class SimPlant(Protocol):
    """The minimal interface autotune_sim (and any plant-driven pidkit
    tooling) expects from a plant.

    A plant does not need to inherit from this class; any passed model
    with matching step()/get_state() methods fulfills the expectation.

    Methods:
        step(u, dt): Advance the plant's internal state forward by one
            timestep, given a control input u and elapsed time dt.
            Returns the new state.
        get_state(): Return the plant's current state.
    """

    def step(self, u: float, dt: float) -> float: ...
    def get_state(self) -> float: ...

def _run_trial(*, plant_factory, kp, ki, kd, setpoint, dt, steps, output_limits):
    plant = plant_factory()
    pid = PID(kp=kp, ki=ki, kd=kd, setpoint=setpoint, output_limits=output_limits)

    times, errors, control_outputs = [], [], []
    t = 0
    for step in range(steps):
        pv = plant.get_state()
        u = pid.compute(pv=pv, dt=dt)
        error = setpoint - pv
        plant.step(u, dt)

        times.append(t)
        errors.append(error)
        control_outputs.append(u)

        t += dt

    return times, errors, control_outputs

def _effort_cost(*, control_outputs, dt):
    """Integral of absolute control output; stand-in for actuator effort."""
    cost = sum(abs(u) for u in control_outputs) * dt
    return cost

def _itae(*, times, errors, dt):
    """Integral of time-weighted absolute error.
    
    Assumes uniform timestep spacing (constant dt);
    designed for use with simulation plants where
    dt is not affected by outside variability (such as
    relay-based hardware irregularity).
    """
    cost = sum(t * abs(error) for t, error in zip(times, errors)) * dt
    return cost

def _count_zero_crossings(*, errors):
    def _check_cross(num1, num2):
        if (num1 > 0 and num2 < 0) or (num1 < 0 and num2 > 0):
            return True
        return False

    count = 0
    for i in range(1, len(errors)):
        if _check_cross(errors[i], errors[i-1]):
            count += 1

    return count

def _max_overshoot_ratio(*, errors):
    initial = errors[0]
    initial_sign = initial > 0

    overshoot_errors = [
        error for error in errors
        if (error > 0) != initial_sign and error != 0
        ]
    if not overshoot_errors:
        return 0.0, 0.0

    peak_error = max(abs(error) for error in overshoot_errors)
    error_ratio = peak_error / abs(initial)
    return error_ratio, peak_error

def _check_reject(*, errors, crossing_threshold=2, overshoot_threshold=0.5):
    crossings = _count_zero_crossings(errors=errors)
    overshoot_ratio, peak_error = _max_overshoot_ratio(errors=errors)
    return crossings >= crossing_threshold or overshoot_ratio >= overshoot_threshold

def _find_kp(*, plant_factory, setpoint, dt, steps, output_limits,
            crossing_threshold, overshoot_threshold,
            doubling_cap, refinement_cap, stop_tolerance, start_candidate):

    # Doubling phase for bracket discovery
    candidate = start_candidate
    low = 0
    high = float('inf')
    for i in range(doubling_cap):
        trace_times, trace_errors, trace_controls = _run_trial(kp=candidate, ki=0, kd=0, plant_factory=plant_factory, setpoint=setpoint, dt=dt, steps=steps, output_limits=output_limits)
        if _check_reject(errors=trace_errors,crossing_threshold=crossing_threshold, overshoot_threshold=overshoot_threshold):
            high = candidate
            break
        low = candidate
        candidate *= 2
    if high == float('inf'):
        raise RuntimeError("kp search exceeded doubling_cap without finding unstable boundary. Please increase doubling_cap")

    # Halving phase for refinement
    for i in range(refinement_cap):
        mid = (low + high) / 2
        trace_times, trace_errors, trace_controls = _run_trial(kp=mid, ki=0, kd=0, plant_factory=plant_factory, setpoint=setpoint, dt=dt, steps=steps, output_limits=output_limits)
        if _check_reject(errors=trace_errors, crossing_threshold=crossing_threshold, overshoot_threshold=overshoot_threshold):
            high = mid
        else:
            low = mid
        if (high - low) / (low + math.ulp(0.0)) < stop_tolerance:
            break

    return low

def _find_ki(*, plant_factory, kp, setpoint, dt, steps, output_limits,
            doubling_cap, refinement_cap, stop_tolerance, start_candidate,
            effort_weight):

    # Candidate bracket discovery
    def _cost(ki):
        times, errors, control_outputs = _run_trial(plant_factory=plant_factory, kp=kp, ki=ki, kd=0, setpoint=setpoint, dt=dt, steps=steps, output_limits=output_limits)
        return _itae(times=times, errors=errors, dt=dt) + effort_weight * _effort_cost(control_outputs=control_outputs, dt=dt)

    prev_ki = start_candidate
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

        if (high - low) / (low + math.ulp(0.0)) < stop_tolerance:
            break

    mid = (high + low) / 2
    return mid

def _find_kd(*, plant_factory, kp, ki, setpoint, dt, steps, output_limits,
            doubling_cap, refinement_cap, stop_tolerance, start_candidate,
            effort_weight):

    # Candidate bracket discovery
    def _cost(kd):
        times, errors, control_outputs = _run_trial(plant_factory=plant_factory, kp=kp, ki=ki, kd=kd, setpoint=setpoint, dt=dt, steps=steps, output_limits=output_limits)
        return _itae(times=times, errors=errors, dt=dt) + effort_weight * _effort_cost(control_outputs=control_outputs, dt=dt)

    prev_kd = start_candidate
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

        if (high - low) / (low + math.ulp(0.0)) < stop_tolerance:
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

def _find_tolerance_margin(*, times, errors, error_tolerance):
    tolerance = error_tolerance * abs(errors[0])
    stop_time, steady_error = None, None

    for i, error in reversed(list(enumerate(errors))):
        if abs(error) >= tolerance:
            stop_time = times[i + 1]
            steady_error = sum(error for error in errors[i + 1:]) / len(errors[i + 1:])
            break

    return stop_time, steady_error

@dataclass
class TuneResult:
    """Result of an autotuning run.

    Attributes:
        kp: Tuned proportional gain.
        ki: Tuned integral gain.
        kd: Tuned derivative gain.
        times: Timestamps from the final trial, in seconds.
        errors: setpoint-pv difference error at each timestep of the final trial.
        pv_values: Process variable at each timestep of the final trial.
        control_outputs: PID control output at each timestep of the final trial.
        peak_overshoot: Largest error magnitude reached after crossing setpoint, if any.
        zero_crossings: Number of times the error changed sign during the final trial.
        score: Final ITAE cost of the tuned gains.
        stop_time: Time at which error settled within tolerance, or None if the system did not settle.
        steady_error: Average error over the settled region, or None if the system did not settle.
    """
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
    stop_time: float | None
    steady_error: float | None

def autotune_sim(*, plant_factory, setpoint, dt, steps,
                output_limits=(None, None), 
                crossing_threshold=2,
                overshoot_threshold=0.5,
                doubling_cap=50,
                refinement_cap=100,
                stop_tolerance=1e-3,
                error_tolerance=0.02,
                start_candidate=1e-6,
                aggression=0.25,
                base_effort_weight=0.05):
    """Automatically tune PID gain values for any
    simulated plant with a constant timestep.

    Searches for kp, ki, and kd in sequence: kp is found
    via exponential search biased toward stability
    (minimal overshoot / oscillation behavior),
    and subsequently bisected for refinement.
    ki and kd depend on exponential search biased toward
    a point where time-weighted error (ITAE) is minimized,
    then refined via golden-section search. Each stage holds
    previous gains as a fixed value.

    The plant is not touched directly; the system depends on
    a user-supplied factory which provides per-trial instances
    in order to avoid cross-trial data leaks.

    Example:
        >>> result = autotune_sim(plant_factory=lambda: Thermostat(control_scale=1.0, response_scale=60.0), setpoint=25.0, dt=0.1, steps=600)
        >>> pid = PID(kp=result.kp, ki=result.ki, kd=result.kd, setpoint=25.0)

    Args:
        plant_factory: Zero-argument callable returning a fresh plant
            instance. The plant must follow the conventions of the
            SimPlant protocol.
        setpoint: Target value to tune toward.
        dt: Fixed timestep used for every simulated trial.
        steps: Number of timesteps to simulate per trial.
        output_limits: Optional (min, max) tuple bounding the PID's
            control output during tuning. Either side may be None for
            unbounded; defaults to fully unbounded.
        crossing_threshold: Number of error sign-changes, indicating oscillation,
            in a trial's trace during the kp search.
        overshoot_threshold: Ratio of peak overshoot to initial error
            that counts as excessive during the kp search.
        doubling_cap: Maximum exponential-search iterations per stage
            before raising, if no boundary/minimum is found.
        refinement_cap: Maximum refinement iterations (bisection or
            golden-section) per stage.
        stop_tolerance: Relative bracket-width tolerance that stops
            refinement early, once reached.
        error_tolerance: Fraction of initial error used to define the
            settling band for stop_time/steady_error metrics.
        start_candidate: Starting value for each stage's exponential
            search. Should be small relative to any expected gain scale.
        aggression: Value from 0 to 1 controlling how close kp lands to
            the discovered stable boundary, and strength of penalty against
            ki/kd for actuator effort. 1 uses the raw boundary kp with
            no effort penalty (fast, enables bang-bang behavior); 0 scales
            kp toward gentle behavior to avoid output_limits saturation
            and greatly increases the effort penalty.
        base_effort_weight: Base weight applied to total actuator effort
            in the ki/kd cost function, scaled by (1 - aggression).

    Returns:
        TuneResult: Dataclass with the tuned gains, the full final trial trace,
            and diagnostic metrics. See TuneResult for field details.
    """
    if not isinstance(plant_factory(), SimPlant):
        raise TypeError("plant_factory must return an object matching the SimPlant protocol (step(u, dt) and get_state()).")

    initial_state = plant_factory().get_state()
    initial_error = setpoint - initial_state

    min_output, max_output = output_limits
    relevant_limit = max_output if initial_error > 0 else min_output
    if relevant_limit is not None:
        kp_gentle = abs(relevant_limit) / abs(initial_error)
    else:
        kp_gentle = 1 / abs(initial_error)

    boundary_kp = _find_kp(plant_factory=plant_factory, setpoint=setpoint, dt=dt, steps=steps, output_limits=output_limits,
            crossing_threshold=crossing_threshold, overshoot_threshold=overshoot_threshold,
            doubling_cap=doubling_cap, refinement_cap=refinement_cap, stop_tolerance=stop_tolerance, start_candidate=start_candidate)

    kp = (kp_gentle ** (1 - aggression)) * (boundary_kp ** aggression)
    effort_weight = base_effort_weight * (1 - aggression)

    ki = _find_ki(plant_factory=plant_factory, kp=kp, setpoint=setpoint, dt=dt, steps=steps, output_limits=output_limits,
            doubling_cap=doubling_cap, refinement_cap=refinement_cap, stop_tolerance=stop_tolerance, start_candidate=start_candidate,
            effort_weight=effort_weight)
    
    kd = _find_kd(plant_factory=plant_factory, kp=kp, ki=ki, setpoint=setpoint, dt=dt, steps=steps, output_limits=output_limits,
            doubling_cap=doubling_cap, refinement_cap=refinement_cap, stop_tolerance=stop_tolerance, start_candidate=start_candidate,
            effort_weight=effort_weight)

    times, errors, pv_values, control_outputs = _run_final_trial(plant_factory=plant_factory, kp=kp, ki=ki, kd=kd, setpoint=setpoint, dt=dt, steps=steps, output_limits=output_limits)

    overshoot_ratio, peak_overshoot = _max_overshoot_ratio(errors=errors)
    zero_crossings = _count_zero_crossings(errors=errors)
    score = _itae(times=times, errors=errors, dt=dt) + effort_weight * _effort_cost(control_outputs=control_outputs, dt=dt)
    stop_time, steady_error = _find_tolerance_margin(times=times, errors=errors, error_tolerance=error_tolerance)

    return TuneResult(
        kp= kp,
        ki= ki,
        kd= kd,

        times= times,
        errors= errors,
        pv_values= pv_values,
        control_outputs= control_outputs,

        peak_overshoot= peak_overshoot,
        zero_crossings= zero_crossings,
        score= score,
        stop_time= stop_time,
        steady_error= steady_error
    )
