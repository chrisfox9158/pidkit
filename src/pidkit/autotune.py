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

