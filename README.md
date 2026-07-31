# pidkit
A minimal, standalone PID controller for error correction in Python.

## Install
**Latest Version:**
```bash
pip install git+https://github.com/chrisfox9158/pidkit.git
```

**v0.1.0:**
```bash
pip install git+https://github.com/chrisfox9158/pidkit.git@v0.1.0
```

## Quick usage
```python
from pidkit import PID

pid = PID(kp=1.0, ki=0.1, kd=0.05, setpoint=100)

# Control step use:
correction = pid.compute(pv=95, dt=0.1)
```
`compute()` returns a single correction signal; this signal must be interpreted and used effectively by user implementation.

## Timing Modes
`compute()` accepts an optional `dt`:
- **Pass `dt` explicitly** for deterministic timing. Use cases include simulations, tests, or any loop with existing time-tracking. Highly recommended for sim environments.
- **Omit `dt` (or pass `None`)** and the controller measures elapsed time automatically using the system clock. Suited for real-time use cases.

Switching between timing modes on one PID instance is not supported due to cross-compatibility issues between real-time and passed-time calls; single-mode use is recommended.

## Output Limiting
```python
pid = PID(kp=1.0, ki=0.1, kd=0.05, setpoint=100, output_limits=(-10, 10))
```

`output_limits` optionally bounds the returned correction. Either side of the `(min, max)` tuple input can be `None` for unbounded. When the output saturates, integral accumulation pauses automatically if the error is still pushing further into that bound, preventing integral windup. When error begins to return to the setpoint, integral accumulation resumes regardless of saturation.

## API
### PID Initialization
```python
PID(kp, ki, kd, setpoint, output_limits=(None, None))
```
Initializes PID instance with specified arguments:
- `kp`, `ki`, `kd` — Proportional, integral, and derivative gain tuners
- `setpoint` — Target value for controlled environment variable
- `output_limits` — Optional `(min, max)` tuple bounding output; either side may be `None`

### .compute():
```python
.compute(pv, dt=None)
```
Computes and returns the correction signal for single control step with specified arguments:
- `pv` — Process variable, or current measured value of the controlled environment variable
- `dt` — Timestep; see [Timing Modes](#timing-modes) above for manual and automatic behaviors

## License
MIT License — see [LICENSE](LICENSE)
