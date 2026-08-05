# Autotuning
`autotune_sim` finds PID gains automatically against any plant that can be simulated with a constant timestep.

## Quick Start
```python
from pidkit import autotune_sim, PID

result = autotune_sim(
    plant_factory=lambda: MyPlant(...),
    setpoint=25.0,
    dt=0.1,
    steps=600,
)

pid = PID(kp=result.kp, ki=result.ki, kd=result.kd, setpoint=25.0)
```
Quick use is simple: call autotune_sim with a configured plant_factory, and feed the results to a PID initialization. For more on factory setup, see below.

## Usage
`autotune_sim` acts as a plant-agnostic system; the call only needs a **factory**: a zero-argument callable that returns a fresh plant instance on every call.

### Setup options
**Lambda Format:**
```python
plant_factory = lambda: Thermostat(control_scale=1.0, response_scale=60.0)
```

**Function Format:**
```python
def plant_factory():
    return Thermostat(control_scale=1.0, response_scale=60.0)
```

### Eligibility
Your plant just needs two methods to be eligible for `autotune_sim` use:
```python
def step(self, u: float, dt: float) -> float: ...  # advance state, return new state
def get_state(self) -> float: ...                    # report current state
```

No inheritance is required as this is a structural contract (`pidkit.SimPlant`) rather than a base class. Anything with matching methods qualifies, regardless of additional structures.

## How it works
Gains are found in sequence, each stage holding the previous gain results constant:
1. **kp** — found via exponential search: start small, double until the response oscillates or overshoots badly, then bisect to refine the exact boundary. This produces the largest stable kp for the plant.
2. **ki** — found via exponential search toward the point where a time-weighted error cost (ITAE) stops improving, then refined with golden-section search.
3. **kd** — same method as ki, searched last, with kp and ki both held fixed.

The final kp is not used at its raw discovered boundary; see below.

## The `aggression` parameter
`aggression` (0 to 1, default 0.25) controls how close the tuning lands to the most aggressive stable response versus a gentler one.

A lower `aggression` parameter is recommended for smoother-approach response curves. 0.2 to 0.4 are reliable values.

- **High aggression (near 1.0)** uses kp close to its true stability boundary, with little penalty on how hard the controller pushes. While fast, this can drive an actuator to its limits for an extended stretch, especially against a large initial error.
- **Low aggression (near 0.0)** scales kp toward a gentler reference value sized to avoid saturating `output_limits` (or, if unbounded, sized relative to the initial error), and increases the cost penalty against ki/kd for total control effort. This creates a slower, smoother approach.

**Known limitation:** `aggression` is currently a single knob calibrated by feel, not by a measured trade-off. There's no explicit slowness penalty in the aggression-linked PID gains discovery system. Thus, very low aggression values can become continuously gentler with no lower bound tied to acceptable response time. This system currently depends on user adjustment to preference.

## Known behavior: kd often lands near zero
For simple, non-resonant plants (first-order lag, simple integrators), `kd` frequently converges to a very small value. This is expected as derivative action contributes little without resonance or dead time for it to counteract. It has been observed to converge to a consistent near-zero value across several different basic first-order plants. However, against a lagged system, the discovered `kd` value increased noticeably; it is likely that simplistic plants simply do not require a `kd` parameter as more complex systems demand.

## Validation
`autotune_sim` has been tested against:
- **First-order lag** (`Thermostat`-style plants) across widely different scales (control_scale 0.05–1.2, response_scale 30–900), with consistent discovery of stable, non-oscillating gains without manual configuration.
- **Pure and drag-damped integrators** (`CartVelocity`-style plants) provided similar results to `Thermostat`-style plants across a similar range of scales.
- **First-order-plus-dead-time**, compared directly against the classical Cohen-Coon tuning formula (chosen as Cohen-Coon is derived especially for dead-time-dominant processes). **Results:**

  Test plant: `thermostat_delayed`
  ```
  control_scale=1.0, response_scale=30.0, t_ambient = 20.0, t_initial = 20.0

  theta=15.0, setpoint=30.0, output_limits=(None, None)

  aggression=1.0 (zeroes effort penalty for undamped output),
  base_effort_weight=0.05 (irrelevant at aggression=1.0),
  
  steps=400, timestep=1
  ```

  | | Cohen-Coon | autotune_sim | 
  |---|---|---|
  | kp | 2.92 | 2.05 |
  | ki | 0.094 | 0.041 |
  | kd | 14.58 | 16.78 |

  kp and kd land within the same order of magnitude and proportion to one another. ki is significantly lower than Cohen-Coon's prediction, plausibly because Cohen-Coon's formula has no mechanism to account for oscillation risk, while autotune_sim's ITAE-based search actively penalizes the lingering, time-weighted error that aggressive integral action risks feeding under a large dead time. The resulting response curve is stable and convergent (visible decaying oscillation settling by ~5x the dead time), consistent with this explanation.

**Not yet tested:** resonant or underdamped second-order systems (mass-spring-damper-style plants). Dead time and simple first-order dynamics are covered; oscillatory plant dynamics driven by the plant itself, rather than a delay, remain unvalidated.

## Parameters
See the `autotune_sim` docstring for the full parameter list (`crossing_threshold`, `overshoot_threshold`, `doubling_cap`, `refinement_cap`, `stop_tolerance`, `error_tolerance`, `start_candidate`, `base_effort_weight`)

## TuneResult
`autotune_sim` returns a `TuneResult` dataclass:

```python
result.kp, result.ki, result.kd       # tuned gains

result.score                          # final combined cost (ITAE + effort)
result.peak_overshoot                 # largest post-setpoint error magnitude
result.zero_crossings                 # oscillation count in the final trial
result.stop_time, result.steady_error # settling metrics, or None if it never settled

result.times, result.errors,
result.pv_values, result.control_outputs  # full traces from the final trial
```

Full field descriptions exist in the `TuneResult` docstring.