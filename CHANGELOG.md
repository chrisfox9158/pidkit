# Changelog
All notable changes to pidkit are documented here. Versions follow [Semantic Versioning](https://semver.org/).

## [0.2.0]
### Added
- **`autotune_sim`** — automatic PID gain discovery against any simulated plant. Finds `kp`, `ki`, and `kd` in sequence:
  - `kp` via exponential search toward the stability boundary (oscillation/overshoot onset), refined by bisection.
  - `ki` and `kd` via exponential search toward the point where a time-weighted error cost (ITAE) plus a control-effort penalty stops improving, refined by golden-section search.
  - Each stage holds previously found gains fixed.
- **`aggression` parameter** (0–1, default 0.5) on `autotune_sim` — controls how close the tuned `kp` lands to its discovered stability boundary versus a gentler reference value, and how strongly `ki`/`kd` are penalized for total control effort. Scaling is geometric (not linear) between the gentle reference and the stability boundary; effective behavior scales proportionally across plants of different magnitude.
- **`base_effort_weight` parameter** — base weight applied to total actuator effort in the `ki`/`kd` cost function, scaled by `(1 - aggression)`.
- **`TuneResult`** — dataclass returned by `autotune_sim`, holding the tuned gains, the full final-trial trace (`times`, `errors`, `pv_values`, `control_outputs`), and diagnostic metrics (`peak_overshoot`, `zero_crossings`, `score`, `stop_time`, `steady_error`).
- **`SimPlant`** — a `runtime_checkable` `Protocol` defining the minimal interface `autotune_sim` requires from a plant: `step(u, dt) -> float` and `get_state() -> float`. No inheritance required; any object with matching methods satisfies the requirements. `autotune_sim` validates the plant returned by `plant_factory()` against this protocol at call time and raises `TypeError` on mismatch.
- **`docs/controller.md`** and **`docs/autotuning.md`** — usage guides split out from the root README, covering `PID` and `autotune_sim` respectively in depth.
- Validation of `autotune_sim` against:
  - First-order lag plants across control_scale 0.05–1.2 and response_scale 30–900.
  - Pure and drag-damped integrator plants across a comparable range.
  - A first-order-plus-dead-time plant, compared directly against the classical Cohen-Coon tuning formula. Resulting `kp`/`kd` landed within the same order of magnitude and proportion as Cohen-Coon's prediction. See `docs/autotuning.md` for full results.
  - Simple Pytest testing in existing `tests/` suite (`tests/test_autotune.py`).

### Changed
- **`PID.__init__` and `PID.compute()` are now keyword-only.** All arguments must be passed by name (`PID(kp=1.0, ki=0.1, ...)` instead of `PID(1.0, 0.1, ...)`). This is a breaking change for any code calling `PID` positionally. **Rationale:** `kp`, `ki`, `kd` sit adjacent in the signature as same-typed floats, and a transposed pair would type-check silently while producing an incorrect controller; keyword-only arguments ensure a visible error.

## [0.1.0]
### Added
- **`PID`** — minimal PID controller class.
  - `compute(pv, dt=None)` — computes one control step. `dt=None` uses automatic wall-clock timing via `time.monotonic()`; an explicit `dt` gives deterministic behavior for simulations, tests, or any caller managing its own timing.
  - `output_limits` — optional `(min, max)` output bounding; either side may be `None` for unbounded.
  - Anti-windup via conditional integral accumulation: integral accumulation pauses while the output is saturated in the direction the error is pushing, and resumes once error begins returning toward setpoint.
- `src/` layout packaging via Hatchling, `pyproject.toml`, editable install support via `pip`/`uv`.
- Initial Pytest suite (`tests/test_pid.py`).
- MIT License.