# pidkit
A minimal, dependency-free PID controller for Python.

## Install
**Latest version:**
```bash
pip install git+https://github.com/chrisfox9158/pidkit.git
```

**v0.2.1:**
```bash
pip install git+https://github.com/chrisfox9158/pidkit.git@v0.2.1
```

## Quick usage
```python
from pidkit import PID

pid = PID(kp=1.0, ki=0.1, kd=0.05, setpoint=100)
correction = pid.compute(pv=95, dt=0.1)
```

Don't know what gains to use for your sim? Let pidkit find them:

```python
from pidkit import autotune_sim, PID

result = autotune_sim(
    plant_factory=lambda: MyPlant(...),
    setpoint=100,
    dt=0.1,
    steps=600,
)

pid = PID(kp=result.kp, ki=result.ki, kd=result.kd, setpoint=100)
```

For full usage details:
- [Controller](docs/controller.md) — pidkit **essential** use; the `PID` class, timing modes, output limiting
- [Autotuning](docs/autotuning.md) — `autotune_sim`, the `aggression` parameter, validation results

## What's new in v0.2.0
- **`autotune_sim`** — automatic PID gain discovery against any simulated plant, requiring only a minimal `step`/`get_state` interface (`pidkit.SimPlant`).
- **`TuneResult`** — structured result object with tuned gains, the full final-trial trace, and diagnostic metrics (settling time, overshoot, oscillation count, final cost).
- **Keyword-only arguments** on `PID.__init__` and `PID.compute()`, preventing silent transposition of same-typed arguments like `kp`/`ki`.

See [CHANGELOG.md](CHANGELOG.md) for the full version history.

## License
MIT License — see [LICENSE](LICENSE)
