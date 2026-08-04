from pidkit import autotune_sim, TuneResult, SimPlant

class _TestPlant:
    def __init__(self, state=0.0, gain=1.0, tau=1.0):
        self._state = state
        self._gain = gain
        self._tau = tau

    def step(self, u, dt):
        self._state += dt * ((-self._state + self._gain * u) / self._tau)
        return self._state

    def get_state(self):
        return self._state

def test_autotune_stability():
    plant_factory = lambda: _TestPlant(state=0.0, gain=1.0, tau=5.0)

    result = autotune_sim(
        plant_factory=plant_factory,
        setpoint=10.0,
        dt=0.1,
        steps=1000
    )

    assert result.zero_crossings < 2
    assert result.kp > 0
    assert result.ki > 0
    assert result.kd > 0

def test_autotune_stability_secondary():
    plant_factory = lambda: _TestPlant(state=0.0, gain=0.05, tau=200.0)

    result = autotune_sim(
        plant_factory=plant_factory,
        setpoint=1000.0,
        dt=0.5,
        steps=1000
    )

    assert result.zero_crossings < 2
    assert result.kp > 0