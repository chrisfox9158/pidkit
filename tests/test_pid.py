from pidkit import PID

def test_zero():
    pid = PID(kp=1.0, ki=0.1, kd=0.05, setpoint=50)
    output = pid.compute(pv=50, dt=0.1)
    assert output == 0

def test_convergence():
    pid = PID(kp=0.5, ki=0.1, kd=0.05, setpoint=100)
    pv = 0
    for _ in range(200):
        output = pid.compute(pv=pv, dt=0.1)
        pv += output * 0.1
    assert abs(pid.setpoint - pv) < 1.0

def test_intensive():
    pid = PID(kp=0, ki=1.0, kd=0, setpoint=100, output_limits=(-10, 10))

    # Build integral without saturating
    for _ in range(3):
        pid.compute(pv=95, dt=1)

    # Saturate; integral freeze
    pid.compute(pv=95, dt=1)
    frozen_integral = pid._integral

    # Keep integral frozen
    pid.compute(pv=95, dt=1)
    assert pid._integral == frozen_integral

    # Error flips, still same clamping
    pid.compute(pv=105, dt=1)
    assert pid._integral != frozen_integral