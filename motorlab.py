"""Sampled PI control of a brushed DC motor; all quantities are in SI units."""
from dataclasses import dataclass
import math
import random


@dataclass(frozen=True)
class Motor:
    resistance: float = 1.0
    inductance: float = 0.5
    inertia: float = 0.01
    damping: float = 0.1
    k: float = 0.01  # torque and back-EMF constants, equal in SI

    def __post_init__(self):
        for value in (self.resistance, self.inductance, self.inertia, self.damping, self.k):
            if not math.isfinite(value) or value <= 0:
                raise ValueError("Motor parameters must be finite and positive")

    def derivative(self, state, voltage, load=0.0):
        current, speed = state
        return ((voltage - self.resistance * current - self.k * speed) / self.inductance,
                (self.k * current - self.damping * speed - load) / self.inertia)

    def equilibrium(self, voltage, load=0.0):
        speed = (self.k * voltage - self.resistance * load) / (
            self.resistance * self.damping + self.k**2)
        return ((voltage - self.k * speed) / self.resistance, speed)


def step(motor, state, voltage, load, dt):
    """RK4 plant integration with voltage and opposing load held constant."""
    if not math.isfinite(dt) or dt <= 0:
        raise ValueError("dt must be finite and positive")
    def shifted(slope, scale):
        return tuple(x + scale * dt * d for x, d in zip(state, slope))
    a = motor.derivative(state, voltage, load)
    b = motor.derivative(shifted(a, 0.5), voltage, load)
    c = motor.derivative(shifted(b, 0.5), voltage, load)
    d = motor.derivative(shifted(c, 1.0), voltage, load)
    return tuple(x + dt * (aa + 2*bb + 2*cc + dd) / 6
                 for x, aa, bb, cc, dd in zip(state, a, b, c, d))


@dataclass
class PI:
    kp: float
    ki: float
    limit: float = 24.0
    integral: float = 0.0  # integral contribution, in volts

    def __post_init__(self):
        if (not all(math.isfinite(x) for x in (self.kp, self.ki, self.limit))
                or self.kp < 0 or self.ki < 0 or self.limit <= 0):
            raise ValueError("Gains must be nonnegative and voltage limit positive")

    def update(self, error, dt):
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("dt must be finite and positive")
        increment = self.ki * error * dt
        candidate = self.kp * error + self.integral + increment
        # Conditional integration prevents accumulating error into saturation.
        if (abs(candidate) <= self.limit
                or (candidate > self.limit and increment < 0)
                or (candidate < -self.limit and increment > 0)):
            self.integral += increment
        return max(-self.limit, min(self.limit, self.kp * error + self.integral))


@dataclass(frozen=True)
class Scenario:
    name: str
    motor: Motor = Motor()
    target: float = 1.0
    load: float = 0.0
    load_at: float = 2.0


TRAIN = (
    Scenario("nominal"),
    Scenario("load", load=0.025),
    Scenario("heavy_rotor", motor=Motor(inertia=0.016), load=0.015),
)
HELD_OUT = (
    Scenario("hot_winding", motor=Motor(resistance=1.3), load=0.035),
    Scenario("higher_friction", motor=Motor(damping=0.13), load=0.02),
    Scenario("lower_speed", target=0.65, load=0.04),
)


def simulate(gains, scenario, dt=0.005, duration=5.0):
    if not math.isfinite(duration) or duration <= 0 or not math.isfinite(dt) or dt <= 0:
        raise ValueError("duration and dt must be finite and positive")
    steps = round(duration / dt)
    if steps < 1 or not math.isclose(steps * dt, duration, abs_tol=1e-10):
        raise ValueError("duration must be an integer multiple of dt")
    controller = PI(*gains)
    state = (0.0, 0.0)
    rows = []
    for n in range(steps):
        t = n * dt
        load = scenario.load if t >= scenario.load_at else 0.0
        voltage = controller.update(scenario.target - state[1], dt)
        rows.append((t, state[0], state[1], voltage, load))
        state = step(scenario.motor, state, voltage, load, dt)
    # Final state is explicit; controls at this endpoint describe the prior interval.
    rows.append((duration, state[0], state[1], rows[-1][3], rows[-1][4]))
    return rows


def simulate_sampled(gains, scenario, control_dt=0.005, plant_dt=0.001,
                     duration=5.0, noise_std=0.0, seed=0):
    """Separate controller sampling from plant integration.

    Independent zero-mean Gaussian speed measurement noise is drawn once per
    control update. Voltage is held between updates. Returned speeds are true
    plant states, never noisy measurements. Periods and load events must align
    with the plant grid so neither integration nor disturbance timing changes
    accidentally when the controller rate is varied.
    """
    if (not all(math.isfinite(v) and v > 0 for v in (control_dt, plant_dt, duration))
            or not math.isfinite(noise_std) or noise_std < 0):
        raise ValueError("Periods must be positive; noise must be nonnegative and finite")

    def grid_index(value):
        index = round(value / plant_dt)
        if not math.isclose(index * plant_dt, value, rel_tol=0, abs_tol=1e-10):
            raise ValueError("Periods and load events must align with the plant grid")
        return index

    stride, count = grid_index(control_dt), grid_index(duration)
    if stride < 1 or count < 1:
        raise ValueError("Controller period and duration must be at least plant_dt")
    if not math.isfinite(scenario.load_at):
        raise ValueError("Load time must be finite")
    load_index = grid_index(scenario.load_at)
    rng, controller = random.Random(seed), PI(*gains)
    state, rows, voltage = (0.0, 0.0), [], 0.0
    for n in range(count):
        load = scenario.load if n >= load_index else 0.0
        if n % stride == 0:
            measured_speed = state[1] + (rng.gauss(0, noise_std) if noise_std else 0.0)
            voltage = controller.update(scenario.target - measured_speed, control_dt)
        rows.append((n * plant_dt, state[0], state[1], voltage, load))
        state = step(scenario.motor, state, voltage, load, plant_dt)
    rows.append((duration, state[0], state[1], voltage, load))
    return rows


def metrics(rows, target):
    dt = rows[1][0] - rows[0][0]
    errors = [abs(target - r[2]) for r in rows]
    iae = dt * (sum(errors) - (errors[0] + errors[-1]) / 2)
    effort = dt * sum(r[3]**2 for r in rows[:-1])
    overshoot = max(0.0, max(r[2] for r in rows) - target)
    return {"iae_rad": iae, "voltage_effort_V2s": effort,
            "overshoot_rad_s": overshoot, "final_error_rad_s": errors[-1],
            "peak_current_A": max(abs(r[1]) for r in rows),
            "saturation_fraction": sum(abs(r[3]) >= 24 for r in rows[:-1]) / (len(rows)-1)}


def score(gains, scenarios=TRAIN):
    # Normalize units: 1 rad error, 24^2 * 5 V^2 s effort, 1 rad/s overshoot.
    values = [metrics(simulate(gains, s), s.target) for s in scenarios]
    return sum(m["iae_rad"] + 0.1*m["voltage_effort_V2s"]/(24**2*5)
               + 2*m["overshoot_rad_s"] for m in values) / len(values)
