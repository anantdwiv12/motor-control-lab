"""Independent matrix-exponential reference for piecewise-constant motor inputs."""
from functools import lru_cache
import math
import numpy as np
from scipy.linalg import expm
from motorlab import step


@lru_cache(maxsize=128)
def transition(motor, dt):
    # Construct directly from physical parameters; do not call Motor.derivative.
    # Augmented state is [current, speed, held voltage, held opposing load].
    matrix = np.array([
        [-motor.resistance/motor.inductance, -motor.k/motor.inductance, 1/motor.inductance, 0],
        [motor.k/motor.inertia, -motor.damping/motor.inertia, 0, -1/motor.inertia],
        [0, 0, 0, 0],
        [0, 0, 0, 0],
    ])
    result = expm(matrix * dt)
    result.flags.writeable = False
    return result


def reference_step(motor, state, voltage, load, dt):
    if not math.isfinite(dt) or dt < 0:
        raise ValueError('Step duration must be finite and nonnegative')
    result = transition(motor, dt) @ np.array([*state, voltage, load])
    return tuple(float(x) for x in result[:2])


def trajectory(motor, events, dt=0.005, duration=1.0, method='reference'):
    """Return (time, current, speed) on a uniform output grid.

    Events are (time, voltage, load), strictly increasing and starting at zero.
    Split integration at every event, even off-grid; new inputs apply from that
    event onward. State is continuous at events. This open-loop validation
    harness does not alter the existing closed-loop simulator's grid contract.
    """
    if method not in ('reference', 'rk4'):
        raise ValueError('Unknown integration method')
    if not all(math.isfinite(x) and x > 0 for x in (dt, duration)):
        raise ValueError('dt and duration must be finite and positive')
    count = round(duration/dt)
    if count < 1 or not math.isclose(count*dt, duration, rel_tol=0, abs_tol=1e-10):
        raise ValueError('Duration must align with output grid')
    if (not events
            or any(len(e) != 3 or not all(math.isfinite(x) for x in e) for e in events)
            or events[0][0] != 0
            or any(a[0] >= b[0] for a,b in zip(events, events[1:]))
            or any(e[0] < 0 or e[0] > duration for e in events)):
        raise ValueError('Events must start at zero and increase within the duration')
    integrate = reference_step if method == 'reference' else step
    grid = [n*dt for n in range(count)] + [duration]
    grid_set = set(grid)
    boundaries = sorted(grid_set | {e[0] for e in events})
    state, rows, index = (0.0, 0.0), [(0.0, 0.0, 0.0)], 0
    for left, right in zip(boundaries, boundaries[1:]):
        while index+1 < len(events) and events[index+1][0] <= left:
            index += 1
        _, voltage, load = events[index]
        state = integrate(motor, state, voltage, load, right-left)
        if right in grid_set:
            rows.append((right, *state))
    return rows
