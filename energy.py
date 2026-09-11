"""Signed energy accounting for trajectories with held interval inputs."""
import math


def energy_balance(motor, rows):
    """Account for (time, current, speed, voltage, load) rows in SI units.

    Voltage/load in row n are held over [t_n, t_{n+1}); state is continuous.
    Rows must include every input discontinuity. The final row's input values
    do not contribute. Trapezoidal quadrature introduces measurable O(dt²) error.
    """
    if len(rows) < 2 or any(len(r) != 5 or not all(math.isfinite(x) for x in r) for r in rows):
        raise ValueError('At least two finite five-column rows are required')
    if any(b[0] <= a[0] for a,b in zip(rows, rows[1:])):
        raise ValueError('Trajectory times must strictly increase')
    electrical, copper, friction, load_work = [], [], [], []
    for a,b in zip(rows, rows[1:]):
        dt = b[0]-a[0]
        _, i0, w0, voltage, load = a
        _, i1, w1, _, _ = b
        electrical.append(dt*voltage*(i0+i1)/2)
        copper.append(dt*motor.resistance*(i0*i0+i1*i1)/2)
        friction.append(dt*motor.damping*(w0*w0+w1*w1)/2)
        load_work.append(dt*load*(w0+w1)/2)
    stored = lambda r: (motor.inductance*r[1]**2 + motor.inertia*r[2]**2)/2
    result = dict(electrical_net_J=math.fsum(electrical), copper_loss_J=math.fsum(copper),
                  friction_loss_J=math.fsum(friction), load_work_J=math.fsum(load_work),
                  stored_change_J=stored(rows[-1])-stored(rows[0]))
    result['residual_J'] = (result['electrical_net_J']-result['copper_loss_J']
                            -result['friction_loss_J']-result['load_work_J']-result['stored_change_J'])
    return result
