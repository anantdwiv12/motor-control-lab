"""A nominal-model PI design using stable slow-pole cancellation."""
import math
from motorlab import Motor


def pole_cancellation_pi(motor=Motor(), damping_ratio=0.8):
    """Return (Kp, Ki) for a chosen reduced continuous-time damping ratio.

    This design requires real plant poles. It does not certify the sampled,
    saturated implementation or compensate for parameter uncertainty.
    """
    if not math.isfinite(damping_ratio) or damping_ratio <= 0:
        raise ValueError('Damping ratio must be finite and positive')
    total = motor.resistance / motor.inductance + motor.damping / motor.inertia
    product = (motor.resistance * motor.damping + motor.k**2) / (motor.inductance * motor.inertia)
    discriminant = total**2 - 4*product
    if discriminant < 0:
        raise ValueError('This design requires real plant poles')
    fast = (total + math.sqrt(discriminant)) / 2
    slow = product / fast  # avoids subtractive cancellation for separated poles
    gain = motor.k / (motor.inductance * motor.inertia)
    kp = (fast / (2*damping_ratio))**2 / gain
    return kp, slow*kp
