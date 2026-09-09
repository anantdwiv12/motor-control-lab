import math
import unittest

from design import pole_cancellation_pi
from motorlab import Motor, Scenario, simulate_sampled


class DesignTests(unittest.TestCase):
    def test_full_characteristic_polynomial_factorization(self):
        # Check (s+a)(s²+b*s+g*Kp), including the hidden stable mode,
        # against s*D(s)+g*(Kp*s+Ki) for multiple physical plants.
        for motor in (Motor(), Motor(resistance=1.3), Motor(inertia=0.02)):
            kp, ki = pole_cancellation_pi(motor)
            a = ki/kp
            total = motor.resistance/motor.inductance + motor.damping/motor.inertia
            product = (motor.resistance*motor.damping+motor.k**2)/(motor.inductance*motor.inertia)
            b = total-a
            g = motor.k/(motor.inductance*motor.inertia)
            self.assertGreater(a, 0)
            self.assertGreater(b, 0)
            self.assertAlmostEqual(a*b+g*kp, product+g*kp, places=10)
            self.assertAlmostEqual(b/(2*math.sqrt(g*kp)), 0.8, places=12)
            for s in (-0.5, 1.0, complex(-1, 3)):
                self.assertAlmostEqual(abs((s+a)*(s*s+b*s+g*kp)
                                          - (s*(s*s+total*s+product)+g*(kp*s+ki))), 0, places=10)

    def test_small_signal_response_matches_analytic_second_order_step(self):
        kp, ki = pole_cancellation_pi()
        motor = Motor()
        wn = math.sqrt(motor.k/(motor.inductance*motor.inertia)*kp)
        zeta = 0.8
        wd = wn*math.sqrt(1-zeta*zeta)
        rows = simulate_sampled((kp, ki), Scenario('linear', target=0.05),
                                control_dt=0.0001, plant_dt=0.0001, duration=2)
        self.assertLess(max(abs(r[3]) for r in rows), 24)
        def exact(t):
            return 0.05*(1-math.exp(-zeta*wn*t)*(math.cos(wd*t)
                         + zeta/math.sqrt(1-zeta*zeta)*math.sin(wd*t)))
        self.assertLess(max(abs(r[2]-exact(r[0])) for r in rows), 2e-5)

    def test_rejects_unsupported_design_inputs(self):
        for zeta in (0, -1, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                pole_cancellation_pi(damping_ratio=zeta)
        with self.assertRaises(ValueError):
            pole_cancellation_pi(Motor(resistance=1, inductance=1, inertia=1, damping=1, k=1))


if __name__ == '__main__':
    unittest.main()
