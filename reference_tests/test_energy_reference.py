import unittest
from energy import energy_balance
from motorlab import Motor
from reference import reference_step


class EnergyReferenceTests(unittest.TestCase):
    def test_independent_reference_conservation_refines(self):
        motor = Motor()
        errors = []
        for dt in (0.01, 0.005, 0.0025):
            # Exact held-input propagation at every sample, evaluated from t=0.
            rows = [(n*dt, *reference_step(motor,(0,0),12,0.02,n*dt),12,0.02)
                    for n in range(round(1/dt)+1)]
            errors.append(abs(energy_balance(motor,rows)['residual_J']))
        self.assertLess(errors[-1], 0.001)
        for coarse,fine in zip(errors,errors[1:]):
            self.assertGreater(coarse/fine, 3.9)
            self.assertLess(coarse/fine, 4.1)

    def test_negative_terminal_power_is_not_clamped(self):
        motor = Motor()
        dt = 0.0001
        rows = [(n*dt, *reference_step(motor,(2,0.5),-1,0,n*dt),-1,0)
                for n in range(101)]
        result = energy_balance(motor,rows)
        self.assertLess(result['electrical_net_J'], 0)
        self.assertGreater(result['copper_loss_J'], 0)
        self.assertLess(result['stored_change_J'], 0)
        self.assertLess(abs(result['residual_J']), 1e-7)
