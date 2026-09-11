import unittest
from energy import energy_balance
from motorlab import Motor, Scenario, simulate_sampled


class EnergyTests(unittest.TestCase):
    def test_equilibrium_power_balance_with_load(self):
        motor = Motor()
        i,w = motor.equilibrium(12, 0.03)
        result = energy_balance(motor, [(0,i,w,12,0.03), (2,i,w,12,0.03)])
        self.assertAlmostEqual(result['electrical_net_J'], 24*i, places=12)
        self.assertAlmostEqual(result['residual_J'], 0, places=11)
        self.assertEqual(result['stored_change_J'], 0)

    def test_voltage_and_load_are_left_held_at_jumps(self):
        # Constant states isolate quadrature semantics, not a physical solution.
        rows = [(0,2,3,5,0.1), (1,2,3,-4,-0.2), (2,2,3,999,999)]
        result = energy_balance(Motor(), rows)
        self.assertAlmostEqual(result['electrical_net_J'], 2)
        self.assertAlmostEqual(result['load_work_J'], -0.3)

    def test_free_decay_dissipates_stored_energy(self):
        from motorlab import step
        motor, state, rows = Motor(), (2.0, 0.5), []
        for n in range(2001):
            rows.append((n*0.001, *state, 0, 0))
            if n < 2000:
                state = step(motor, state, 0, 0, 0.001)
        result = energy_balance(motor, rows)
        self.assertEqual(result['electrical_net_J'], 0)
        self.assertLess(result['stored_change_J'], 0)
        self.assertGreater(result['copper_loss_J'], 0)
        self.assertLess(abs(result['residual_J']), 1e-5)

    def test_sampled_controller_energy_refinement(self):
        scenario = Scenario('load', load=0.03)
        errors = []
        for dt in (0.002, 0.001):
            rows = simulate_sampled((20,40), scenario, control_dt=0.01, plant_dt=dt)
            errors.append(abs(energy_balance(scenario.motor, rows)['residual_J']))
        self.assertGreater(errors[0]/errors[1], 3.8)
        self.assertLess(errors[0]/errors[1], 4.2)

    def test_rejects_nonfinite_and_nonincreasing_rows(self):
        for rows in ([], [(0,0,0,0,0)], [(0,0,0,0,0),(0,0,0,0,0)],
                     [(0,0,0,0,0),(1,float('nan'),0,0,0)]):
            with self.assertRaises(ValueError):
                energy_balance(Motor(), rows)
