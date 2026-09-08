import math
import json
from pathlib import Path
import unittest
from motorlab import Motor, PI, Scenario, simulate, simulate_sampled, step


class PhysicsTests(unittest.TestCase):
    def test_equilibrium_solves_both_equations_with_load(self):
        motor = Motor()
        for voltage, load in ((12, 0), (12, 0.02), (-12, -0.02)):
            state = motor.equilibrium(voltage, load)
            for derivative in motor.derivative(state, voltage, load):
                self.assertAlmostEqual(derivative, 0, places=12)

    def test_open_loop_converges_to_analytic_equilibrium(self):
        motor, state = Motor(), (0.0, 0.0)
        for _ in range(2000):
            state = step(motor, state, 12, 0.02, 0.005)
        for actual, expected in zip(state, motor.equilibrium(12, 0.02)):
            self.assertAlmostEqual(actual, expected, places=6)

    def test_electromechanical_energy_balance(self):
        motor = Motor()
        current, speed, voltage, load = 2.3, 0.7, 8.0, 0.015
        di, dw = motor.derivative((current, speed), voltage, load)
        stored_power = motor.inductance*current*di + motor.inertia*speed*dw
        net_power = voltage*current - motor.resistance*current**2 - motor.damping*speed**2 - load*speed
        self.assertAlmostEqual(stored_power, net_power, places=12)

    def test_rk4_convergence_against_decoupled_rl_solution(self):
        # Test the integrator against an independent analytic two-mode response
        # using a motor-like derivative with a single known exponential.
        class RL:
            def derivative(self, state, voltage, load):
                return (2*(voltage-state[0]), 0.0)
        errors = []
        for dt in (0.1, 0.05):
            state = (0.0, 0.0)
            for _ in range(round(1/dt)):
                state = step(RL(), state, 1, 0, dt)
            errors.append(abs(state[0] - (1-math.exp(-2))))
        self.assertGreater(errors[0]/errors[1], 14)


class ControllerTests(unittest.TestCase):
    def test_unreachable_command_does_not_wind_up(self):
        pi = PI(20, 30)
        for _ in range(1000):
            self.assertEqual(pi.update(10, 0.005), 24)
        self.assertEqual(pi.integral, 0)
        self.assertLess(pi.update(-1, 0.005), 0)

    def test_regulates_and_rejects_load(self):
        rows = simulate((20, 40), Scenario("disturbance", load=0.03), duration=8)
        self.assertLess(abs(rows[-1][2]-1), 0.002)
        self.assertTrue(all(abs(r[3]) <= 24 for r in rows))

    def test_sample_rate_refinement(self):
        scenario = Scenario("test", load=0.03)
        coarse = simulate((20, 40), scenario, dt=0.005)
        fine = simulate((20, 40), scenario, dt=0.0025)
        self.assertLess(max(abs(a[2]-b[2]) for a, b in zip(coarse, fine[::2])), 0.01)

    def test_invalid_parameters(self):
        with self.assertRaises(ValueError):
            Motor(inductance=0)
        with self.assertRaises(ValueError):
            PI(-1, 2)
        with self.assertRaises(ValueError):
            simulate((1, 1), Scenario("bad"), dt=0)


class SampledSimulationTests(unittest.TestCase):
    def test_zero_noise_preserves_original_simulation(self):
        scenario = Scenario('same', load=0.03)
        self.assertEqual(simulate((20, 40), scenario),
                         simulate_sampled((20, 40), scenario, plant_dt=0.005))

    def test_voltage_is_held_between_controller_updates(self):
        rows = simulate_sampled((20, 40), Scenario('hold'), control_dt=0.02, duration=0.1)
        for start in range(0, 100, 20):
            self.assertEqual(len(set(r[3] for r in rows[start:start+20])), 1)
        self.assertNotEqual(rows[0][3], rows[20][3])

    def test_seed_reproducibility_and_voltage_bounds(self):
        # Run past initial voltage saturation, which masks sensor differences.
        kwargs = dict(noise_std=0.05, duration=1.0)
        a = simulate_sampled((60, 130), Scenario('noise'), seed=1, **kwargs)
        self.assertEqual(a, simulate_sampled((60, 130), Scenario('noise'), seed=1, **kwargs))
        self.assertNotEqual(a, simulate_sampled((60, 130), Scenario('noise'), seed=2, **kwargs))
        self.assertTrue(all(abs(r[3]) <= 24 for r in a))

    def test_noise_does_not_directly_modify_plant_state(self):
        # Zero gains isolate the sensor from actuator dynamics: noise cannot
        # change any physical state when no voltage or load is applied.
        rows = simulate_sampled((0, 0), Scenario('isolated'), noise_std=10, duration=0.1)
        self.assertTrue(all(r[1] == 0 and r[2] == 0 and r[3] == 0 for r in rows))

    def test_grid_and_noise_validation(self):
        for kwargs in ({'control_dt': 0.0015}, {'duration': 0.0015},
                       {'control_dt': 0.0001}, {'noise_std': -1}, {'noise_std': float('nan')}):
            with self.assertRaises(ValueError):
                simulate_sampled((1, 1), Scenario('invalid'), **kwargs)
        with self.assertRaises(ValueError):
            simulate_sampled((1, 1), Scenario('unaligned', load_at=0.0025))

    def test_tuned_gains_plant_refinement_with_identical_sensor_samples(self):
        gains = json.loads((Path(__file__).parent / 'results/summary.json').read_text())['tuned_gains']
        scenario = Scenario('refinement', load=0.03)
        kwargs = dict(control_dt=0.05, noise_std=0.05, seed=4, duration=3)
        coarse = simulate_sampled(gains, scenario, plant_dt=0.001, **kwargs)
        fine = simulate_sampled(gains, scenario, plant_dt=0.0005, **kwargs)
        self.assertLess(max(abs(a[2]-b[2]) for a,b in zip(coarse, fine[::2])), 1e-7)


if __name__ == "__main__":
    unittest.main()
