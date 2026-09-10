import unittest
from motorlab import Motor
from reference import reference_step, trajectory


class ReferenceTests(unittest.TestCase):
    def test_equilibrium_and_zero_time(self):
        motor = Motor()
        state = motor.equilibrium(12, 0.03)
        self.assertEqual(reference_step(motor, state, 12, 0.03, 0), state)
        for actual, expected in zip(reference_step(motor, state, 12, 0.03, 3), state):
            self.assertAlmostEqual(actual, expected, places=11)

    def test_held_input_composition(self):
        motor, initial = Motor(), (2.0, -0.3)
        first = reference_step(motor, initial, -8, 0.02, 0.137)
        split = reference_step(motor, first, -8, 0.02, 0.863)
        whole = reference_step(motor, initial, -8, 0.02, 1)
        for a,b in zip(split, whole):
            self.assertAlmostEqual(a, b, places=11)

    def test_off_grid_event_matches_explicit_composition(self):
        motor = Motor()
        events = [(0, 12, 0), (0.137, -6, 0.02)]
        actual = trajectory(motor, events, dt=0.1, duration=0.2)[-1][1:]
        before = reference_step(motor, (0, 0), 12, 0, 0.137)
        expected = reference_step(motor, before, -6, 0.02, 0.063)
        for a,b in zip(actual, expected):
            self.assertAlmostEqual(a, b, places=12)
        # Delaying the event to the next grid point must be detectably different.
        delayed = trajectory(motor, [(0, 12, 0), (0.2, -6, 0.02)], dt=0.1, duration=0.2)
        self.assertGreater(abs(actual[0]-delayed[-1][1]), 0.1)

    def test_event_at_output_boundary_uses_previous_input_until_boundary(self):
        motor = Motor()
        rows = trajectory(motor, [(0, 12, 0), (0.1, 0, 0.02)], dt=0.1, duration=0.2)
        before = reference_step(motor, (0, 0), 12, 0, 0.1)
        after = reference_step(motor, before, 0, 0.02, 0.1)
        self.assertEqual(rows[1][1:], before)
        self.assertEqual(rows[2][1:], after)

    def test_rk4_refinement_against_independent_reference(self):
        motor = Motor(resistance=1.3)
        events = [(0, 12, 0), (0.137, 12, 0.03), (0.701, -6, 0.01)]
        errors = []
        for dt in (0.02, 0.01, 0.005):
            exact = trajectory(motor, events, dt=dt)
            numerical = trajectory(motor, events, dt=dt, method='rk4')
            errors.append(max(abs(a[2]-b[2]) for a,b in zip(exact, numerical)))
        self.assertGreater(errors[0]/errors[1], 10)
        self.assertGreater(errors[1]/errors[2], 10)
        self.assertLess(errors[2], 1e-7)

    def test_rejects_ambiguous_schedules(self):
        for events in ([], [()], [(0, 12)], [(0.1, 12, 0)], [(0, 12, 0), (0, 3, 0)],
                       [(0, 12, 0), (2, 0, 0)], [(0, float('nan'), 0)]):
            with self.assertRaises(ValueError):
                trajectory(Motor(), events)
        with self.assertRaises(ValueError):
            trajectory(Motor(), [(0, 0, 0)], dt=0)


if __name__ == '__main__':
    unittest.main()
