"""test_plant.py - checks the plant model against hand-derived physics.

Run from the repo root:  python -m unittest discover -s sim
"""

import math
import statistics
import unittest
from dataclasses import replace

from plant import G, Plant, PlantParams, SensorParams

QUIET = SensorParams(gyro_bias_dps=0.0, gyro_bias_walk=0.0, gyro_noise_dps=0.0, accel_noise_g=0.0)


def run(plant, cmd1, cmd2, seconds, dt=0.005):
    for _ in range(round(seconds / dt)):
        plant.step(cmd1, cmd2, dt)


def settle_motors(plant, cmd1, cmd2):
    """Start the motors already at their commanded thrust (skip the lag)."""
    plant.thrust1 = cmd1 * plant.p.max_thrust
    plant.thrust2 = cmd2 * plant.p.max_thrust


class TestDynamics(unittest.TestCase):
    def test_level_with_equal_thrust_is_an_equilibrium(self):
        plant = Plant(sensors=QUIET)
        run(plant, 0.5, 0.5, 2.0)
        self.assertEqual(plant.theta, 0.0)

    def test_more_thrust_on_motor1_tilts_positive(self):
        plant = Plant(sensors=QUIET)
        run(plant, 0.6, 0.4, 0.1)
        self.assertGreater(plant.theta_deg, 0.0)

    def test_small_tilt_grows_at_the_linearised_rate(self):
        # Released from rest, the motion mixes a growing mode e^(+s1 t) and a
        # decaying one e^(s2 t), which skews a growth measurement. Starting with
        # omega = s1 * theta puts the bar purely on the growing mode.
        params = PlantParams()
        rate = params.instability_rate()
        plant = Plant(params, QUIET, theta0_deg=0.01)
        plant.omega = rate * plant.theta
        settle_motors(plant, 0.5, 0.5)
        theta_start = plant.theta
        run(plant, 0.5, 0.5, 1.0)                     # grows ~14x, still under 1 degree
        measured = math.log(plant.theta / theta_start) / plant.t
        self.assertAlmostEqual(measured, rate, delta=1e-3 * rate)

    def test_constant_torque_gives_quadratic_angle(self):
        params = replace(PlantParams(), com_height=0.0, damping=0.0, stop_deg=89.0)
        plant = Plant(params, QUIET)
        settle_motors(plant, 0.6, 0.4)
        run(plant, 0.6, 0.4, 0.3)
        alpha = params.arm * 0.2 * params.max_thrust / params.inertia     # rad/s^2
        self.assertAlmostEqual(plant.theta, 0.5 * alpha * plant.t ** 2, delta=1e-9)

    def test_motor_reaches_63_percent_after_one_time_constant(self):
        params = PlantParams()
        plant = Plant(params, QUIET)
        plant.step(1.0, 0.0, params.motor_tau)
        self.assertAlmostEqual(plant.thrust1 / params.max_thrust, 1 - math.exp(-1), delta=1e-6)

    def test_damping_decays_spin_exponentially(self):
        params = replace(PlantParams(), com_height=0.0, stop_deg=89.0)
        plant = Plant(params, QUIET)
        plant.omega = 1.0
        run(plant, 0.0, 0.0, 0.5)
        self.assertAlmostEqual(plant.omega, math.exp(-params.damping / params.inertia * 0.5), delta=1e-6)

    def test_falls_to_the_end_stop_and_stays(self):
        plant = Plant(sensors=QUIET, theta0_deg=1.0)
        run(plant, 0.5, 0.5, 3.0)
        self.assertAlmostEqual(plant.theta_deg, plant.p.stop_deg)
        self.assertEqual(plant.omega, 0.0)

    def test_commands_are_clamped_to_0_1(self):
        a, b = Plant(sensors=QUIET), Plant(sensors=QUIET)
        run(a, 5.0, -3.0, 0.1)
        run(b, 1.0, 0.0, 0.1)
        self.assertEqual(a.theta, b.theta)

    def test_result_does_not_depend_on_step_size(self):
        coarse, fine = Plant(sensors=QUIET, theta0_deg=2.0), Plant(sensors=QUIET, theta0_deg=2.0)
        run(coarse, 0.7, 0.3, 0.2, dt=0.010)
        run(fine, 0.7, 0.3, 0.2, dt=0.001)
        self.assertAlmostEqual(coarse.theta, fine.theta, delta=1e-9)


class TestSensors(unittest.TestCase):
    def test_gyro_at_rest_reads_bias_plus_noise(self):
        sensors = replace(SensorParams(), gyro_bias_walk=0.0)
        plant = Plant(sensors=sensors, seed=3)
        readings = [plant.read_sensors()[0] for _ in range(20000)]
        self.assertAlmostEqual(statistics.fmean(readings), sensors.gyro_bias_dps, delta=0.01)
        self.assertAlmostEqual(statistics.stdev(readings), sensors.gyro_noise_dps, delta=0.01)

    def test_accel_angle_is_unbiased_and_noisy(self):
        params = replace(PlantParams(), stop_deg=10.0)
        plant = Plant(params, theta0_deg=10.0, seed=4)       # resting on the end stop
        angles = []
        for _ in range(20000):
            _, ax, az = plant.read_sensors()
            angles.append(math.degrees(math.atan2(ax, az)))
        self.assertAlmostEqual(statistics.fmean(angles), 10.0, delta=0.05)
        self.assertGreater(statistics.stdev(angles), 1.0)    # ~1.7 deg: too noisy to use alone

    def test_gyro_bias_drifts_as_a_random_walk(self):
        sensors = SensorParams()
        finals = []
        for seed in range(400):
            plant = Plant(sensors=sensors, seed=seed)
            for _ in range(100):
                plant.step(0.0, 0.0, 0.01)                    # 1 s
            finals.append(plant.gyro_bias - sensors.gyro_bias_dps)
        self.assertAlmostEqual(statistics.stdev(finals), sensors.gyro_bias_walk * 1.0, delta=0.01)

    def test_same_seed_gives_identical_runs(self):
        a, b = Plant(theta0_deg=1.0, seed=9), Plant(theta0_deg=1.0, seed=9)
        for _ in range(50):
            a.step(0.55, 0.45, 0.005)
            b.step(0.55, 0.45, 0.005)
            self.assertEqual(a.read_sensors(), b.read_sensors())


class TestParams(unittest.TestCase):
    def test_instability_rate_solves_the_characteristic_equation(self):
        p = PlantParams()
        s = p.instability_rate()
        residual = p.inertia * s * s + p.damping * s - p.mass * G * p.com_height
        self.assertAlmostEqual(residual, 0.0, delta=1e-12)
        # By hand: (-0.01 + sqrt(0.0001 + 4 * 0.01 * 0.0981)) / 0.02 = 2.672 /s,
        # so a tilt grows e-fold every ~0.37 s. (Ignoring damping, sqrt(mgh/J)
        # would suggest 0.32 s: damping slows the fall noticeably.)
        self.assertAlmostEqual(s, 2.672, delta=0.001)


if __name__ == "__main__":
    unittest.main()
