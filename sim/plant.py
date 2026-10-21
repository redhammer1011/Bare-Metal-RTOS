"""plant.py - 1-axis pitch "seesaw" test stand: physics and sensor models.

A rigid bar on a pivot with a motor at each end, like one axis of a quadcopter
clamped to a test stand. Pure Python (no numpy) so it runs anywhere.

Sign convention: motor 1 sits at +arm and motor 2 at -arm, both thrusting
"up", so thrust1 > thrust2 rotates the bar toward positive theta.

    J * theta'' = arm * (thrust1 - thrust2)      motor torque
                + m * g * h * sin(theta)         pivot below the centre of mass:
                                                 tips further away from level
                - b * theta'                     bearing / air damping

The gravity term makes level (theta = 0) an UNSTABLE equilibrium, like a
balanced pencil: left alone, any tilt grows. That is deliberate: a controller
bug shows up as the bar falling over instead of hiding in a stable plant.

Motors are first-order lags: thrust approaches command * max_thrust with
time constant motor_tau. Hard stops at +/- stop_deg end the motion.

Sensors (an IMU mounted on the pivot axis):
    gyro      rate in deg/s + bias (constant start + random walk) + white noise
    accel_x   sin(theta) in g + white noise
    accel_z   cos(theta) in g + white noise
so atan2(accel_x, accel_z) is a drift-free but noisy angle, and integrating the
gyro is a smooth angle that drifts: the trade-off a complementary filter fuses.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass

G = 9.81  # m/s^2


@dataclass(frozen=True)
class PlantParams:
    inertia: float = 0.01          # J, kg m^2 about the pivot
    arm: float = 0.2               # m, pivot to each motor
    max_thrust: float = 2.0        # N per motor at command 1.0
    mass: float = 0.5              # kg
    com_height: float = 0.02       # h, m: centre of mass above the pivot (> 0: unstable)
    damping: float = 0.01          # b, N m s/rad
    motor_tau: float = 0.03        # s, motor/propeller time constant
    stop_deg: float = 45.0         # mechanical end stops
    max_substep: float = 0.001     # s, integration step limit

    def instability_rate(self) -> float:
        """Exponential growth rate (1/s) of a small tilt with equal, settled thrust.

        Linearised: J*th'' + b*th' - m*g*h*th = 0  ->  positive root of
        J*s^2 + b*s - m*g*h = 0.
        """
        J, b, k = self.inertia, self.damping, self.mass * G * self.com_height
        return (-b + math.sqrt(b * b + 4 * J * k)) / (2 * J)


@dataclass(frozen=True)
class SensorParams:
    gyro_bias_dps: float = 1.5           # initial bias, deg/s
    gyro_bias_walk: float = 0.05         # bias random walk, deg/s per sqrt(s)
    gyro_noise_dps: float = 0.3          # white noise std dev per sample, deg/s
    accel_noise_g: float = 0.03          # white noise std dev per axis per sample, g


class Plant:
    def __init__(self, params: PlantParams = PlantParams(), sensors: SensorParams = SensorParams(),
                 theta0_deg: float = 0.0, seed: int = 1):
        self.p = params
        self.s = sensors
        self.rng = random.Random(seed)
        self.t = 0.0
        self.theta = math.radians(theta0_deg)   # rad
        self.omega = 0.0                        # rad/s
        self.thrust1 = 0.0                      # N
        self.thrust2 = 0.0
        self.gyro_bias = sensors.gyro_bias_dps  # deg/s

    # ------------------------------------------------------------ physics
    def _derivatives(self, theta, omega, thrust1, thrust2, cmd1, cmd2):
        p = self.p
        torque = (p.arm * (thrust1 - thrust2)
                  + p.mass * G * p.com_height * math.sin(theta)
                  - p.damping * omega)
        return (omega,
                torque / p.inertia,
                (cmd1 * p.max_thrust - thrust1) / p.motor_tau,
                (cmd2 * p.max_thrust - thrust2) / p.motor_tau)

    def _rk4(self, h: float, cmd1: float, cmd2: float) -> None:
        """One 4th-order Runge-Kutta step of length h."""
        y = (self.theta, self.omega, self.thrust1, self.thrust2)
        k1 = self._derivatives(*y, cmd1, cmd2)
        k2 = self._derivatives(*(yi + h / 2 * ki for yi, ki in zip(y, k1)), cmd1, cmd2)
        k3 = self._derivatives(*(yi + h / 2 * ki for yi, ki in zip(y, k2)), cmd1, cmd2)
        k4 = self._derivatives(*(yi + h * ki for yi, ki in zip(y, k3)), cmd1, cmd2)
        self.theta, self.omega, self.thrust1, self.thrust2 = (
            yi + h / 6 * (a + 2 * b + 2 * c + d) for yi, a, b, c, d in zip(y, k1, k2, k3, k4))

    def step(self, cmd1: float, cmd2: float, dt: float) -> None:
        """Advance dt seconds with motor commands held constant (each clamped to 0..1)."""
        cmd1 = min(max(cmd1, 0.0), 1.0)
        cmd2 = min(max(cmd2, 0.0), 1.0)
        n = max(1, math.ceil(dt / self.p.max_substep - 1e-9))
        h = dt / n
        stop = math.radians(self.p.stop_deg)
        for _ in range(n):
            self._rk4(h, cmd1, cmd2)
            if abs(self.theta) >= stop:                  # inelastic end stop
                self.theta = math.copysign(stop, self.theta)
                if self.omega * self.theta > 0:
                    self.omega = 0.0
        self.gyro_bias += self.s.gyro_bias_walk * math.sqrt(dt) * self.rng.gauss(0.0, 1.0)
        self.t += dt

    # ------------------------------------------------------------ sensors
    def read_sensors(self) -> tuple[float, float, float]:
        """(gyro_rate_dps, accel_x_g, accel_z_g) as the IMU would report them now."""
        s, rng = self.s, self.rng
        gyro = math.degrees(self.omega) + self.gyro_bias + rng.gauss(0.0, s.gyro_noise_dps)
        ax = math.sin(self.theta) + rng.gauss(0.0, s.accel_noise_g)
        az = math.cos(self.theta) + rng.gauss(0.0, s.accel_noise_g)
        return gyro, ax, az

    @property
    def theta_deg(self) -> float:
        return math.degrees(self.theta)

    @property
    def omega_dps(self) -> float:
        return math.degrees(self.omega)
