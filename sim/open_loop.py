"""open_loop.py - run the plant with fixed motor commands (no controller) and plot.

Three experiments, saved to docs/img/open_loop.png:
  1. Released 0.5 deg off level with equal thrust: the bar falls over, matching
     the linearised two-mode solution until the end stop.
  2. A motor command step: thrust follows with the 30 ms first-order lag.
  3. Bar held at 10 deg for 20 s: the accelerometer angle is noisy but centred
     on 10 deg; the integrated gyro is smooth but drifts away. Fusing the two is
     the complementary filter's job.

Usage:  python sim/open_loop.py
"""

from __future__ import annotations

import math
import statistics
from dataclasses import replace
from pathlib import Path

import matplotlib
matplotlib.use("Agg")                      # write files, no window
import matplotlib.pyplot as plt

from plant import Plant, PlantParams, SensorParams

OUT = Path(__file__).resolve().parent.parent / "docs" / "img" / "open_loop.png"
DT = 0.005                                 # 200 Hz, the rate the control loop will run at


def falls_over():
    params = PlantParams()
    theta0 = 0.5
    plant = Plant(params, theta0_deg=theta0)
    plant.thrust1 = plant.thrust2 = 0.5 * params.max_thrust     # motors already at hover
    # Linear prediction released from rest: theta = theta0 (s2 e^(s1 t) - s1 e^(s2 t)) / (s2 - s1)
    J, b, k = params.inertia, params.damping, params.mass * 9.81 * params.com_height
    root = math.sqrt(b * b + 4 * J * k)
    s1, s2 = (-b + root) / (2 * J), (-b - root) / (2 * J)
    t, sim, lin = [], [], []
    while plant.t < 2.0:
        t.append(plant.t)
        sim.append(plant.theta_deg)
        lin.append(theta0 * (s2 * math.exp(s1 * plant.t) - s1 * math.exp(s2 * plant.t)) / (s2 - s1))
        plant.step(0.5, 0.5, DT)
    hit = next(ti for ti, th in zip(t, sim) if th >= params.stop_deg - 1e-6)
    return t, sim, lin, hit, params.stop_deg


def motor_lag():
    params = PlantParams()
    plant = Plant(params, sensors=SensorParams(0, 0, 0, 0))
    t, cmd, thrust = [], [], []
    # Count integer steps: comparing a float sum of 0.001s against 0.05 can
    # land the command step one sample late (0.001 * 50 != 0.05 exactly).
    for i in range(200):
        command = 0.8 if i >= 50 else 0.2
        t.append(i * 0.001)
        cmd.append(command * params.max_thrust)
        thrust.append(plant.thrust1)
        plant.step(command, command, 0.001)
    return t, cmd, thrust, params.motor_tau


def sensor_drift():
    params = replace(PlantParams(), stop_deg=10.0)
    plant = Plant(params, theta0_deg=10.0, seed=7)               # resting on a 10 deg stop
    t, accel_angle, gyro_angle = [], [], []
    integrated = 10.0                                            # start the gyro integral at the truth
    while plant.t < 20.0:
        gyro, ax, az = plant.read_sensors()
        t.append(plant.t)
        accel_angle.append(math.degrees(math.atan2(ax, az)))
        integrated += gyro * DT
        gyro_angle.append(integrated)
        plant.step(0.0, 0.0, DT)
    return t, accel_angle, gyro_angle


def main():
    t1, sim, lin, hit, stop = falls_over()
    t2, cmd, thrust, tau = motor_lag()
    t3, acc, gyr = sensor_drift()

    fig, ax = plt.subplots(3, 1, figsize=(9, 11))

    ax[0].plot(t1, sim, label="simulation")
    ax[0].plot(t1, lin, "--", label="linearised prediction")
    ax[0].axhline(stop, color="grey", lw=0.8)
    ax[0].set(title=f"1. No control: 0.5° off level falls to the {stop:.0f}° stop in {hit:.2f} s",
              xlabel="time (s)", ylabel="angle (°)", ylim=(-2, stop + 5))
    ax[0].legend()

    ax[1].plot(t2, cmd, label="commanded thrust")
    ax[1].plot(t2, thrust, label="actual thrust")
    ax[1].axvline(0.05 + tau, color="grey", lw=0.8, ls=":")
    ax[1].set(title=f"2. Motor lag: thrust reaches 63% of a step after τ = {tau * 1000:.0f} ms",
              xlabel="time (s)", ylabel="thrust (N)")
    ax[1].legend()

    ax[2].plot(t3, acc, lw=0.5, alpha=0.6, label="accelerometer angle: noisy, no drift")
    ax[2].plot(t3, gyr, lw=1.5, label="integrated gyro: smooth, drifts")
    ax[2].axhline(10.0, color="black", lw=1, label="true angle")
    ax[2].set(title="3. Sensors at rest at 10°: why a complementary filter is needed",
              xlabel="time (s)", ylabel="angle (°)")
    ax[2].legend(loc="upper left")

    fig.tight_layout()
    OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT, dpi=110)

    print(f"saved {OUT}")
    print(f"1. falls from 0.5 deg to the {stop:.0f} deg stop in {hit:.2f} s; "
          f"max sim-vs-linear gap below 5 deg: "
          f"{max(abs(a - b) for a, b in zip(sim, lin) if a < 5):.3f} deg")
    i_tau = 50 + round(tau / 0.001)
    start = thrust[50]                                   # lag from the low command isn't fully settled
    print(f"2. thrust after one tau: {(thrust[i_tau] - start) / (1.6 - start) * 100:.1f}% of the step")
    print(f"3. accel angle: mean {statistics.fmean(acc):.2f} deg, std dev {statistics.stdev(acc):.2f} deg; "
          f"gyro integral after 20 s: {gyr[-1]:.1f} deg (drift {gyr[-1] - 10:.1f} deg)")


if __name__ == "__main__":
    main()
