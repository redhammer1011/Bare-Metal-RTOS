"""plot_closed_loop.py - plot a host closed-loop run of the C++ AttitudeController.

Runs build-native/closed_loop_sim (build it with: cmake --build --preset native)
and saves docs/img/closed_loop_host.png.

Usage:  python sim/plot_closed_loop.py [step_to_deg] [duration_s] [dt_s]
"""

from __future__ import annotations

import csv
import io
import subprocess
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent.parent
EXE = ROOT / "build-native" / ("closed_loop_sim.exe" if sys.platform == "win32" else "closed_loop_sim")
OUT = ROOT / "docs" / "img" / "closed_loop_host.png"


def main() -> int:
    if not EXE.exists():
        print(f"{EXE} not found: run  cmake --build --preset native", file=sys.stderr)
        return 1
    run = subprocess.run([str(EXE), *sys.argv[1:]], capture_output=True, text=True, check=True)
    rows = list(csv.DictReader(io.StringIO(run.stdout)))
    col = {k: [float(r[k]) for r in rows] for k in rows[0]}
    metrics = run.stderr.strip()

    fig, ax = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
    ax[0].plot(col["t"], col["setpoint"], "k--", lw=1, label="setpoint")
    ax[0].plot(col["t"], col["theta"], label="true angle")
    ax[0].plot(col["t"], col["estimate"], lw=0.8, alpha=0.8, label="filter estimate")
    ax[0].set(ylabel="angle (°)", title=f"Host closed loop: {metrics}")
    ax[0].legend()

    ax[1].plot(col["t"], col["motor1"], lw=0.8, label="motor 1")
    ax[1].plot(col["t"], col["motor2"], lw=0.8, label="motor 2")
    ax[1].set(ylabel="command (0..1)")
    ax[1].legend()

    ax[2].plot(col["t"], col["bias_true"], "k--", lw=1, label="true gyro bias")
    ax[2].plot(col["t"], col["bias_est"], label="estimated bias")
    ax[2].set(ylabel="bias (°/s)", xlabel="time (s)")
    ax[2].legend()

    fig.tight_layout()
    OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT, dpi=110)
    print(f"saved {OUT}\n{metrics}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
