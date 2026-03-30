#!/usr/bin/env python3
"""
Inverse kinematics validation plots.

Generates four subplots from inverse_kinematic_validation_4seg.csv:
  1. Target vs real end-effector position (x, y, z)
  2. IK position error (error_dist) over time — convergence
  3. Joint angles (theta, phi) over time
  4. Cable lengths (c1, c2, c3) over time

Usage:
    python plot_inverse_kinematics.py [csv_file]

Defaults to ../csv/inverse_kinematic_validation_4seg.csv if no argument given.
"""

import sys
import pandas as pd
import matplotlib.pyplot as plt

DEFAULT_CSV = "../csv/inverse_kinematic_validation_4seg.csv"


def main():
    csv_file = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_CSV
    df = pd.read_csv(csv_file)
    t = df["time"]

    fig, axes = plt.subplots(2, 2, figsize=(14, 9))
    fig.suptitle("Inverse Kinematics Validation", fontsize=14)

    # --- Plot 1: Target vs real position ---
    ax = axes[0, 0]
    for axis, color in zip(["x", "y", "z"], ["tab:blue", "tab:orange", "tab:green"]):
        ax.plot(t, df[f"target_{axis}"], color=color, linestyle="--", label=f"{axis} target")
        ax.plot(t, df[f"real_{axis}"], color=color, label=f"{axis} real")
    ax.set_title("End-effector position: target vs real")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Position (m)")
    ax.legend(fontsize=7)
    ax.grid(True)

    # --- Plot 2: IK error over time ---
    ax = axes[0, 1]
    ax.plot(t, df["error_dist"], color="tab:red")
    ax.set_title("IK position error over time (convergence)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Error distance (m)")
    ax.grid(True)

    # --- Plot 3: Joint angles ---
    ax = axes[1, 0]
    ax.plot(t, df["theta"], color="tab:blue", label="theta")
    ax.plot(t, df["phi"], color="tab:orange", label="phi")
    ax.set_title("Joint angles over time")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angle (rad)")
    ax.legend()
    ax.grid(True)

    # --- Plot 4: Cable lengths ---
    ax = axes[1, 1]
    for cable, color in zip([1, 2, 3], ["tab:blue", "tab:orange", "tab:green"]):
        ax.plot(t, df[f"c{cable}"], color=color, label=f"c{cable}")
    ax.set_title("Cable lengths over time")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Length (m)")
    ax.legend()
    ax.grid(True)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
