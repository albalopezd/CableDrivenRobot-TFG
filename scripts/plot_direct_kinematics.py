#!/usr/bin/env python3
"""
Direct kinematics validation plots.

Generates four subplots from direct_kinematic_validation_4seg.csv:
  1. Real vs theoretical end-effector position (x, y, z)
  2. Position error (d_real) over time
  3. Total rotation angle (theta_total) over time
  4. Segment bending parameters (b1x, b2x, b3x) over time

Usage:
    python plot_direct_kinematics.py [csv_file]

Defaults to ../csv/direct_kinematic_validation_4seg.csv if no argument given.
"""

import sys
import pandas as pd
import matplotlib.pyplot as plt

DEFAULT_CSV = "../csv/direct_kinematic_validation_4seg.csv"


def main():
    csv_file = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_CSV
    df = pd.read_csv(csv_file)
    t = df["time"]

    fig, axes = plt.subplots(2, 2, figsize=(14, 9))
    fig.suptitle("Direct Kinematics Validation", fontsize=14)

    # --- Plot 1: Real vs theoretical position ---
    ax = axes[0, 0]
    for axis, color in zip(["x", "y", "z"], ["tab:blue", "tab:orange", "tab:green"]):
        ax.plot(t, df[f"{axis}_real"], color=color, label=f"{axis} real")
        ax.plot(t, df[f"{axis}_theo"], color=color, linestyle="--", label=f"{axis} theo")
    ax.set_title("End-effector position: real vs theoretical")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Position (m)")
    ax.legend(fontsize=7)
    ax.grid(True)

    # --- Plot 2: Position error over time ---
    ax = axes[0, 1]
    ax.plot(t, df["d_real"], color="tab:red")
    ax.set_title("Position error over time")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Error d_real (m)")
    ax.grid(True)

    # --- Plot 3: Total rotation angle ---
    ax = axes[1, 0]
    ax.plot(t, df["theta_total"], color="tab:purple")
    ax.set_title("Total rotation angle (theta_total)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angle (rad)")
    ax.grid(True)

    # --- Plot 4: Segment bending (x component per segment) ---
    ax = axes[1, 1]
    for seg, color in zip([1, 2, 3], ["tab:blue", "tab:orange", "tab:green"]):
        ax.plot(t, df[f"b{seg}x"], color=color, label=f"b{seg}x")
    ax.set_title("Segment bending — x component")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Bending (m)")
    ax.legend()
    ax.grid(True)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
