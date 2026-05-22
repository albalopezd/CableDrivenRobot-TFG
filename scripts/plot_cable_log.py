#!/usr/bin/env python3
"""
Plot data from data_logger.py CSV output.

Usage:
    python plot_cable_log.py <csv_file>
"""

import sys
import pandas as pd
import matplotlib.pyplot as plt


def main():
    if len(sys.argv) != 2:
        print("Usage: python plot_cable_log.py <csv_file>")
        sys.exit(1)

    df = pd.read_csv(sys.argv[1])
    t = df["t"]

    fig, axes = plt.subplots(3, 3, figsize=(14, 9))
    fig.suptitle("Cable Robot — Control Log", fontsize=13)

    colors = {"des": "#1f77b4", "act": "#ff7f0e", "err": "#d62728"}
    cables = [("c1", "Cable 1"), ("c2", "Cable 2"), ("c3", "Cable 3")]

    # Fila 0: deseado vs actual por cable
    for col, (key, label) in enumerate(cables):
        ax = axes[0, col]
        ax.plot(t, df[f"des_{key}"], label="Deseado", color=colors["des"])
        ax.plot(t, df[f"act_{key}"], label="Actual",  color=colors["act"], linestyle="--")
        ax.set_title(label)
        ax.set_ylabel("Longitud (m)")
        ax.legend(fontsize=7)
        ax.grid(True)

    # Fila 1: error por cable
    for col, (key, label) in enumerate(cables):
        ax = axes[1, col]
        ax.plot(t, df[f"err_{key}"], color=colors["err"])
        ax.axhline(0, color="black", linewidth=0.8, linestyle=":")
        ax.set_title(f"Error {label}")
        ax.set_ylabel("Error (m)")
        ax.grid(True)

    # Fila 2: pose del extremo
    for col, (axis, label) in enumerate([("x", "Pose X"), ("y", "Pose Y"), ("z", "Pose Z")]):
        ax = axes[2, col]
        ax.plot(t, df[f"pose_{axis}"], color="#2ca02c")
        ax.set_title(label)
        ax.set_xlabel("Tiempo (s)")
        ax.set_ylabel("Posición (m)")
        ax.grid(True)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
