#!/usr/bin/env python3
"""
Generic plotter: plot any two columns from a CSV file.

Usage:
    python plot_generic.py <csv_file> <x_column> <y_column>

Example:
    python plot_generic.py ../csv/direct_kinematic_validation_4seg.csv time d_real
"""

import sys
import pandas as pd
import matplotlib.pyplot as plt


def main():
    if len(sys.argv) != 4:
        print("Usage: python plot_generic.py <csv_file> <x_column> <y_column>")
        print("\nAvailable columns can be checked with: python plot_generic.py <csv_file> --list")
        sys.exit(1)

    csv_file = sys.argv[1]

    if sys.argv[2] == "--list":
        df = pd.read_csv(csv_file)
        print("Available columns:", list(df.columns))
        sys.exit(0)

    x_col = sys.argv[2]
    y_col = sys.argv[3]

    df = pd.read_csv(csv_file)

    missing = [c for c in [x_col, y_col] if c not in df.columns]
    if missing:
        print(f"Error: column(s) not found: {missing}")
        print("Available columns:", list(df.columns))
        sys.exit(1)

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(df[x_col], df[y_col])
    ax.set_xlabel(x_col)
    ax.set_ylabel(y_col)
    ax.set_title(f"{y_col} vs {x_col}")
    ax.grid(True)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
