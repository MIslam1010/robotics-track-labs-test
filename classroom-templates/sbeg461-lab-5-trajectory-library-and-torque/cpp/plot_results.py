"""Plots for the C++ track of SBEG 461 Lab 5 (given; plotting stays in Python).

./build/lab_tests writes these CSV files into the folder you run it from (each one only when the functions it
uses return results of the right size):
  lab5_cpp_scaling_cubic.csv, lab5_cpp_scaling_quintic.csv,
  lab5_cpp_scaling_trapezoid.csv     t, s, sd, sdd   (T = 1 s; trapezoid v = 1.5, a = 4.5)        -> Step 1 plot
  lab5_cpp_fig_9_7a.csv              t, x, y         (via-point cubic of MR Fig. 9.7a)            -> Step 3 plot
  lab5_cpp_bsplines.csv              r, x_open, y_open, x_clamped, y_clamped (same four points)   -> Step 3 plot
  lab5_cpp_tau_AB.csv                t, r1..r6       (tau_i / tau_max_i, A -> B quintic at T_min) -> Step 5 Q4 plot

Usage (from the folder that holds the CSV files):   python cpp/plot_results.py  [folder]
Saves lab5_step1_scalings.png, lab5_step3_vias.png and lab5_step5_torques.png.
"""
import os
import sys

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def load(folder, name):
    path = os.path.join(folder, name)
    if not os.path.exists(path):
        print("missing", path, "(run ./build/lab_tests from this folder first)")
        return None
    with open(path) as fh:
        header = fh.readline().strip().split(",")
    D = np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)
    return {h: D[:, k] for k, h in enumerate(header)}


def main():
    folder = sys.argv[1] if len(sys.argv) > 1 else "."

    data = {k: load(folder, f"lab5_cpp_scaling_{k}.csv") for k in ("cubic", "quintic", "trapezoid")}
    if any(d is not None for d in data.values()):
        fig, axes = plt.subplots(3, 1, figsize=(7, 8), sharex=True)
        for k, d in data.items():
            if d is None:
                continue
            for ax, col in zip(axes, ("s", "sd", "sdd")):
                ax.plot(d["t"], d[col], label=k)
        for ax, lab in zip(axes, ("s", "ds/dt (1/s)", "d2s/dt2 (1/s^2)")):
            ax.set_ylabel(lab)
            ax.grid(True)
        axes[0].legend()
        axes[0].set_title("Step 1: time scalings, T = 1 s")
        axes[-1].set_xlabel("t (s)")
        fig.tight_layout()
        fig.savefig(os.path.join(folder, "lab5_step1_scalings.png"), dpi=150)
        print("saved lab5_step1_scalings.png")

    via, bs = load(folder, "lab5_cpp_fig_9_7a.csv"), load(folder, "lab5_cpp_bsplines.csv")
    if via is not None or bs is not None:
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.5))
        if via is not None:
            ax1.plot(via["x"], via["y"], label="via-point cubic (MR Fig. 9.7a)")
            ax2.plot(via["t"], via["x"], label="x(t)")
            ax2.plot(via["t"], via["y"], label="y(t)")
        if bs is not None:
            ax1.plot(bs["x_open"], bs["y_open"], "--", label="B-spline, 4 control points")
            ax1.plot(bs["x_clamped"], bs["y_clamped"], ":", label="B-spline, clamped ends")
        ax1.plot([0, 0, 1, 1], [0, 1, 1, 0], "ko")
        ax1.set_xlabel("x")
        ax1.set_ylabel("y")
        ax1.set_aspect("equal")
        ax1.grid(True)
        ax1.legend(fontsize=8)
        ax1.set_title("Step 3: path in the plane")
        ax2.set_xlabel("t (s)")
        ax2.set_ylabel("coordinate")
        ax2.grid(True)
        ax2.legend()
        fig.tight_layout()
        fig.savefig(os.path.join(folder, "lab5_step3_vias.png"), dpi=150)
        print("saved lab5_step3_vias.png")

    tau = load(folder, "lab5_cpp_tau_AB.csv")
    if tau is not None:
        fig, ax = plt.subplots(figsize=(8, 4.5))
        for i in range(1, 7):
            ax.plot(tau["t"], tau[f"r{i}"], label=f"joint {i}")
        ax.axhline(1.0, color="k", lw=0.8)
        ax.axhline(-1.0, color="k", lw=0.8)
        ax.set_xlabel("t (s)")
        ax.set_ylabel("tau_i / tau_max,i")
        ax.set_title("Step 5 Q4: A -> B quintic at T = T_min")
        ax.grid(True)
        ax.legend(ncol=3, fontsize=8)
        fig.tight_layout()
        fig.savefig(os.path.join(folder, "lab5_step5_torques.png"), dpi=150)
        print("saved lab5_step5_torques.png")


if __name__ == "__main__":
    main()
