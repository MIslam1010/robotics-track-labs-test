"""Plots for the C++ track of SBEG 46X Lab 11 (given; plotting stays in Python).

./build/lab_tests writes these CSV files into the folder you run it from (each one only when the functions it
uses work):
  lab11_cpp_odometry.csv        t, s, phi, x, y (truth), phi_odo, x_odo, y_odo, e_pos, e_phi   (dead reckoning) -> A1
  lab11_cpp_ekf.csv             t, phi_est, x_est, y_est, e_pos, e_phi, sigma_phi, sigma_x, sigma_y     (EKF) -> B1
  lab11_cpp_ekf_readings.csv    k, t, id, d2, accepted                (every reading of the log, file order) -> B1
  lab11_cpp_track_<est>.csv     t, phi_d, x_d, y_d, phi, x, y, phi_est, x_est, y_est, v, omega
                                (simulate(p, est, "nonlinear"), est = truth, odometry, ekf)               -> C1

Usage (from the folder that holds the CSV files):   python cpp/plot_results.py  [folder]
Saves lab11_fig1_paths.png, lab11_fig2_odometry_error.png, lab11_fig3_ekf_error.png and lab11_fig4_tracking.png
(Figures 1-4 of the report) and prints the error-growth slopes of Experiment A1 and the 3-sigma fraction of B1.
Figure 5 (Experiment C2) and Tables 1-3: write the values from your own runs (see the handout, C++ track).
"""
import json
import os
import sys

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))


def load(folder, name):
    path = os.path.join(folder, name)
    if not os.path.exists(path):
        print("missing", path, "(run ./build/lab_tests from this folder first)")
        return None
    with open(path) as fh:
        header = fh.readline().strip().split(",")
    D = np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)
    return {h: D[:, k] for k, h in enumerate(header)}


def markers(ax):
    with open(os.path.join(HERE, "..", "data", "wheelchair.json")) as fh:
        lm = np.array(json.load(fh)["landmarks"], float)
    ax.plot(lm[:, 1], lm[:, 2], "k^", ms=7, label="markers")
    for i, x, y in lm:
        ax.annotate(f"{int(i)}", (x, y), textcoords="offset points", xytext=(4, 4))


def main():
    folder = sys.argv[1] if len(sys.argv) > 1 else "."

    d = load(folder, "lab11_cpp_odometry.csv")
    if d is not None:
        fig, ax = plt.subplots(figsize=(7, 6))
        ax.plot(d["x"], d["y"], "k-", lw=1.2, label="truth")
        ax.plot(d["x_odo"], d["y_odo"], "C3--", lw=1.2, label="odometry (nominal model)")
        markers(ax)
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_aspect("equal")
        ax.grid(True)
        ax.legend()
        ax.set_title("Figure 1: true and dead-reckoned paths")
        fig.savefig(os.path.join(folder, "lab11_fig1_paths.png"), dpi=150, bbox_inches="tight")
        print("saved lab11_fig1_paths.png")

        fig, (a1, a2) = plt.subplots(1, 2, figsize=(11, 4))
        m = d["s"] > 1.0
        a1.loglog(d["s"][m], d["e_pos"][m], label="position error")
        a1.loglog(d["s"][m], np.abs(d["e_phi"][m]), label="|heading error|")
        a1.set_xlabel("distance travelled s (m)")
        a1.set_ylabel("error (m, rad)")
        a1.grid(True, which="both")
        a1.legend()
        a2.plot(d["s"], d["e_pos"], label="position error (m)")
        a2.plot(d["s"], d["e_phi"], label="heading error (rad)")
        a2.set_xlabel("distance travelled s (m)")
        a2.grid(True)
        a2.legend()
        fig.suptitle("Figure 2: odometry error growth")
        fig.savefig(os.path.join(folder, "lab11_fig2_odometry_error.png"), dpi=150, bbox_inches="tight")
        print("saved lab11_fig2_odometry_error.png")
        mm = (d["s"] > 5.0) & (d["e_pos"] > 0)
        slope = np.polyfit(np.log(d["s"][mm]), np.log(d["e_pos"][mm]), 1)[0]
        print(f"A1: log-log slope of the position error for s > 5 m: {slope:.2f}")

    d = load(folder, "lab11_cpp_ekf.csv")
    if d is not None:
        rd = load(folder, "lab11_cpp_ekf_readings.csv")
        sig_pos = np.hypot(d["sigma_x"], d["sigma_y"])
        fig, (a1, a2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
        a1.plot(d["t"], 1e3 * d["e_pos"], label="position error")
        a1.plot(d["t"], 3e3 * sig_pos, "k--", lw=0.8, label=r"$3\sqrt{\sigma_x^2+\sigma_y^2}$")
        if rd is not None:
            rej = rd["accepted"] < 0.5
            k = rd["k"][rej].astype(int)
            a1.plot(rd["t"][rej], 1e3 * d["e_pos"][k], "rx", ms=5, label="rejected reading")
        a1.set_ylabel("position error (mm)")
        a1.grid(True)
        a1.legend()
        a2.plot(d["t"], d["e_phi"], label="heading error")
        a2.plot(d["t"], 3 * d["sigma_phi"], "k--", lw=0.8, label=r"$\pm3\sigma_\phi$")
        a2.plot(d["t"], -3 * d["sigma_phi"], "k--", lw=0.8)
        a2.set_xlabel("t (s)")
        a2.set_ylabel("heading error (rad)")
        a2.grid(True)
        a2.legend()
        fig.suptitle("Figure 3: EKF localisation errors with 3-sigma bounds")
        fig.savefig(os.path.join(folder, "lab11_fig3_ekf_error.png"), dpi=150, bbox_inches="tight")
        print("saved lab11_fig3_ekf_error.png")
        print(f"B1: rms / max position error {1e3 * np.sqrt(np.mean(d['e_pos'] ** 2)):.1f} / "
              f"{1e3 * d['e_pos'].max():.1f} mm; heading error inside +-3 sigma for "
              f"{100 * np.mean(np.abs(d['e_phi']) <= 3 * d['sigma_phi']):.1f} % of the samples")

    runs = {est: load(folder, f"lab11_cpp_track_{est}.csv") for est in ("truth", "odometry", "ekf")}
    if any(r is not None for r in runs.values()):
        fig, ax = plt.subplots(figsize=(7, 6))
        ref = next(r for r in runs.values() if r is not None)
        ax.plot(ref["x_d"], ref["y_d"], "k-", lw=2, alpha=0.3, label="reference")
        for (est, r), c in zip(runs.items(), ("C0", "C3", "C2")):
            if r is not None:
                ax.plot(r["x"], r["y"], c, lw=1, label=f"true path, controller fed by {est}")
        markers(ax)
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_aspect("equal")
        ax.grid(True)
        ax.legend(fontsize=8)
        ax.set_title("Figure 4: closed-loop runs, nonlinear law (MR Eq. 13.31)")
        fig.savefig(os.path.join(folder, "lab11_fig4_tracking.png"), dpi=150, bbox_inches="tight")
        print("saved lab11_fig4_tracking.png")


if __name__ == "__main__":
    main()
