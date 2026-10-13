---
title: "Lab 6 — Time-optimal time scaling: TOPP (toppra) vs quintic, and a hardware run on the course arm"
subtitle: "SBEG 461 Robot Kinematics and Dynamics — Week 6"
---

# Overview

| Item | Detail |
|---|---|
| Session | Session 4, 50 min in class (the first 10 min are the hardware refresher and the haptic-device induction, see Part E) + up to 2 h to finish (due before the Week 7 lab) |
| CLOs | **461-4** (generate time-optimal trajectories that respect kinematic and torque limits); 461-2 (verified dynamics code, here reused inside a planner) |
| Files (your Lab code folder) | `starter.py` (you edit), `test_lab.py` (auto-checks), `course_arm.py` (the local model file of the course arm and its safety-checklist values; numbers identical to Course resources: Course robot models and hardware safety checklist), `data/lab6_task.json` (task), `data/sim_servo_log.csv` (**synthetic** log for the simulation fallback) |
| Reference | `modern_robotics` (`InverseDynamics`), `toppra` 0.6 (TOPP-RA solver), closed-form results of Workshop 6 |
| Reading | MR §9.4, printed pp. 338–347 (Eqs. 9.30–9.39, the algorithm on pp. 343–344, caveats §9.4.4); LaValle §14.6.3, printed pp. 846–854 |
| Assessment | Auto-checked (`test_lab.py`) + results table + phase-plane figures + short reflection (not a formal report) |
| AI use | Allowed with disclosure (see Reflection) |
| Tier | 0 (simulation) for everyone; Tier 1 (course arm): one TA-supervised station run per team (the Week 6 hardware run, Part E), with a simulation fallback |

Your functions go into your `sbeg461_dynamics` library as `timescaling.py`. Lab 5's trajectory library gave you polynomial time scalings; today you add the time-optimal one. Lab 8 (RRT + smoothing + time scaling) and the project's Scenarios 3 and 5 ("time-optimal scaling under the torque and joint-speed limits") reuse `topp`, `quintic_min_time`, `command_table` and `check_command`. `path_dynamics` reuses the RNEA from Labs 2–4 through `modern_robotics`.

**Application context.**
*Biomedical:* a patient-positioning or rehabilitation robot must move **fast but safely**: no faster than its motors allow, and no faster than the patient tolerates. Today's task is a mock-up of a positioning move with the course arm carrying a 0.1 kg foam phantom.
*Defence (protective/logistic):* an EOD robot arm moving a suspect package, or a gantry moving a demining sensor head (Workshop 6, Problem 3), needs the same planner. Its constraints are torque limits, speed limits near people, and a margin for the controller.
All robot data are **illustrative values** (see `lab/course_arm.py` and `lab/data/lab6_task.json`); they are not the data of any product.

# Learning objectives

By the end of the lab you can:

1. Compute the path-constrained dynamics $m(s)$, $c(s)$, $g(s)$ (MR Eq. 9.33) with three RNEA calls, and from them the motion cone $[L, U]$ and the velocity limit curve (MR Eqs. 9.36–9.37).
2. Implement the single-switch bang-bang time scaling (MR §9.4.1, Fig. 9.12a), recognise when it fails (MR Fig. 9.12b), and compare it with `toppra`.
3. Compare time-optimal scaling with the fastest feasible quintic on the course arm under torque, speed and acceleration limits, and explain the difference in time, jerk and actuator use.
4. Produce a safe hardware command file (derated speed, joint-limit margins, slow first run), run it on the course arm under supervision or in the simulation fallback, and quantify tracking lag and error.

# Pre-lab check (complete before the session; 5 questions)

1. Write $\dot\theta$ and $\ddot\theta$ in terms of $\theta'$, $\theta''$, $\dot s$, $\ddot s$ (MR Eqs. 9.1–9.2).
2. Show that $m(s) = $ `InverseDynamics(θ, 0, θ', g = 0)` and $c(s) = $ `InverseDynamics(θ, θ', θ'', g = 0)`.
3. Why is $d(\dot s^2)/ds = 2\ddot s$? Why is it convenient to integrate $x = \dot s^2$ rather than $\dot s$?
4. For the MR 2R arm on $\theta(s) = s(\pi/2, \pi/2)$, what is $c(0)$? Why?
5. The course arm's rated joint speed is 160°/s. What is the speed limit (rad/s) for your hardware plan (50 %) and for the first run (25 %)?

*Answers (for self-check):* 1. $\dot\theta = \theta'\dot s$, $\ddot\theta = \theta'\ddot s + \theta''\dot s^2$. 2. The RNEA returns $M\ddot\theta + c(\theta,\dot\theta) + g$. With $\dot\theta = 0$, $\ddot\theta = \theta'$ and no gravity it returns $M\theta' = m(s)$. With $\dot\theta = \theta'$, $\ddot\theta = \theta''$ and no gravity it returns $M\theta'' + \theta'^{\mathsf T}\Gamma\theta' = c(s)$ (MR Eq. 9.32). 3. By the chain rule $\frac{d}{ds}\dot s^2 = 2\dot s\frac{d\dot s}{ds} = 2\dot s\frac{\ddot s}{\dot s} = 2\ddot s$. The ODE in $x$ has no $1/\dot s$, so it is well behaved at $\dot s = 0$ (start and end). 4. $c = 0$, because every term of $c(\theta,\dot\theta)$ has a factor $\sin\theta_2 = 0$ at $\theta_2 = 0$. 5. $0.5\times2.793 = 1.396$ rad/s and $0.25\times2.793 = 0.698$ rad/s.

# Setup

```bash
cd "<your Lab code folder>"   # or your GitHub Classroom repo
python -c "import numpy, modern_robotics, toppra; print('ok')"
LAB_IMPL=starter python -m pytest -q test_lab.py   # 17 failed, 1 passed, 4 errors (NotImplementedError) is expected
```

On Windows (PowerShell) use `$env:LAB_IMPL="starter"; python -m pytest -q test_lab.py`. If `toppra` is missing: `pip install toppra` (it is in the course Docker image and the JupyterHub fallback).

The provided parts of `starter.py` are: `StraightLinePath`, `SemicirclePath`, `waypoint_path` (a natural cubic spline through joint-space waypoints), the models `mr_2r_model`, `elbow_exo_model` (Workshop 6 P2), `gantry_model` (Workshop 6 P3) and `course_arm_model`, `inverse_dynamics`, `quintic_s`, `quintic_traj`, `peak_usage`, `write_log`, `load_log` and `simulate_servo`. Paths are called like `toppra` paths: `path(s, 0)` is $\theta$, `path(s, 1)` is $\theta'$, `path(s, 2)` is $\theta''$.

# Procedure

## Part A — Path dynamics and the motion cone (12 min)

1. Implement `path_dynamics(model, path, s)` with three calls of `inverse_dynamics` (pre-lab Q2).
2. Implement `accel_limits(m, c, g, sdot, tau_min, tau_max)` (MR Eqs. 9.36–9.37). Swap the bounds when $m_i < 0$. A **zero-inertia** joint ($|m_i| <$ `EPS_M`) gives no bound on $\ddot s$, but makes the state inadmissible if $c_i\dot s^2 + g_i$ is outside the torque range (MR §9.4.4).
3. Implement `velocity_limit(...)`: the kinematic bound $\min_i\dot\theta_{i,\max}/|\theta'_i|$, then bisection on $\dot s$ for the largest admissible value.

**Checkpoint A.** `pytest -q -k "path_dynamics or accel_limits or velocity_limit"` passes. Your code reproduces the hand results of Workshop 6: $m(0.5) = (9.615,\ 4.252)$ and $\dot s_{\lim}(0.5) = 2.155$ for the 2R arm (P4); $\dot s_{\lim} = 1.007,\ 1.293,\ 1.162$ at $s = 0,\ 0.25,\ 0.5$ for the gantry (P3); the cone $[-1.6,\ 1.2]$ of Quiz 5 Q2. On the course-arm task path at $s = 0.5$ (the middle waypoint): $\theta' = (1.5, -0.2, 0.2, 0, 0, 0)$, $g = (0, -0.887, -0.382, -0.130, 0, 0)$ N m, and $\dot s_{\lim} = 5.59$ from 60 % torque alone but $0.931$ with the 50 % speed limit. On this light arm the **speed** limit binds long before the torque limit.

**Figure A.** Plot the velocity limit curve of the gantry (P3) for $s\in[0,1]$ and mark the dip at the zero-inertia point $s = 0.5$.

## Part B — Bang-bang and TOPP (12 min)

4. Implement `bang_bang(model, path, tau_min, tau_max, N)` exactly as its docstring says. Compute $F$ backward on $L$ from $(1,0)$, stopping where it meets the velocity limit curve. Then integrate $A$ forward on $U$ from $(0,0)$ (Heun steps in $x = \dot s^2$) until it crosses $F$, and raise `RuntimeError` if $A$ reaches the velocity limit curve first. $T = \sum 2h/(\dot s_k + \dot s_{k+1})$.
5. Implement `topp(model, path, tau_max, qdot_max, qddot_max, N)` with `toppra`: a `JointTorqueConstraint` built on `inverse_dynamics`, optional `JointVelocityConstraint` and `JointAccelerationConstraint`, `algo.TOPPRA(..., parametrizer="ParametrizeConstAccel")`, then `compute_trajectory(0, 0)`. Return the duration, the trajectory and `problem_data.sd_vec` (the optimal $\dot s$ on the grid).

**Checkpoint B.**
* Elbow exoskeleton (Workshop 6 P2, $|\tau|\le10$ N m): $s^* = 0.7452$, $\dot s^* = 4.517$, $T = 0.3885$ s, all in closed form. With `qdot_max=[3.0]`, toppra gives $T \approx 0.7465$ s (exact value 0.7466 s).
* MR 2R arm ($|\tau|\le(20, 10)$): `bang_bang` gives $T = 1.368$ s with $s^* = 0.479$; `topp` gives 1.370 s.
* Gantry semicircle: `bang_bang` **raises** (the forward curve hits the velocity limit curve near $s = 0.41$); `topp` gives $T = 1.166$ s and touches the limit at $s = 0.5$.
* Torque scaling: `topp` with $0.6\tau_{\max}$ on the 2R arm takes $1.368/\sqrt{0.6} = 1.77$ s (Workshop 6 P5).

**Figure B.** For the 2R arm and the gantry, plot on one $(s,\dot s)$ figure each: the velocity limit curve, your $A$ and $F$ curves (2R only), and toppra's $\dot s(s)$. Add 6–8 motion cones as short arrows $(\dot s, L)$ and $(\dot s, U)$, normalised to the same length.

## Part C — TOPP vs quintic on the course arm (8 min; finish the table at home)

6. Implement `quintic_min_time(...)`: precompute $m$, $c$, $g$ at the quintic sample points, then bisect on $T$ until the torque, speed and acceleration limits just hold.
7. Load the task (`load_task()`, `course_arm_model(0.1)`, `waypoint_path(...)`) and fill the table for three limit sets: **sim** (100 % rated speed, 60 % rated torque), **hw** (50 % speed, 60 % torque, $|\ddot q|\le4$ rad/s²) and **first run** (25 % speed, 60 % torque, 4 rad/s²). Use `peak_usage` for the peak speed and torque ratios and the peak jerk.

| Case | $T_{\text{TOPP}}$ (s) | $T_{\text{quintic}}$ (s) | ratio | which limit is active? | peak jerk TOPP / quintic |
|---|---|---|---|---|---|
| sim | 0.615 | 1.10 | 1.79 | | |
| hw | 1.566 | 2.296 | 1.47 | | |
| first run | 2.50 | 4.40 | 1.76 | | |

**Checkpoint C.** Your numbers agree with the table to about 1 %; `test_topp_*` and `test_quintic_*` pass. In the sim case, check the joint-1 torque ratio from `peak_usage` with $N = 4000$ samples. It slightly exceeds 1 (about 1.01) because toppra enforces the constraints only at its grid points. Write one sentence on what this means for a hardware plan.

## Part D — The hardware command file (3 min)

8. Implement `command_table(traj, T, dt, slow)` and `check_command(t, q, model, speed_fraction, margin_deg)`.
9. Make the **first-run command**: take the **hw** TOPP plan and stretch it by `slow = 2` (Workshop 6 P5(b): speeds ÷2, accelerations ÷4, torques stay feasible). Sample it at the servo period 0.02 s (`command_table(hw["traj"], hw["T"], dt=0.02, slow=2)`) and check it against 25 % rated speed with a **10° joint-limit margin** (`check_command(t, q, model, 0.25, 10.0)`; 10° is checklist rule 1, the task's `joint_margin_deg` and the default of `check_command`). Write it with `write_log("cmd_first.csv", t, q)`. This file is the command of the Week 6 hardware run (Part E).

**Checkpoint D.** `check_command` returns `ok = True` with a peak speed ratio close to 1; the command has 158 samples, ends at 3.131 s, and its closest approach to an end stop is joint 4 at 137.5°: 27.5° from the end stop, 17.5° inside the ±155° software limit. The same plan without stretching (`slow = 1`) fails the 25 % check with a ratio of about 2. The stretched plan lasts $2\times1.566 = 3.13$ s, whereas re-solving TOPP directly at 25 % gives 2.50 s. Explain the difference in your reflection.

## Part E — Hardware run on the course arm (supervised station) or simulation fallback (5 min in class + analysis at home)

**Why only a reference trajectory?** The course arm's smart servos are position-controlled (project brief §5.1). Your TOPP plan is sent as a stream of joint-angle set-points every 20 ms, and the servo's internal loop tracks it. The run therefore tests your path, your time scaling and your safety checks, and lets you compare commanded with measured motion. It does **not** test a torque controller.

**The Week 6 hardware run (this is its only definition).** Each team makes **one** run on the course arm: the **hw** TOPP plan of Part C stretched by `slow = 2` (duration $2\times1.566 = 3.13$ s), sampled at the **20 ms** servo period, i.e. your `cmd_first.csv` of Part D. It must have passed `check_command` at **25 % rated speed** with the **10° margin**, and its dry run in simulation (`simulate_servo`) must have been shown to the TA. The 2.50 s plan re-solved at 25 % and the unstretched 50 % plan are **not** run in Week 6. The payload is the 0.1 kg foam phantom.

**Refresher and induction (first 10 min of the lab).** You completed the course-arm hardware safety induction in SBEG 361. The lab starts with a short **refresher** of the course-arm checklist and the **haptic-device (Hapkit) induction**. Only students who attended both and completed the SBEG 361 induction may enter the station; anyone else watches from outside the keep-out zone and uses the simulation fallback. The **full project induction** for the Hapkit and the arm is in **Week 8** (hardware rotation 1) and is required before the project sessions. The Week 6 run is practice: it does **not** count towards the project's hardware completion, which is **two supervised project sessions** (Weeks 10–13, project brief and rubric).

**Safety.** The course-arm hardware safety checklist (Course resources: Course robot models and hardware safety checklist) applies in full. This is a **TA-supervised station run**; the values are those of the checklist:

| Checklist rule | Value in Lab 6 |
|---|---|
| 1 Software joint limits | ≥ 10° inside the hardware limits: ±155° (J6 ±165°); `check_command` with `margin_deg = 10` |
| 2 Keep-out zone | taped, radius = reach + 0.3 m ≈ 0.58 m |
| 3 Joint speed | ≤ 25 % of rated (≤ 40 °/s), because this is the first run of a new motion |
| 4 Payload | ≤ 0.125 kg; here the 0.1 kg foam phantom, fixed to the gripper |
| 5 Induction | SBEG 361 induction completed and today's refresher attended |
| 6 Keep-out discipline | nobody inside the zone while the arm can move; no faces at arm height |
| 7 Before the session | e-stop tested; the TA present for the whole run |
| Current limit | servo current ≤ 60 % of rated, set in the servo firmware by the TA |

1. The TA powers the arm, tests the **e-stop** at the start of the session and holds it during every motion. There is a soft mat under the workspace and the cables are tidy.
2. The TA checks your `check_command` output (25 %, 10°) and your simulated dry run, and writes the predicted peak speed on the session log.
3. **Calibration:** the PoE zero of the course arm is not the servo zero. The TA applies the station's `servo_zero_offset` ($q_{\text{servo}} = q_{\text{model}} + \text{offset}$) and first moves the arm slowly to the first waypoint with the station's homing routine.
4. **One run per team**, started at the TA's signal from the station laptop outside the keep-out zone. The station logger writes `t, cmd_1..6, meas_1..6` (the format of `write_log` and of the project's `hardware.py`). The TA initials the session log; any fault is written down with time, unit number and symptom.
5. Stop immediately (e-stop) on any unexpected motion, noise, contact or overheating smell. Do not restart until the TA has checked the arm.

**Simulation fallback** (no station slot, no arm, no induction, or a failed check; not penalised): run `simulate_servo(t, q)` on the same command (second-order servo, $\omega_n = 25$ rad/s, $\zeta = 0.9$, 20 ms delay, 12-bit encoder; illustrative values), or use the provided `data/sim_servo_log.csv`. That file is a **synthetic** log made from the reference first-run command (instructor script). It is not a measurement, so label it "simulated" in every plot.

10. Implement `tracking_metrics(t, cmd, meas)`: RMS and maximum error per joint, the lag that minimises the RMS error, and the RMS after removing the lag.

**Checkpoint E.** On `data/sim_servo_log.csv`: lag $= 0.10$ s (compare the servo-model estimate, delay $+ 2\zeta/\omega_n = 0.092$ s), largest RMS error $0.042$ rad (joint 1), and RMS after removing the lag below $0.004$ rad. On hardware, expect a larger lag and error, plus a steady error on the gravity-loaded joints 2–3.

**Figure E.** Commanded vs measured angle of joints 1 and 3 against time, with the error on a second axis.

<!-- library-block:begin -->
# Your `sbeg461_dynamics` library

When the tests pass, copy the functions you wrote in Lab 6 into your cumulative library as `timescaling.py`. The functions you write (9):

`path_dynamics(model: dict, path, s: float)`, `accel_limits(m, c, g, sdot: float, tau_min, tau_max)`, `velocity_limit(model: dict, path, s: float, tau_min, tau_max, qdot_max=None, sdot_max: float=100.0, tol: float=1e-09)`, `bang_bang(model: dict, path, tau_min, tau_max, N: int=2000)`, `topp(model: dict, path, tau_max, qdot_max=None, qddot_max=None, N: int=200, tau_min=None)`, `quintic_min_time(model: dict, path, tau_max, qdot_max=None, qddot_max=None, N: int=400, tol: float=0.0001, tau_min=None)`, `command_table(traj, T: float, dt: float=0.02, slow: float=1.0)`, `check_command(t, q, model: dict, speed_fraction: float, margin_deg: float=10.0, tol: float=0.01)`, `tracking_metrics(t, cmd, meas, max_lag: float=0.2)`.

The provided helpers at the top of `starter.py` (loaders, plotting, simulators) do not have to go into the library. The full list of library functions of all labs is in the Week 13 lab handout.
<!-- library-block:end -->

# Deliverables (upload to the course site)

1. `starter.py` passing `LAB_IMPL=starter python -m pytest -q test_lab.py` (all tests).
2. Figures A, B and E (PNG, labelled axes, units, legend, "simulated" or "measured" in the title).
3. The completed table of Part C, with the active limit of each case and one sentence on jerk.
4. Your first-run command file `cmd_first.csv` and the log you analysed (station log or simulated), with the metrics of Part E.
5. Reflection (≤ 1 page, below).

# Reflection (≤ 1 page)

1. In the **hw** case the TOPP plan is 1.47 times faster than the best quintic. Where in the motion does the time difference come from? Use a phase-plane plot of both scalings.
2. Why does re-solving TOPP at 25 % speed (2.50 s) beat stretching the 50 % plan by 2 (3.13 s)? When would you still prefer the stretched plan on hardware?
3. List the caveats of MR §9.4.4 that you saw today (static posture, zero-inertia points, discontinuous acceleration, no margin for feedback). For each, say how your plan handled it.
4. *(Hardware or fallback)* Is the measured lag acceptable for the patient-positioning mock-up? What would you change, in the plan or in the servo settings, before going to 50 % speed?
5. **AI-use note** (required, Course syllabus §8 AI-use policy): name the tool(s), the prompts or questions, and what you kept or rejected. You must be able to explain every line of `bang_bang` and `topp` in the oral check.

# Troubleshooting

| Symptom | Likely cause and fix |
|---|---|
| `bang_bang` returns a negative or `nan` time | You integrated $\dot s$ instead of $x = \dot s^2$, or you did not clip $x \ge 0$; or you divided by $\dot s$ at $\dot s = 0$. |
| `accel_limits` gives $L > U$ everywhere | The bounds were not swapped for $m_i < 0$, or $\dot s$ was used where $\dot s^2$ was needed. |
| `velocity_limit` returns 0 | The state $(s, 0)$ is inadmissible: the robot cannot hold the path statically with these limits (try 30 % torque on the course arm). |
| toppra prints "path not controllable" / `topp` raises | The same static-posture failure, or limits that are inconsistent with the start and end at rest. Check that $g(s)$ fits within the torque limits along the path. |
| toppra plan slightly exceeds a limit between grid points | Expected (constraints are enforced at the grid points only). Increase `N` or derate the limit by 2–3 %. |
| `quintic_min_time` never stops | Feasibility is not monotone if the robot cannot hold the path statically; the function raises after $T > 10^4$ s. Check the gravity torques. |
| Command file fails `check_command` with `starts_at_rest = False` | You sampled `traj(t)` with `t` in the stretched time without dividing by `slow`. |
| Station arm moves to an unexpected pose | Wrong `servo_zero_offset` or degrees vs radians. Press the e-stop, tell the TA, and check the first row of your file against `task["waypoints"][0]`. |
| `ModuleNotFoundError: toppra` | `pip install toppra` in the course environment, or use the JupyterHub fallback. |

# C++ track

You may do this lab in C++17 instead of Python. The functions, data files, checkpoints, figures, results table, hardware run and deliverables are the same. The auto-checks compare your results with `mrcpp` (the C++ port of `modern_robotics` in the course folder `cpp_support`), with the closed-form Workshop 6 results, and with the numbers of the Python reference (`modern_robotics` and the real `toppra`), so both tracks are marked against identical values.

**Files** (in the `cpp` sub-folder of your Lab code folder): `starter.cpp` (your work), `lab.hpp` (declarations and documentation — do not change), `test_lab.cpp` (auto-checks, doctest), `cpp_cases.json` (reference numbers, including the random path states that `test_lab.py` draws with NumPy), `CMakeLists.txt`, and given code you do not edit: `course_arm.hpp` / `course_arm.cpp` (the C++ copy of `course_arm.py`: the course arm and its safety-checklist values, numbers identical) and `toppralite.hpp` / `toppralite.cpp` (a small C++ stand-in for the part of `toppra` used in `topp`, see below). The data files are the same `data/lab6_task.json` and `data/sim_servo_log.csv`. `cpp_support` (Course resources) must sit next to your lab folders or in a parent folder of them (the build finds it automatically up to 6 levels up).

**Requirements.** A C++17 compiler — Windows: Visual Studio Build Tools (2022 or 2026), workload "Desktop development with C++"; macOS: `xcode-select --install`; Ubuntu: `sudo apt install build-essential`. CMake and Ninja come with `pip install cmake ninja` (both are in the course `requirements.txt`). `toppra` is not needed for the C++ track.

**Build and run** (from your Lab code folder):

```bash
python run_lab.py 6 --cpp                      # configure, build, run, print Score: k/n
# or directly:
cmake -S cpp -B build -G Ninja
cmake --build build
./build/lab_tests                              # Windows: build\lab_tests.exe
./build/lab_tests -tc="*path_dynamics*,*accel_limits*"   # only some tests (like pytest -k)
```

With the untouched starter, the program builds and all 20 test cases fail with `TODO: <function>`: this is expected. Rebuild (`cmake --build build`) after every change. The whole C++ test run takes well under a second.

**Functions to write** (in `starter.cpp`, the same 9 as in Python, same order): `path_dynamics`, `accel_limits`, `velocity_limit` (Part A); `bang_bang`, `topp`, `quintic_min_time` (Parts B–C); `command_table`, `check_command` (Part D); `tracking_metrics` (Part E). Given (complete, as in `starter.py`): `straight_line_path`, `semicircle_path`, `waypoint_path`, `mr_2r_model`, `elbow_exo_model`, `gantry_model`, `course_arm_model`, `inverse_dynamics`, `load_task`, `data_path`, `limits_x`, `quintic_s`, `quintic_traj`, `peak_usage`, `write_log`, `load_log`, `simulate_servo`, and the small helpers `linspace` (= `np.linspace`) and `interp` (= `np.interp`) at the top of `starter.cpp`. As in Python, `path_dynamics` uses `inverse_dynamics` (the RNEA of `mrcpp`), not new dynamics code, and `topp` is a thin wrapper around the solver library.

**`topp` with `toppralite`.** `toppra` is a Python library, so the C++ track gives `toppralite`: the same TOPP-RA algorithm with the same constraint forms (torque: collocation; acceleration: interpolation; velocity: a bound on $\dot s^2$), the same greedy forward pass and the same constant-acceleration parametrisation. It reproduces the durations of the real `toppra` to about $10^{-8}$ s, and the tests check your `topp` against the `toppra` numbers. The calls translate one to one:

| Python (`toppra`) | C++ (`toppralite`, `namespace ta = toppralite;`) |
|---|---|
| `constraint.JointTorqueConstraint(inv_dyn, np.c_[tau_min, tau_max], np.zeros(n))` | `ta::joint_torque_constraint(inv_dyn, tau_lim, Eigen::VectorXd::Zero(n))`; `tau_lim` is $n\times2$, `inv_dyn` a lambda calling `inverse_dynamics(model, q, qd, qdd)` |
| `constraint.JointVelocityConstraint(np.c_[-v, v])`, `JointAccelerationConstraint(np.c_[-a, a])` | `ta::joint_velocity_constraint(vlim)`, `ta::joint_acceleration_constraint(alim)` ($n\times2$ matrices) |
| `algo.TOPPRA(cons, path, gridpoints=grid, parametrizer="ParametrizeConstAccel")` | `ta::TOPPRA inst(cons, path, grid);` |
| `traj = inst.compute_trajectory(0.0, 0.0)`; `None` if infeasible | `auto traj = inst.compute_trajectory(0.0, 0.0);` `nullptr` if infeasible (then `throw std::runtime_error(...)`) |
| `traj.duration`, `inst.problem_data.sd_vec` | `traj->duration()`, `inst.problem_data().sd_vec` |

**Checkpoints.**

| Checkpoint | Command (`./build/lab_tests ...`) | C++ test cases |
|---|---|---|
| A — Path dynamics, cone, velocity limit | `-tc="*path_dynamics*,*accel_limits*,*test_velocity_limit*"` | 7 |
| B — Bang-bang and TOPP | `-tc="*bang_bang*,*topp_gantry*,*torque_scaling*"` | 5 |
| C — TOPP vs quintic on the course arm | `-tc="*course_arm_respects*,*quintic*"` | 3 |
| D — Command file and safety check | `-tc="*command*,*safety*"` | 3 |
| E — Tracking metrics | `-tc="*tracking*"` | 2 |

All 20 test cases pass with `./build/lab_tests`. The Python track counts 22 tests because pytest counts each $s$ of `test_path_dynamics_2r_closed_form` separately; the C++ track checks the three values inside one test case. With the Python starter, `test_task_follows_safety_checklist` already passes; in C++ it also runs `check_command` on the arm resting at `q_home` (passes) and 8° from an end stop (fails the 10° margin), so it fails until `check_command` is written. The checkpoint values in the procedure above are the same in both tracks.

**Differences from the Python version.**

| Python | C++ |
|---|---|
| a model `dict` (`model["Mlist"]`, `r["tau_max"]`, …) | `lab::Model` (= `course_arm::Robot`) with fields `name`, `n`, `Mlist`, `Glist`, `Slist`, `g`; the course arm also fills `M`, `limits` ($n\times2$), `qdot_max`, `tau_max`, `q_home` |
| `StraightLinePath(q0, q1)`, `SemicirclePath(0.3)`, `waypoint_path(W)`; `path(s, order)` with arrays of $s$ | `lab::straight_line_path(q0, q1)`, `lab::semicircle_path(0.3)`, `lab::waypoint_path(W)` ($K\times n$, one waypoint per row) return a `lab::Path` (`std::function`); call `path(s, order)` with one $s$ at a time |
| `traj(t, order)` with an array of times; `traj.duration` | `lab::Trajectory` (`std::function`), one $t$ at a time; the duration is `ToppResult::T` |
| `toppra` inside `topp` | the given `toppralite` (table above) |
| returned dicts and 3-tuples (`path_dynamics`, `bang_bang`, `topp`, `peak_usage`, `check_command`, `tracking_metrics`) | small structs with the same field names: `PathDynamics{m, c, g}`, `BangBang`, `ToppResult{T, traj, s_grid, sdot_grid}`, `PeakUsage`, `CommandCheck`, `TrackingMetrics`; `accel_limits` and `command_table` return a `std::pair` (`auto [L, U] = ...`, `auto [t, q] = ...`, `q` is $N\times n$, row = sample) |
| optional arguments `qdot_max=None`, `qddot_max=None`, `tau_min=None` | an empty `Eigen::VectorXd()` (the default) means "not given" |
| scalars broadcast to all joints (`qddot_max=4.0`, `[-tm]`) | always a vector of size $n$: `Eigen::VectorXd::Constant(6, 4.0)` |
| `raise RuntimeError(...)` (`bang_bang`, `topp`, `quintic_min_time`) | `throw std::runtime_error(...)`; in the `F` loop of `bang_bang` catch only `const std::runtime_error&` |
| `_limits_x`, `DATA / "sim_servo_log.csv"`, `load_task()` (a dict) | `lab::limits_x`, `lab::data_path("sim_servo_log.csv")`, `lab::load_task()` (a `lab::Task` struct with the JSON fields) |
| `course_arm.COURSE_ARM_SAFETY["speed_fraction"]` | `course_arm::COURSE_ARM_SAFETY.speed_fraction` (same field names) |
| `np.pi`, `np.deg2rad(x)` | a local `constexpr double PI = 3.14159265358979323846;`, `x * PI / 180` (`M_PI` is not standard C++) |

**Figures, table and command file in C++.** Write a small `cpp/experiments.cpp` (your own `main()`) that writes the data of Figures A, B and E and of the Part C table as CSV files (`sbeg::write_csv` from `<sbeg/labio.hpp>`), and plot them with a short Python script (matplotlib) or any plotting tool you prefer; plotting stays in Python. To build it, add these lines at the end of `cpp/CMakeLists.txt` (below `sbeg_lab(...)`), then `cmake --build build` and run `./build/experiments` (Windows: `build\experiments.exe`):

```cmake
add_executable(experiments experiments.cpp starter.cpp course_arm.cpp toppralite.cpp)
target_link_libraries(experiments PRIVATE sbeg_mrcpp)
target_include_directories(experiments PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(experiments PRIVATE "SBEG_LAB_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}\"")
set_target_properties(experiments PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
``` Write the first-run command with `lab::write_log("cmd_first.csv", t, q)` after `lab::check_command(t, q, model, 0.25, 10.0)` returns `ok`; the file format is the one the station logger and the project's `hardware.py` read, so Part E (hardware run or simulation fallback with `lab::simulate_servo`) does not change.

A matrix-size mistake (e.g. adding a 2-vector to a 6-vector) does not crash the test program: it is reported as a failed test with the message "Eigen: wrong matrix size or index out of range".

**C++ troubleshooting.**

| Symptom | Likely cause | Fix |
|---|---|---|
| `cpp_support not found` | the course folder `cpp_support` is not next to / above your Lab code folder | copy it there, or configure with `-DSBEG_CPP_SUPPORT=<path>` |
| `No CMAKE_CXX_COMPILER could be found` | no C++ compiler installed | install it (see Requirements); on Windows run from the "x64 Native Tools Command Prompt" if Ninja cannot find `cl` |
| `cannot open file: .../cpp_cases.json` | Lab code folder moved after configuring | delete the `build` folder and configure again |
| `test_bang_bang_detects_velocity_limit_curve` fails with `"other exception: ..." == "std::runtime_error"`, `"Eigen size/index error: ..."` or `"no exception"` | `bang_bang` not written yet, a wrong exception type, a matrix-size mistake, or the velocity limit curve is not detected | `throw std::runtime_error(...)` when curve A meets the velocity limit curve; fix the size error first |
| `bang_bang` hides a `TODO` error from `path_dynamics` | `catch (...)` (or `catch (const std::exception&)`) in the backward loop of curve F | catch only `const std::runtime_error&` (the velocity-limit-curve signal) |
| `topp` fails with "bad input gridpoints" or a dof message | grid not strictly increasing on $[0, 1]$, or limit matrices not $n\times2$ | use `linspace(0.0, 1.0, N + 1)`; build `tau_lim` with `.col(0) = tau_min`, `.col(1) = tau_max` |
| `topp` result is far from 1.5656 s on the course arm | velocity or acceleration constraint missing (empty vector passed on) | add each constraint only when its vector is non-empty, and pass all three limit sets of the test |
| Wrong results only after `auto x = A * b;` | `auto` keeps an Eigen expression, not a vector | write the type: `Eigen::VectorXd x = A * b;` |
| A loop like `for (std::size_t k = N; k >= 0; --k)` never ends or crashes | unsigned counter: always $\ge 0$ | use `int` (or `Eigen::Index`) for loops that count down |
| Tests still show your old result | program not rebuilt | `cmake --build build` before `./build/lab_tests` |
