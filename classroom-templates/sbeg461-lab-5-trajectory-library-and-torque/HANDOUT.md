---
title: "Lab 5 — Trajectory library and torque feasibility on the course arm"
subtitle: "SBEG 461 Robot Kinematics and Dynamics — Week 5"
---

# Overview

| Item | Detail |
|---|---|
| Session | Session 4, 50 min in class + up to 2 h to finish (due before the Week 6 lab) |
| CLOs | 461-4 (point-to-point and via-point trajectories that respect kinematic and torque limits); 461-3 (actuator limits); 461-2 (reuse of the verified RNEA) |
| Files | `lab/starter.py` (you edit), `lab/test_lab.py` (auto-checks), `lab/data/course_arm.json`, `lab/data/course_arm.urdf`, `lab/data/tasks.json` (data; the course-arm numbers are those of Course resources: Course robot models and hardware safety checklist) |
| Reference | `modern_robotics` (`CubicTimeScaling`, `QuinticTimeScaling`, `JointTrajectory`, `InverseDynamics`, `InverseDynamicsTrajectory`), `scipy.interpolate.BSpline` |
| Reading | MR §9.1–9.3 (printed pp. 327–338) and the start of §9.4, Eqs. 9.30–9.33 (pp. 338–340); MR §8.3.2 (RNEA, pp. 292–293); Corke §3.1 (pp. 43–51) and §7.4.1 *Joint-Space Motion* (pp. 153–155); Gotor §7.1–7.2 (pp. 193–212) |
| Assessment | Auto-checked (`test_lab.py`) + results table + plots + short reflection (not a formal report) |
| AI use | Allowed with disclosure (see Reflection) |
| Tier | 0 (simulation only); Step 6 makes a quintic baseline for Lab 6 (the Week 6 hardware command is defined and made in Lab 6) |

The functions you write today form the trajectory module of your `sbeg461_dynamics` library. Lab 6 (time-optimal scaling and the first hardware run of a trajectory) and the course project (`sbeg461_project/traj.py`: `quintic_s`, `joint_quintic`, `torque_profile`, `check_limits`, `min_quintic_time`) use the same ideas and similar names. The RNEA is the one you verified in Labs 2 and 4; a reference copy is provided as `rnea(...)` in `starter.py`, and you may paste your own.

**The robot.** The course arm (the desktop 6R used in every hardware session; numbers from Course resources: "Course robot models and hardware safety checklist", copied into `data/course_arm.json`) with its maximum payload of 0.25 kg at the tool point. Its inertial data and ratings are **illustrative values**. For hardware runs the course uses at most 50 % of the rated joint speed (160 °/s) and 60 % of the rated torque, so the limits in this lab are
$$\dot\theta_{\max} = 1.396\ \text{rad/s (all joints)},\qquad \tau_{\max} = (1.08,\ 1.80,\ 0.90,\ 0.36,\ 0.36,\ 0.24)\ \text{N m}.$$

**Application context.**
*Biomedical:* in a hospital laboratory the arm moves a 0.25 kg sample carrier from its ready pose A to a centrifuge rack B, directly or via a waypoint W above a tray, and seats a sample tube with a short dip of the forearm (A → D). Smooth (quintic) motions keep the samples from spilling, the same reason MR §9.4 mentions "a glass of water".
*Defence (protective):* on an EOD training rig the same arm lifts a 0.25 kg **inert mock package** from A to C. Operators of real EOD manipulators use the same tools: slow, smooth time scalings and a check that no actuator saturates while holding a suspect object.

# Learning objectives

By the end of the lab you can:

1. Implement the cubic, quintic and trapezoidal time scalings (MR §9.2.2) and straight-line joint trajectories, and verify them against `modern_robotics` and the peak values printed in MR Figs. 9.3–9.4.
2. Implement cubic via-point interpolation (MR Eqs. 9.26–9.29) with a via-velocity heuristic, reproduce MR Fig. 9.7(a), and implement a uniform cubic B-spline with its convex-hull property (MR p. 338).
3. Compute the joint torques along a trajectory with the RNEA and check them, and the joint speeds, against the course-arm limits (MR Eqs. 9.30–9.31).
4. Use uniform time scaling, $\tau_c = g + (\tau - g)/c^2$, to find the shortest feasible duration of any trajectory, decide whether the speed or the torque limit is active, and explain the result physically.

# Pre-lab check (complete before the session; 5 questions)

1. Write MR Eq. 9.2 for a straight-line path in joint space. Which term vanishes and why?
2. A joint moves $\Delta\theta = 1$ rad from rest to rest with a cubic time scaling in $T = 2$ s. Give its peak speed and peak acceleration.
3. A trapezoid has $v = 0.5$ s⁻¹ and $a = 1$ s⁻² (in $s$-units). Is it three-stage? Give $T$ and $t_a$.
4. A rest-to-rest joint trajectory is slowed down so that it takes twice as long. By what factor do $M(\theta)\ddot\theta$, $c(\theta,\dot\theta)$ and $g(\theta)$ change at corresponding points of the path?
5. Why does MR (p. 338) say that the convex-hull property of B-splines helps to respect joint limits, while cubic via-point trajectories may not?

*Answers (for self-check):* 1. $\ddot\theta = \Delta\theta\,\ddot s + \frac{d^2\theta}{ds^2}\dot s^2$ with $\frac{d^2\theta}{ds^2} = 0$ (MR Eq. 9.5), so $\ddot\theta = \ddot s\,\Delta\theta$ and $\dot\theta = \dot s\,\Delta\theta$: every joint follows the same profile scaled by its own $\Delta\theta_i$. 2. $\frac{3}{2T} = 0.75$ rad/s at $t = 1$ s; $\frac{6}{T^2} = 1.5$ rad/s² at $t = 0$ and $T$. 3. $v^2/a = 0.25 \le 1$: three stages; $T = (a + v^2)/(va) = 2.5$ s, $t_a = 0.5$ s. 4. $\dot\theta$ halves and $\ddot\theta$ quarters, so $M\ddot\theta$ and $c = \dot\theta^{\mathsf T}\Gamma\dot\theta$ both become $\tfrac14$ of their values; $g(\theta)$ is unchanged because the configurations are the same. 5. The joint-limit set is a box (convex); a B-spline point is a convex combination of control points, so if all control points are inside the box the whole curve is. A cubic via-point trajectory can overshoot between vias (MR Fig. 9.7).

# Setup

```bash
cd "<your Lab code folder>"   # or your GitHub Classroom repo
python -c "import numpy, scipy, modern_robotics; print('ok')"
LAB_IMPL=starter python -m pytest -q test_lab.py   # 26 failed (NotImplementedError), 1 skipped is expected
```

On Windows (PowerShell) use `$env:LAB_IMPL="starter"; python -m pytest -q test_lab.py`.

Rules: inside `starter.py` do **not** call `modern_robotics` or SciPy (they are the ground truth of the tests); use NumPy and the provided helpers (`rnea`, `load_course_arm`, `load_tasks`, `export_csv`, …). Every function has a docstring with its formula and MR reference, and a `# TODO` hint. Run the tests after each step; a checkpoint is complete when the listed tests pass. Ask a TA to sign it off.

# Procedure

## Step 1 — Time scalings (10 min)

Implement `cubic_time_scaling(T, t)` and `quintic_time_scaling(T, t)` (return $(s, \dot s, \ddot s)$, work for scalars and arrays, clip $t$ to $[0, T]$), then `trapezoid_params(v, a)` → $(T, t_a, v_{\text{peak}})$ and `trapezoid_time_scaling(v, a, t)` (MR Eqs. 9.16–9.24). Handle the bang-bang case $v^2/a > 1$: the peak speed is then $\sqrt a$ and $t_a = T/2 = 1/\sqrt a$.

Plot $s$, $\dot s$, $\ddot s$ of the three scalings on one figure for $T = 1$ s (for the trapezoid take $v = 1.5$ s⁻¹ and $a = 4.5$ s⁻², which also gives $T = 1$ s: check it with `trapezoid_params`) and compare with MR Figs. 9.3–9.5.

**Checkpoint 1:** `test_cubic_matches_modern_robotics_and_mr_fig_9_3`, `test_quintic_matches_modern_robotics_and_mr_fig_9_4`, `test_time_scalings_accept_scalars_and_clip`, `test_trapezoid_three_stage_mr_eqs_9_16_to_9_24`, `test_trapezoid_bang_bang_when_v2_over_a_exceeds_1`.

## Step 2 — Straight-line joint trajectories (5 min)

Implement `trapezoid_from_limits(theta_start, theta_end, qdot_lim, qddot_lim)` (MR Eqs. 9.14–9.15; joints that do not move impose no limit) and `joint_trajectory(theta_start, theta_end, T, N, method)` with `method` = 3, 5 or `"trap"`, returning `(t, Q, Qd, Qdd)`.

**Checkpoint 2:** `test_trapezoid_from_limits_mr_exercise_9_4_data`, `test_joint_trajectory_matches_modern_robotics`, `test_joint_trajectory_trapezoid_respects_limits`. For A → B with $\ddot\theta_{\text{limit}} = 4$ rad/s² on every joint (in `tasks.json`) you should get $v = 1.1636$ s⁻¹, $a = 3.3333$ s⁻², $T = 1.2085$ s; joint 1 ($\Delta\theta_1 = 1.2$ rad, the largest move) saturates both.

## Step 3 — Via points and B-splines (10 min)

Implement `via_cubic_coefficients` (MR Eqs. 9.26–9.29), `heuristic_via_velocities` (Lecture B rule: zero at the ends; at an interior via the average of the adjacent slopes if both have the same strict sign, otherwise zero), `via_trajectory`, `bspline_basis` and `uniform_cubic_bspline` (global parameter $u\in[0, n-3]$; segment $i = \lfloor u\rfloor$; return the point and its first two derivatives with respect to $u$).

Reproduce MR Fig. 9.7(a)/9.8: vias $(0,0), (0,1), (1,1), (1,0)$ at $T = 0, 1, 2, 3$ with velocities $(0,0), (1,0), (0,-1), (0,0)$. Plot the path in the $(x, y)$ plane and $x(t), y(t)$. Then plot a uniform cubic B-spline with the same four points as control points (one segment) and, on the same axes, with each end point repeated three times (clamped). Which curves leave the unit square?

**Checkpoint 3:** `test_via_cubic_coefficients_formula_and_continuity`, `test_via_trajectory_mr_figure_9_7a`, `test_heuristic_via_velocities`, `test_bspline_basis_partition_of_unity`, `test_uniform_cubic_bspline_matches_scipy_and_convex_hull`.

## Step 4 — Torque feasibility on the course arm (15 min)

Implement `torque_profile(robot, Q, Qd, Qdd)` (one `rnea` call per sample, $\mathcal F_{\text{tip}} = 0$), `check_limits(robot, Qd, Tau)` and `min_time_scale(robot, Q, Qd, Qdd)`.

The idea of `min_time_scale` (MR §9.7 credits it to Hollerbach): if a trajectory $\theta(t)$ is replayed $c$ times slower, $\theta_c(t) = \theta(t/c)$, the configurations are the same, the velocities are divided by $c$ and the accelerations by $c^2$. Because $c(\theta,\dot\theta) = \dot\theta^{\mathsf T}\Gamma(\theta)\dot\theta$ is quadratic in the velocity (MR p. 339),
$$\tau_c = g(\theta) + \frac{\tau - g(\theta)}{c^2} = g + \frac{d}{c^2}.$$
So one evaluation of the trajectory gives the feasible range of $c$ in closed form: $\lvert\dot\theta\rvert/c \le \dot\theta_{\max}$ and, sample by sample and joint by joint, $c^2 \ge d/(\tau_{\max} - g)$ if $d > 0$ or $c^2 \ge -d/(\tau_{\max} + g)$ if $d < 0$. If $\lvert g\rvert \ge \tau_{\max}$ anywhere, the arm cannot even hold that configuration and no $c$ helps (raise `ValueError`). For a rest-to-rest trajectory computed with $T = 1$ s, $c$ is the minimum feasible duration in seconds.

**Checkpoint 4:** `test_course_arm_data`, `test_torque_profile_matches_modern_robotics`, `test_check_limits`, `test_min_time_scale_equals_bisection`, `test_min_time_scale_checkpoint_values`, `test_min_time_scale_rejects_static_overload`, `test_via_trajectory_on_course_arm_is_feasible_and_stretch_works`.

Values to expect: gravity torque at A with the payload $g = (0,\ -1.140,\ -0.570,\ -0.242,\ 0,\ 0)$ N m, i.e. 63 %, 63 % and 67 % of the derated limits of joints 2–4 before the arm even moves.

## Step 5 — Experiments and results table (10 min in class, finish at home)

Use 401 samples per point-to-point trajectory (computed with $T = 1$ s) and fill in the table. "Max $\tau$ ratio" is the largest $\lvert\tau_i\rvert/\tau_{\max,i}$ when the move is run at its $T_{\min}$.

| Move (tasks.json) | Profile | $T_{\text{vel}}$ (s) | $T_\tau$ (s) | $T_{\min}$ (s) | Active limit (joint) | Max $\tau$ ratio at $T_{\min}$ |
|---|---|---|---|---|---|---|
| A → B | cubic | | | | | — |
| A → B | quintic | | | | | |
| A → B | trapezoid ($\ddot\theta_{\text{limit}} = 4$) | — | — | | — | |
| A → C | quintic | | | | | — |
| A → D | cubic | | | | | — |
| A → D | quintic | | | | | — |
| A → W → B | via cubic (0, 1.2, 2.4 s) | | | | | — |

Reference values (your numbers should agree to 3 digits): A → B cubic 1.289 / 0.402 s and quintic 1.611 / 0.405 s, both limited by the **speed of joint 1**; at the quintic $T_{\min}$ the largest torque ratio is 0.782 on joint 3 (0.469 of the *rated* torque, as in the Lecture B course-arm example), and the trapezoid needs 1.2085 s with 0.781 on joint 3. A → C quintic 1.074 / 0.346 s. A → D: cubic 0.107 / 0.237 s, quintic 0.134 / 0.235 s, both limited by the **torque of joint 4**. The via trajectory is feasible as timed ($c = 0.716$); it could be run in $2.4c = 1.719$ s, limited by the speed of joint 2.

Questions (answer in ≤ 150 words in total):

1. Why are the long moves speed-limited and the short move A → D torque-limited? (Hint: $T_{\text{vel}} \propto \lvert\Delta\theta\rvert$, while the inertial torque is $\propto \lvert\Delta\theta\rvert/T^2$, so for a fixed gravity margin $T_\tau$ grows only like $\sqrt{\lvert\Delta\theta\rvert}$.)
2. In A → D only joints 2 and 3 move, yet joint 4 limits the motion. Explain with the off-diagonal terms of $M(\theta)$ (Week 1) and the small rating of the wrist motor.
3. The trapezoid takes 25 % less time than the quintic on A → B with almost the same peak torque. Why would you still prefer the quintic on hardware?
4. For A → B, plot $\tau_i(t)/\tau_{\max,i}$ for all joints at $T = T_{\min}$ (quintic). Which part of the joint-3 torque is gravity and which part is dynamics?

## Step 6 — Quintic baseline for Lab 6 (at home)

**This file is not run on the arm.** The Week 6 hardware command is defined in Lab 6 (Part D, step 9, and Part E): it is your **time-optimal (TOPP) plan stretched by `slow = 2`**, sampled at the servo period of **20 ms**, checked with `check_command` at 25 % rated speed with a 5° joint-limit margin, written as `cmd_first.csv`, and run **once per team** on the TA-operated station. Follow Lab 6 and Course resources: "Course robot models and hardware safety checklist" for that run.

What you make here is the quintic you will compare it with in Lab 6 (TOPP vs quintic). Generate the A → B quintic with $T = 3.5$ s (more than twice $T_{\min}$, so it also stays below 25 % of the rated speed), sample it every 20 ms (the same period as the Lab 6 command), check it with `check_limits` and save it with `export_csv("traj_AB_quintic.csv", t, Q, Qd)`. Bring the file and your plots to the Week 6 lab.

Final run: `LAB_IMPL=starter python -m pytest -q test_lab.py` → 26 passed, 1 skipped (the shared-model check is skipped outside the course build tree).

<!-- library-block:begin -->
# Your `sbeg461_dynamics` library

When the tests pass, copy the functions you wrote in Lab 5 into your cumulative library as `trajectory.py`. The functions you write (14):

`cubic_time_scaling(T, t)`, `quintic_time_scaling(T, t)`, `trapezoid_params(v, a)`, `trapezoid_time_scaling(v, a, t)`, `trapezoid_from_limits(theta_start, theta_end, qdot_lim, qddot_lim)`, `joint_trajectory(theta_start, theta_end, T, N, method=5, v=None, a=None)`, `via_cubic_coefficients(beta, beta_dot, times)`, `heuristic_via_velocities(points, times)`, `via_trajectory(points, velocities, times, t)`, `bspline_basis(u)`, `uniform_cubic_bspline(ctrl, u)`, `torque_profile(robot, Q, Qd, Qdd)`, `check_limits(robot, Qd, Tau, tol=1e-09)`, `min_time_scale(robot, Q, Qd, Qdd)`.

Names shared with other labs:

* `torque_profile(robot, Q, Qd, Qdd)` does the same job as Lab 2's `torque_profile(robot, theta, dtheta, ddtheta, Ftip=None)` with $F_{tip} = 0$, and takes its arguments in the same order. Lab 2's version (in `rnea.py`) can replace this one; Lab 8 uses the same signature again.
* `rnea` and the small $SE(3)$ helpers at the top of `starter.py` are provided copies of your Lab 2/Lab 4 code.

The provided helpers at the top of `starter.py` (loaders, plotting, simulators) do not have to go into the library. The full list of library functions of all labs is in the Week 13 lab handout.
<!-- library-block:end -->

# Deliverables (upload to the course site)

1. `starter.py` (all tests pass).
2. Three plots with axis labels and units: (i) $s, \dot s, \ddot s$ for cubic, quintic and trapezoid (Step 1); (ii) MR Fig. 9.7(a) reproduction with the two B-splines (Step 3); (iii) normalised torques $\tau_i/\tau_{\max,i}$ along A → B at $T_{\min}$ (Step 5, Q4).
3. The completed results table and the answers to the Step 5 questions (≤ 150 words).
4. `traj_AB_quintic.csv` (Step 6, simulation baseline for Lab 6; not a hardware command).
5. Reflection (below), ≤ 200 words.

# Reflection (≤ 200 words)

- The minimum times in this lab assume the model (masses, payload, friction-free joints) is exact. Which modelling errors from Week 4 (friction, rotor inertia, gear efficiency) would make the real course arm saturate earlier, and how much margin would you keep?
- A patient-handling or casualty-handling robot must never saturate an actuator while carrying a person (MR p. 347: a saturated actuator leaves no torque for feedback correction). How would you turn today's `min_time_scale` into a safety rule for such a robot?
- **AI-use note:** state whether you used an AI assistant, for which step, what you asked, and how you verified the answer (tests, hand check, comparison with `modern_robotics`). Code that you cannot explain in the lab practical (Week 13) will not earn credit.

# Troubleshooting

| Symptom | Likely cause |
|---|---|
| Cubic/quintic positions right but $\dot s$ or $\ddot s$ wrong by a constant factor | forgot the $1/T$ or $1/T^2$ of the chain rule when writing $s$ in terms of $r = t/T$ |
| Trapezoid ends at $s \ne 1$ | used $v$ instead of the peak speed $\sqrt a$ in the bang-bang case, or mixed joint units with $s$-units |
| `trapezoid_from_limits` returns `inf` or `nan` | divided by $\lvert\Delta\theta_i\rvert = 0$ for a joint that does not move: skip those joints |
| `JointTrajectory` comparison fails by one row | `np.linspace(0, T, N)` includes both end points, as MR does ($N$ rows, spacing $T/(N-1)$) |
| Via trajectory jumps at a via point | segment index off by one, or used the global $t$ instead of $\Delta t = t - T_j$ |
| B-spline does not match SciPy | wrong segment at $u = n - 3$ (clamp $i$ to $n - 4$) or swapped blending functions |
| Torques do not match `modern_robotics` | used the model without the payload (`load_course_arm(payload=False)`) on one side only, or the gravity vector with the wrong sign (`g = (0, 0, -9.81)`) |
| `min_time_scale` disagrees with the bisection test | forgot to subtract gravity before dividing by $c^2$, or used $\lvert d\rvert/(\tau_{\max} - \lvert g\rvert)$ (too conservative when $d$ and $g$ have opposite signs) |
| `ValueError` for every trajectory | `tau_max` is the derated value and you multiplied by 0.6 again |

# C++ track

You may do this lab in C++17 instead of Python. The functions, data files, checkpoints, results table and deliverables are the same (deliverable 1 is `cpp/starter.cpp` instead of `starter.py`); the auto-checks compare your results with `mrcpp` (the C++ port of `modern_robotics` in the course folder `cpp_support`) and with the numbers of the Python reference (including the SciPy B-spline values), so both tracks are marked against identical values.

**Files** (in the `cpp` sub-folder of your Lab code folder): `starter.cpp` (your work), `lab.hpp` (declarations and documentation — do not change), `helpers.cpp` (the provided helpers: `skew`, `trans_inv`, `adjoint`, `ad`, `matrix_exp6`, `rnea`, `load_course_arm`, `load_tasks`, `export_csv` — do not change), `test_lab.cpp` (auto-checks, doctest), `cpp_cases.json` (reference numbers), `plot_results.py` (plots for Steps 1, 3 and 5), `CMakeLists.txt`. The data files are the same files in `data/`. `cpp_support` (Course resources) must sit next to your lab folders or in a parent folder of them (the build finds it automatically up to 6 levels up).

**Requirements.** A C++17 compiler — Windows: Visual Studio Build Tools (2022 or 2026), workload "Desktop development with C++"; macOS: `xcode-select --install`; Ubuntu: `sudo apt install build-essential`. CMake and Ninja come with `pip install cmake ninja` (both are in the course `requirements.txt`).

**Build and run** (from your Lab code folder):

```bash
python run_lab.py 5 --cpp                      # configure, build, run, print Score: k/n
# or directly:
cmake -S cpp -B build -G Ninja
cmake --build build
./build/lab_tests                              # Windows: build\lab_tests.exe
./build/lab_tests -tc="*cubic*,*quintic*"      # only some tests (like pytest -k)
python cpp/plot_results.py                     # plots of Steps 1, 3 and 5 (Q4)
```

With the untouched starter, the program builds and all 20 test cases fail with `TODO: <function>`: this is expected. Rebuild (`cmake --build build`) after every change.

**Functions to write** (in `starter.cpp`, same order as in Python, 14 functions): `cubic_time_scaling`, `quintic_time_scaling`, `trapezoid_params`, `trapezoid_time_scaling` (Step 1); `trapezoid_from_limits`, `joint_trajectory` (Step 2); `via_cubic_coefficients`, `heuristic_via_velocities`, `via_trajectory`, `bspline_basis`, `uniform_cubic_bspline` (Step 3); `torque_profile`, `check_limits`, `min_time_scale` (Step 4). The helpers in `helpers.cpp` are given, as the provided helpers of `starter.py`; so are the scalar-time overloads of the three time scalings and the vector overload of `bspline_basis` (they call your functions). Do not call `mrcpp` in `starter.cpp`.

**Checkpoints.** 1: `./build/lab_tests -tc="*cubic*,*quintic*,*scalars*,*trapezoid_three*,*bang_bang*"`; 2: `-tc="*from_limits*,*joint_trajectory*"`; 3: `-tc="*via_cubic*,*figure_9_7a*,*heuristic*,*bspline*"`; 4: `-tc="*course_arm*,*torque_profile*,*check_limits*,*min_time_scale*"`; finally all 20 test cases pass (`./build/lab_tests`). The Python track counts 27 tests because pytest counts each value of a parametrized test separately (three $T$ for the cubic and the quintic, two methods for `joint_trajectory`, three moves for the bisection test); the C++ track checks the same values in one test case each. In C++, `test_course_arm_data` also calls `check_limits`, so it passes only once `check_limits` is written.

**Printing, the results table and the Step 6 file.** For the values of Steps 2 and 5 and for `traj_AB_quintic.csv`, add a temporary `TEST_CASE` at the end of `test_lab.cpp` (include `<iostream>`; remove it before you submit), for example:

```cpp
TEST_CASE("my_step5") {
    const lab::Robot arm = lab::load_course_arm();
    const lab::Tasks tk = lab::load_tasks();
    const lab::Trajectory tr = lab::joint_trajectory(tk.A, tk.B, 1.0, 401, lab::QUINTIC);
    const lab::TimeScaleResult r = lab::min_time_scale(arm, tr.Q, tr.Qd, tr.Qdd);
    std::cout << r.c_vel << " " << r.c_tau << " " << r.binding.first << " joint " << r.binding.second + 1 << "\n";
    const lab::Trajectory q = lab::joint_trajectory(tk.A, tk.B, 3.5, 176, lab::QUINTIC);  // Step 6: every 20 ms
    std::cout << lab::check_limits(arm, q.Qd, lab::torque_profile(arm, q.Q, q.Qd, q.Qdd)).ok << "\n";
    lab::export_csv("traj_AB_quintic.csv", q.t, q.Q, q.Qd);
}
```

For the plots, `./build/lab_tests` writes `lab5_cpp_scaling_cubic.csv`, `lab5_cpp_scaling_quintic.csv`, `lab5_cpp_scaling_trapezoid.csv` (Step 1, $T = 1$ s), `lab5_cpp_fig_9_7a.csv`, `lab5_cpp_bsplines.csv` (Step 3: the via-point cubic and the open and clamped B-splines of the same four points) and `lab5_cpp_tau_AB.csv` (Step 5 Q4: $\tau_i/\tau_{\max,i}$ along A → B at your $T_{\min}$) into the folder you run it from, each one when the functions it uses return results of the right size; `python cpp/plot_results.py` saves `lab5_step1_scalings.png`, `lab5_step3_vias.png` and `lab5_step5_torques.png`.

**Differences from the Python version.**

| Python | C++ |
|---|---|
| `t` a scalar or an array | `Eigen::VectorXd t`; the scalar-`t` overload is given and returns `lab::TimeScalingPoint` (fields `s, sd, sdd`) |
| tuples `(s, sdot, sddot)`, `(T, t_a, v_peak)`, `(v, a, T)`, `(t, Q, Qd, Qdd)`, `(B, Bd, Bdd)`, `(p, dp, ddp)` | the structs `TimeScaling`, `TrapezoidParams`, `TrapezoidLimits`, `Trajectory`, `ViaSamples`, `BSplinePoint` (field names as in Python): `auto [s, sd, sdd] = lab::cubic_time_scaling(T, t);` |
| `(N, n)` array, via points `(k, m)` | `Eigen::MatrixXd` with one **row** per sample / via; one coordinate = a `k x 1` matrix |
| robot dict, `tasks` dict | `lab::Robot` (fields `Slist, M, Mlist, Glist, g, qdot_max, tau_max, limits, q_home`), `lab::Tasks` (`tasks.B`, or `tasks.config("B")`) |
| `method=3`, `5`, `"trap"`; `T=None` | `lab::CUBIC`, `lab::QUINTIC`, `lab::TRAP`; pass any `T` (e.g. `0.0`) with `TRAP` |
| scalar limits broadcast in `trapezoid_from_limits` | one limit per joint: `Eigen::VectorXd::Constant(n, 1.3962634)` |
| `bspline_basis(u)` for a scalar or an array | you write the scalar version (returns `Eigen::Vector4d`); the vector version (4 × len(u)) is given |
| result dicts of `check_limits`, `min_time_scale` | `lab::LimitCheck` (`vel_ratio, tau_ratio, ok`), `lab::TimeScaleResult` (`c_vel, c_tau, c, binding`; `binding` is a `std::pair<std::string, int>`) |
| `raise ValueError(...)` | `throw std::invalid_argument("...")` |
| `np.clip`, `np.where`, `np.searchsorted`, `np.outer(a, b)` | `std::min(std::max(x, lo), hi)`, `if`/`else`, a loop over the via times, `a * b.transpose()` |
| `test_uniform_cubic_bspline_...` calls `scipy.interpolate.BSpline` | compares with SciPy's values stored in `cpp_cases.json` |
| `export_csv` writes 6 decimals | writes full double precision (same columns `t, q1..q6, qd1..qd6`) |
| plots with Matplotlib | CSV files written by `lab_tests` + the given `plot_results.py` |

A matrix-size mistake (e.g. `Q` stored as $n \times N$ instead of $N \times n$) does not crash the test program: it is reported as a failed test with the message "Eigen: wrong matrix size or index out of range".

**C++ troubleshooting.**

| Symptom | Likely cause | Fix |
|---|---|---|
| `cpp_support not found` | the course folder `cpp_support` is not next to / above your Lab code folder | copy it there, or configure with `-DSBEG_CPP_SUPPORT=<path>` |
| `No CMAKE_CXX_COMPILER could be found` | no C++ compiler installed | install it (see Requirements); on Windows run from the "x64 Native Tools Command Prompt" if Ninja cannot find `cl` |
| `cannot open file: .../cpp_cases.json` or `.../data/...` | Lab code folder moved after configuring | delete the `build` folder and configure again |
| B-spline basis or a time scaling is zero everywhere | integer division: `1/6 == 0`, `3/2 == 1` in C++ | write `1.0 / 6.0`, `3.0 / 2.0` |
| `joint_trajectory` shape check fails | `Q` built as $n \times N$, or `LinSpaced(N, 0.0, T)` arguments swapped | one row per sample; `Eigen::VectorXd::LinSpaced(N, low, high)` |
| `min_time_scale` binding joint off by one | printed the 0-based index | `binding.second` is 0-based (joint 4 is `3`), as in Python |
| Tests still show your old result | program not rebuilt | `cmake --build build` before `./build/lab_tests` |
