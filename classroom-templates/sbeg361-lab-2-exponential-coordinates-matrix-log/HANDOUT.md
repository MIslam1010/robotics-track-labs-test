---
title: "Lab 2 — Exponential coordinates, matrix log and IMU orientation"
subtitle: "SBEG 361 Introduction to Robotics — Week 3"
---

# Overview

| Item | Detail |
|---|---|
| Session | Session 4, 50 min in class + up to 2 h to finish (due before the Week 4 lab) |
| CLOs | 361-2 (represent and convert orientations: SO(3), exponential coordinates, quaternions); 361-7 (implement and validate against a reference library) |
| Files | `starter.py` (you edit), `test_lab.py` (auto-checks), `imu_knee.csv` (data), `make_data.py` (how the data were made) |
| Reference | `modern_robotics` (`VecToso3`, `so3ToVec`, `AxisAng3`, `MatrixExp3`, `MatrixLog3`), `scipy.spatial.transform.Rotation` |
| Reading | MR §3.2.2–3.2.3 (printed pp. 74–87), App. B.3 (pp. 583–584); Corke §3.2.3 *Inertial Navigation Systems* (printed pp. 53–55) |
| AI use | Allowed with disclosure (see Reflection) |
| Tier | 0 (simulation and recorded data only) |

The functions you write today go into your `sbeg361_kinematics` library; the Week 4–5 labs (SE(3), twists) build on them.

**Application context.**
*Biomedical:* gait laboratories estimate knee flexion from two IMUs on the thigh and shank instead of (or alongside) optical motion capture.
*Defence (protective/logistic):* the same two-IMU method estimates the knee angle of a soldier load-carriage exoskeleton during field trials, and an EOD robot's controller needs the orientation error between the current and the desired gripper orientation. The data set `imu_knee.csv` is **synthetic, with illustrative values**.

# Learning objectives

By the end of the lab you can:

1. Implement $[\omega]$, Rodrigues' formula (MR Eq. 3.51) and the three-case matrix-log algorithm (MR §3.2.3.3) and validate them against `modern_robotics`.
2. Convert between rotation matrices and unit quaternions (MR Eqs. B.10–B.12) and compose quaternions (MR Eq. B.15).
3. Integrate body-frame gyroscope data with the exponential update $R_{k+1} = R_k e^{[\omega_{b,k}]\Delta t}$ and explain why the first-order update leaves $SO(3)$.
4. Compute a joint angle and an orientation error from two orientations with the matrix log, and explain the effect of gyro bias.

# Pre-lab check (complete before the session; 5 questions)

1. Write $[\omega]$ for $\omega = (3,\ 0,\ -1)$.
2. For $\hat\omega = \hat z$ and $\theta = 90^\circ$, write $e^{[\hat\omega]\theta}$ without using Rodrigues' formula. Check that Rodrigues' formula gives the same result.
3. A rotation matrix has trace $2$. What is its rotation angle?
4. Which of $\omega_s$ or $\omega_b$ does a gyroscope strapped to the shank measure? Which MR equation relates it to $\dot R$?
5. Why do $q$ and $-q$ represent the same rotation? (One sentence using MR Eq. B.12.)

*Answers (for self-check):* 1. $\begin{bmatrix}0&1&0\\-1&0&-3\\0&3&0\end{bmatrix}$. 2. $\begin{bmatrix}0&-1&0\\1&0&0\\0&0&1\end{bmatrix}$. 3. $\cos\theta = \tfrac12$, $\theta = 60^\circ$. 4. $\omega_b$; MR Eq. (3.38), $R^{-1}\dot R = [\omega_b]$. 5. Every entry of (B.12) is quadratic in the $q_i$, so changing all signs leaves $R$ unchanged.

# Setup

```bash
cd "<your Lab code folder>"   # or your Classroom 50 repo
# activate the course Python environment (Docker image, JupyterHub or your own venv)
python -c "import numpy, scipy, modern_robotics; print('ok')"
LAB_IMPL=starter python -m pytest -q test_lab.py               # 17 failures (NotImplementedError) is expected
```

On Windows (PowerShell) use `$env:LAB_IMPL="starter"; python -m pytest -q test_lab.py`. Rules: do **not** call `modern_robotics`, `scipy.spatial.transform` or `scipy.linalg.expm/logm` inside `starter.py`; they are only for checking.

# Procedure

Run the tests after each step. A checkpoint is complete when all listed tests pass; ask a TA to sign it off.

## Step 1 — Skew-symmetric matrices (10 min)

Implement `vec_to_so3`, `so3_to_vec`, `axis_ang3`.

**Checkpoint 1:** `test_vec_to_so3_matches_mr`, `test_skew_properties`, `test_axis_ang3`. Be ready to explain why `test_skew_properties` checks $R[a]R^{\mathsf T} = [Ra]$ (MR Prop. 3.8).

## Step 2 — Matrix exponential (10 min)

Implement `matrix_exp3` with Rodrigues' formula. Return $I$ when $\theta <$ `TOL`.

**Checkpoint 2:** `test_matrix_exp3_mr_example_3_12` (reproduces MR Example 3.12) and `test_matrix_exp3_matches_mr_and_is_rotation`.

## Step 3 — Matrix logarithm (15 min)

Implement `matrix_log3` with all three cases of the MR algorithm. For case (b) try Eq. (3.58) first, then (3.59), then (3.60).

**Checkpoint 3:** `test_matrix_log3_matches_mr_random`, `test_matrix_log3_identity_and_pi_cases`, `test_exp_log_round_trip_small_and_large_angles`.
*Question for your report:* the round-trip test includes $\theta = 10^{-5}$ rad and $\theta = \pi - 10^{-4}$ rad. Which case of your algorithm handles each, and how large is the round-trip error? (Print it.)

## Step 4 — Quaternions (15 min)

Implement `rot_to_quat` (Eqs. B.10–B.11 when $q_0 > 0.5$, otherwise build $q$ from the log via B.9), `quat_to_rot` (B.12), `quat_mul` (B.15), `quat_conj`.
Remember that the printed Eq. (B.11) has a typo in its last entry: it must be $r_{21} - r_{12}$.

**Checkpoint 4:** `test_rot_to_quat_matches_scipy`, `test_quat_to_rot_round_trip`, `test_quat_mul_is_matrix_product`, `test_workshop_quaternion_example` (Workshop Problem 4b: $q_x\otimes q_y = \tfrac12(1,1,1,1)$).
Note: SciPy stores quaternions scalar-**last** $(x, y, z, w)$; MR puts the scalar first.

## Step 5 — IMU orientation from gyroscope data (25 min)

Load the data:

```python
import numpy as np, starter as lab
d = np.genfromtxt("imu_knee.csv", delimiter=",", names=True)    # 501 samples, 100 Hz, 5 s
w_th = np.column_stack([d["thigh_wx"], d["thigh_wy"], d["thigh_wz"]])[:-1]
R0 = lab.quat_to_rot([d["thigh_q0"][0], d["thigh_q1"][0], d["thigh_q2"][0], d["thigh_q3"][0]])
```

a. Implement `integrate_gyro_exp` and `integrate_gyro_euler`. Integrate the ideal thigh and shank gyro signals from the true initial orientation.
b. Plot the rotation error $\|\log(\hat R_k^{\mathsf T}R_k)\|$ (in degrees) of the exponential integrator against time, and $\det\hat R_k$ of the Euler integrator against time. Expected: the exponential error stays below $10^{-8}$ rad (the ideal gyro is exact for a zero-order hold); the Euler determinant reaches about $1.16$ (thigh) after 5 s.
c. Implement `orientation_error`, `rotation_angle` and `knee_flexion`. Compute the knee flexion angle from the true quaternions and plot it; the peak is about $56.7^\circ$.
d. Repeat with the **noisy** gyro columns (`*_nw*`: constant bias + white noise). The shank orientation drifts by about $3.0^\circ$ in 5 s and the knee-angle error reaches about $4.0^\circ$. Plot the knee angle from the noisy gyros on top of the truth.

**Checkpoint 5:** `test_gyro_exponential_integration_reproduces_truth`, `test_euler_integration_drifts_off_SO3`, `test_knee_flexion_from_quaternions`, `test_noisy_gyro_drift_is_bounded_but_nonzero`.

## Step 6 — Orientation error for an EOD gripper (5 min)

An EOD robot's gripper is at $R_{cur} = \mathrm{Rot}(\hat z, 40^\circ)\mathrm{Rot}(\hat y, -20^\circ)$ and must reach $R_{des} = R_{cur}\mathrm{Rot}(\hat x, 35^\circ)$ to align with a package's handle. Use `orientation_error` to find the required rotation in the gripper frame; you should get $(0.6109,\ 0,\ 0)$ rad, i.e. $35^\circ$ about the gripper's own $\hat x$-axis.

**Checkpoint 6:** `test_orientation_error_eod_gripper`. Finally run the whole suite: `LAB_IMPL=starter python -m pytest -q test_lab.py` must show **17 passed**.

# Deliverables (auto-checked notebook lab; not a formal report)

1. `starter.py` with all functions implemented; the full test suite passes.
2. A short notebook or PDF (≤ 3 pages) with: the two plots from Step 5b, the knee-angle plot from Steps 5c–5d, the round-trip errors from Step 3, and answers to the questions in Steps 3 and 5.
3. The reflection below.

Marking (lab category): tests 50 %, plots and answers 35 %, reflection 15 %. Be prepared for a 2-minute oral check: a TA will point to any line of your code and ask you to explain it.

# Reflection (≤ 200 words)

- Why does the exponential update stay on $SO(3)$ while the first-order update does not? What would you do on a microcontroller that cannot afford `sin`/`cos` every sample?
- The bias in the noisy gyro is only about $0.01$ rad/s, yet the knee angle is $4^\circ$ wrong after 5 s. What does this imply for using IMUs to measure joint angles over a 10-minute gait trial or an exoskeleton route march, and how do commercial systems correct it?
- **AI-use note (required):** state which AI tools you used (or "none"), what you asked, and what you kept or changed. You must be able to explain every line you submit.

# Troubleshooting

| Symptom | Likely cause and fix |
|---|---|
| `ModuleNotFoundError: starter` | Run pytest from inside your Lab code folder, or keep `test_lab.py` next to `starter.py`. |
| Every test fails with `NotImplementedError` | Expected until you implement the functions; set `LAB_IMPL=starter`, not `solution`. |
| `matrix_log3` returns `nan` | `arccos` received a value slightly outside $[-1, 1]$ because of rounding, or you divided by $\sin\theta \approx 0$; handle cases (a) and (b) before case (c). |
| π-case test fails for $\hat\omega = (1,0,0)$ | Equations (3.58) and (3.59) both divide by zero there; fall through to (3.60). |
| Quaternion test fails only by sign | You compared $q$ with $-q$; both are correct. Make $q_0 \ge 0$ in `rot_to_quat`. |
| Quaternions from SciPy look "shuffled" | SciPy order is $(x, y, z, w)$; MR order is $(q_0, q_1, q_2, q_3)$ with $q_0 = w$. |
| Gyro integration error is large | You pre-multiplied ($e^{[\omega]\Delta t}R_k$) a **body**-frame rate, or used all 501 rows (the last row of gyro data is padding; use `[:-1]`). |
| Knee angle has the wrong sign | Use $\log(R_{thigh}^{\mathsf T}R_{shank})$, not $\log(R_{shank}^{\mathsf T}R_{thigh})$. |

# C++ track

You may do this lab in C++17 instead of Python. The functions, data file, checkpoints and deliverables are the same; the auto-checks compare your results with `mrcpp` (the C++ port of `modern_robotics` in the course folder `cpp_support`), with the numbers of the Python reference (including the `scipy.spatial.transform` quaternions, stored in `cpp_cases.json`) and, for the IMU ground truth, with Eigen's own quaternion class, so both tracks are marked against identical values.

**Files** (in the `cpp` sub-folder of your Lab code folder): `starter.cpp` (your work), `lab.hpp` (declarations and documentation — do not change), `test_lab.cpp` (auto-checks, doctest), `cpp_cases.json` (reference numbers and the random test inputs of `test_lab.py`), `CMakeLists.txt`. The data file is the same `imu_knee.csv` (in your Lab code folder, next to the `cpp` folder). `cpp_support` (Course resources) must sit next to your lab folders or in a parent folder of them (the build finds it automatically up to 6 levels up).

**Requirements.** A C++17 compiler — Windows: Visual Studio Build Tools (2022 or 2026), workload "Desktop development with C++"; macOS: `xcode-select --install`; Ubuntu: `sudo apt install build-essential`. CMake and Ninja come with `pip install cmake ninja` (both are in the course `requirements.txt`).

**Build and run** (from your Lab code folder):

```bash
python run_lab.py 2 --cpp                      # configure, build, run, print Score: k/n
# or directly:
cmake -S cpp -B build -G Ninja
cmake --build build
./build/lab_tests                              # Windows: build\lab_tests.exe
./build/lab_tests -tc="*vec_to_so3*,*skew*,*axis_ang3*"   # only some tests (like pytest -k)
```

With the untouched starter, the program builds and all 17 test cases fail with `TODO: <function>`: this is expected. Rebuild (`cmake --build build`) after every change.

**Functions to write** (in `starter.cpp`, same order as in Python): `vec_to_so3`, `so3_to_vec`, `axis_ang3` (Step 1); `matrix_exp3` (Step 2); `matrix_log3` (Step 3); `rot_to_quat`, `quat_to_rot`, `quat_mul`, `quat_conj` (Step 4); `integrate_gyro_exp`, `integrate_gyro_euler`, `orientation_error`, `rotation_angle`, `knee_flexion` (Steps 5–6). Nothing is given except the constant `TOL` (in `lab.hpp`). As in Python, do not call `mrcpp` (`mr::...`) or Eigen's `Quaterniond` / `AngleAxisd` inside `starter.cpp`; they are only for checking.

**Checkpoints** (the C++ test cases have the same names as the Python tests, one test case per Python test; 17 in total):

| Checkpoint | Command |
|---|---|
| 1 | `./build/lab_tests -tc="*vec_to_so3*,*skew*,*axis_ang3*"` |
| 2 | `./build/lab_tests -tc="*matrix_exp3*"` |
| 3 | `./build/lab_tests -tc="*matrix_log3*,*exp_log_round_trip*"` |
| 4 | `./build/lab_tests -tc="*rot_to_quat*,*quat_to_rot*,*quat_mul*,*workshop*"` |
| 5 | `./build/lab_tests -tc="*gyro*,*euler*,*knee*"` |
| 6 | `./build/lab_tests -tc="*eod*"`, then `./build/lab_tests`: 17 of 17 test cases pass |

For the plots of Step 5, either use the Python track's code on your C++ results or write them as CSV with `sbeg::write_csv` (in `<sbeg/labio.hpp>`) and plot the CSV in Python; plotting is not part of the C++ auto-checks.

**Differences from the Python version.**

| Python | C++ |
|---|---|
| NumPy 3-vector, `(3, 3)` array | `Eigen::Vector3d`, `Eigen::Matrix3d` |
| quaternion `np.array([q0, q1, q2, q3])` | `Eigen::Vector4d q(q0, q1, q2, q3)`, scalar first as in MR (not `Eigen::Quaterniond`, which stores the scalar last) |
| `axis_ang3` returns a tuple | returns `std::pair<Eigen::Vector3d, double>`: `auto [w, theta] = lab::axis_ang3(v);` |
| `raise ValueError(...)` | `throw std::invalid_argument("...")` |
| gyro samples, array of shape `(N, 3)` | `Eigen::MatrixXd` with N rows and 3 columns; row k as a vector: `omega_b.row(k).transpose()` |
| result of shape `(N+1, 3, 3)` | `std::vector<Eigen::Matrix3d>` with N+1 entries (`push_back`) |
| `np.trace(R)`, `R.T`, `A @ B`, `np.linalg.norm(v)`, `np.cross(a, b)` | `R.trace()`, `R.transpose()`, `A * B`, `v.norm()`, `a.cross(b)` |
| `np.arccos`, `np.pi` | `std::acos`; a local `constexpr double PI = 3.14159265358979323846;` (`M_PI` is not standard C++) |
| `R[2, 1]` | `R(2, 1)` (also 0-based) |

The tests read `imu_knee.csv` themselves (columns by name, the last gyro row dropped as in Python's `[:-1]`); your functions receive Eigen matrices and do not read files. A matrix-size mistake does not crash the test program: it is reported as a failed test with the message "Eigen: wrong matrix size or index out of range".

**C++ troubleshooting.**

| Symptom | Likely cause | Fix |
|---|---|---|
| `cpp_support not found` | the course folder `cpp_support` is not next to / above your Lab code folder | copy it there, or configure with `-DSBEG_CPP_SUPPORT=<path>` |
| `No CMAKE_CXX_COMPILER could be found` | no C++ compiler installed | install it (see Requirements); on Windows run from the "x64 Native Tools Command Prompt" if Ninja cannot find `cl` |
| `cannot open file: .../imu_knee.csv` or `.../cpp_cases.json` | Lab code folder moved after configuring, or `imu_knee.csv` moved out of it | delete the `build` folder and configure again; keep `imu_knee.csv` next to the `cpp` folder |
| `matrix_log3` gives `nan` | `std::acos` of a value slightly outside $[-1, 1]$, or division by $\sin\theta \approx 0$ | handle cases (a) and (b) before case (c) |
| quaternion tests fail with `q` and the reference equal up to order | used `Eigen::Quaterniond` coefficients (`coeffs()` is $(x, y, z, w)$) | build the `Eigen::Vector4d` as $(q_0, q_1, q_2, q_3)$ yourself |
| `REQUIRE( Rs.size() == ... )` fails | the integrator did not return N+1 matrices | start with `out.push_back(R0)`, then one `push_back` per gyro row |
| Tests still show your old result | program not rebuilt | `cmake --build build` before `./build/lab_tests` |
