---
title: "Lab 1 — Rotations in NumPy and SO(3) property tests"
subtitle: "SBEG 361 Introduction to Robotics — Week 2"
---

**Session 4 · 50 min · Individual work, pairs may discuss · Tier 0 (simulation only)**
CLOs: 361-2 (represent orientations with SO(3)), 361-7 (implement and validate against a reference library).
Reading: MR §3.1–3.2.1 (printed pp. 60–74). Library: `modern_robotics` (`RotInv`, `MatrixExp3`, `VecToso3`, `ProjectToSO3`, `TestIfSO3`).

This lab starts your own kinematics library (`sbeg361_kinematics`). Later labs build on these functions: Lab 2 writes the matrix exponential that `rot_axis_angle` anticipates, and some later starters give them back as complete helpers under the names of that lab (Lab 5: `rot_axis(axis, angle)` = your `rot_x`/`rot_y`/`rot_z`; Lab 9: `rot(axis, angle)` = your `rot_axis_angle`, normalising the axis instead of raising).

## 1. Objectives

By the end of the lab you can:

1. build $\mathrm{Rot}(\hat x,\theta)$, $\mathrm{Rot}(\hat y,\theta)$, $\mathrm{Rot}(\hat z,\theta)$ and $\mathrm{Rot}(\hat\omega,\theta)$ in NumPy (MR p. 72);
2. test numerically whether a matrix belongs to SO(3) ($R^TR = I$, $\det R = 1$; MR Definition 3.1) and explain the tolerance you chose;
3. use a rotation matrix in its three roles: represent an orientation, change the frame of a vector, rotate a frame about a fixed or a body axis (MR Eqs. 3.22–3.24);
4. compose planar frames $(P,p)$ (MR Eqs. 3.8–3.9);
5. clean real-looking orientation data from a surgical optical tracker: reject invalid samples and project near-rotations onto SO(3).

## 2. Pre-lab check (complete before the session, 5 questions)

1. How many independent numbers does a $3\times3$ rotation matrix contain, and how many constraints does $R^TR = I$ impose?
2. Why is $R^TR = I$ not enough to define SO(3)? Give a $3\times3$ example that satisfies it but is not a rotation.
3. $R_{sb}$ is known. Which product rotates $\{b\}$ by $R$ about an axis **fixed in $\{s\}$**: $RR_{sb}$ or $R_{sb}R$?
4. Write $\mathrm{Rot}(\hat y,\theta)$. Where does the minus sign go?
5. A tracker returns entries with 6 decimals. Should `is_rotation` use tolerance $10^{-12}$ or $10^{-3}$? Why?

*Answers (self-check):* 1) 3 independent, 6 constraints. 2) It allows $\det R = -1$ (reflections), e.g. $\mathrm{diag}(1,1,-1)$. 3) $RR_{sb}$. 4) $\begin{bmatrix}c&0&s\\0&1&0\\-s&0&c\end{bmatrix}$ — the minus sign is in the bottom-left. 5) $10^{-3}$: rounding and sensor noise make exact equality impossible; $10^{-12}$ would reject every real sample (the `modern_robotics` function `TestIfSO3` also uses $10^{-3}$).

## 3. Setup

```bash
cd "<your Lab code folder>"          # or your GitHub Classroom repo
source ~/Documents/Edu/Lectures/Robotics/MR-Studio/.venv/bin/activate   # or the course Docker image / Colab
python -c "import numpy, modern_robotics; print('ok')"
LAB_IMPL=starter pytest -q         # all tests fail with NotImplementedError: this is expected
```

Files: `starter.py` (your work), `test_lab.py` (auto-checks), `data/tracker_orientations.csv` (20 tracker samples), `make_data.py` (how the data were generated — read it only *after* Part D).
On Windows PowerShell use `$env:LAB_IMPL="starter"; pytest -q`.

## 4. Procedure

### Part A — Elementary rotations (12 min)

Implement `rot_y`, `rot_z` (copy the pattern of the given `rot_x`) and `rot_axis_angle` using the explicit formula of MR p. 72. Raise `ValueError` for a non-unit axis.

**Checkpoint A** — `LAB_IMPL=starter pytest -q -k "elementary or axis_angle"` passes. Then, in a Python shell, verify by hand that `rot_axis_angle([0,0,1], th)` equals `rot_z(th)` and that $\mathrm{Rot}(\hat\omega,\theta) = \mathrm{Rot}(-\hat\omega,-\theta)$.

### Part B — SO(3) property tests (12 min)

Implement `so3_errors`, `is_rotation`, `rot_inv` (transpose — no `np.linalg.inv`) and `project_to_so3` (SVD; force $\det = +1$).

**Checkpoint B** — `LAB_IMPL=starter pytest -q -k "so3 or is_rotation or group or project"` passes. Answer in your notebook:
(i) what is `so3_errors(2*np.eye(3))` and why is the orthogonality error $\sqrt{27}$?
(ii) `test_group_properties` checks closure, inverse, non-commutativity and length preservation. Which MR propositions (3.3–3.6) does each assertion correspond to?

### Part C — The three uses of a rotation matrix (10 min)

Implement `change_frame`, `rotate_in_fixed`, `rotate_in_body` and `planar_compose`.

**Checkpoint C (biomedical)** — reproduce MR Figure 3.7: with $R_b = \mathrm{Rot}(\hat z, 90^\circ)$ and $p_a = (1,1,0)$, obtain $p_b = (1,-1,0)$ and $p_c = (0,-1,-1)$, and show that $R_c$ is obtained **both** as `rotate_in_body(R_b, rot_y(-pi/2))` and as `rotate_in_fixed(R_b, rot_x(pi/2))`.

**Checkpoint C (defence/humanitarian)** — an EOD robot gripper has $R_{sb} = \mathrm{Rot}(\hat z, 30^\circ)\mathrm{Rot}(\hat y, 45^\circ)$. The arm's base joint pans the whole arm $90^\circ$ about the vertical $\hat z_s$ and the wrist rolls $60^\circ$ about the gripper's own $\hat x_b$. Compute both new orientations with your functions. Which column of $R_{sb}$ is unchanged by the wrist roll, and why? (`test_eod_base_pan_and_wrist`.)

### Part D — Application: cleaning surgical-tracker data (12 min)

An optical tracker reports the orientation $R_{sm}$ of a marker on a laparoscopic instrument (`data/tracker_orientations.csv`, illustrative values). Some samples are corrupted.

1. Load the data with `load_tracker_csv`, implement `classify_samples`, and print `so3_errors` for each sample. Which samples are invalid, and for each one say **which** condition fails (orthogonality, unit norm, or determinant).
2. Project the valid samples with `project_to_so3` and report the largest orthogonality error before and after.
3. The marker-to-tip calibration is $R_{mt} = \mathrm{Rot}(\hat x, 30^\circ)$. Implement `tool_orientations` and compute $R_{st}$ for the valid samples. Explain your multiplication order with the subscript-cancellation rule.
4. Should a reflected sample ($\det = -1$) be projected and used? Discuss in one sentence.

**Checkpoint D** — `LAB_IMPL=starter pytest -q` passes completely (18 tests).

## 5. Deliverables (upload to GitHub Classroom by the start of next week's lab)

- `starter.py` completed (all tests passing) — auto-checked.
- `lab1_notes.md` (or a notebook): answers to checkpoints A–D, the table of invalid samples with the failing condition, and the before/after errors from Part D.
- Reflection (≤ 200 words), including the AI-use note below.

**Reflection prompt.** Where could an orientation error of the type found in Part D harm a patient in image-guided surgery, or put an operator at risk during EOD work? What test would you add to a medical-device software pipeline to catch it?

**AI-use note (required).** AI tools are allowed for this lab code **with disclosure**: state the tool, the prompts you used and what you kept. You must be able to explain every line in the oral check. Write "No AI tools used" if that is the case.

## 6. Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `ModuleNotFoundError: starter` | pytest run from another folder | `cd` into your Lab code folder first |
| All tests pass but you changed nothing | `LAB_IMPL` not set, so the tests use `solution` | Set `LAB_IMPL=starter` |
| `rot_y` fails only for some angles | sign of $\sin$ in the wrong corner | compare with MR p. 72 |
| `test_project_to_so3` fails for the reflection | you returned $UV^T$ with $\det = -1$ | multiply by $\mathrm{diag}(1,1,\det(UV^T))$ |
| `is_rotation` rejects every tracker sample | tolerance too tight | use `tol=1e-3` |
| Angles look wrong by a factor of ~57 | degrees passed to `np.cos` | convert with `np.deg2rad` |
| `ValueError` from `rot_axis_angle` | axis not normalised | divide by `np.linalg.norm(w)` before calling |

## 7. C++ track

You may do this lab in C++17 instead of Python. The functions, data file, checkpoints and deliverables are the same; the auto-checks compare your results with `mrcpp` (the C++ port of `modern_robotics` in the course folder `cpp_support`) and with the numbers of the Python reference, so both tracks are marked against identical values.

**Files** (in the `cpp` sub-folder of your Lab code folder): `starter.cpp` (your work), `lab.hpp` (declarations and documentation — do not change), `test_lab.cpp` (auto-checks, doctest), `cpp_cases.json` (reference numbers), `CMakeLists.txt`. The data file is the same `data/tracker_orientations.csv`. `cpp_support` (Course resources) must sit next to your lab folders or in a parent folder of them (the build finds it automatically up to 6 levels up).

**Requirements.** A C++17 compiler — Windows: Visual Studio Build Tools (2022 or 2026), workload "Desktop development with C++"; macOS: `xcode-select --install`; Ubuntu: `sudo apt install build-essential`. CMake and Ninja come with `pip install cmake ninja` (both are in the course `requirements.txt`).

**Build and run** (from your Lab code folder):

```bash
python run_lab.py 1 --cpp                      # configure, build, run, print Score: k/n
# or directly:
cmake -S cpp -B build -G Ninja
cmake --build build
./build/lab_tests                              # Windows: build\lab_tests.exe
./build/lab_tests -tc="*elementary*,*axis_angle*"   # only some tests (like pytest -k)
```

With the untouched starter, the program builds and all 13 test cases fail with `TODO: <function>`: this is expected. Rebuild (`cmake --build build`) after every change.

**Functions to write** (in `starter.cpp`, same order as in Python): `rot_y`, `rot_z`, `rot_axis_angle` (Part A); `so3_errors`, `is_rotation`, `rot_inv`, `project_to_so3` (Part B); `change_frame`, `rotate_in_fixed`, `rotate_in_body`, `planar_compose` (Part C); `classify_samples`, `tool_orientations` (Part D). `rot_x` and `load_tracker_csv` are given.

**Checkpoints.** A: `./build/lab_tests -tc="*elementary*,*axis_angle*"`; B: `-tc="*so3*,*is_rotation*,*group*,*project*"`; C: `-tc="*figure_3_7*,*eod*,*planar*"`; D: all 13 test cases pass (`./build/lab_tests`). The Python track counts 18 tests because pytest counts each angle of `test_elementary_rotations_match_mr` separately; the C++ track checks the same angles in one test case.

**Differences from the Python version.**

| Python | C++ |
|---|---|
| NumPy `(3, 3)` array, 3-vector | `Eigen::Matrix3d`, `Eigen::Vector3d` (planar: `Eigen::Matrix2d`, `Eigen::Vector2d`) |
| `raise ValueError(...)` | `throw std::invalid_argument("...")` |
| `so3_errors` returns a tuple | returns `std::pair<double, double>`: `auto [e, d] = lab::so3_errors(R);` |
| `(N, 3, 3)` array of samples, boolean array | `std::vector<Eigen::Matrix3d>`, `std::vector<bool>` |
| `np.linalg.norm(A)` (Frobenius), `np.linalg.det`, `A.T`, `A @ B` | `A.norm()`, `A.determinant()`, `A.transpose()`, `A * B` |
| `U, S, Vt = np.linalg.svd(M)` | `Eigen::JacobiSVD<Eigen::Matrix3d>` (hint in `starter.cpp`); `svd.matrixU()` is $U$ and `svd.matrixV()` is $V$, **not** $V^T$ |
| `np.pi` | a local `constexpr double PI = 3.14159265358979323846;` (`M_PI` is not standard C++) |

`so3_errors` and `is_rotation` take an `Eigen::MatrixXd` (any size) so that a wrong size can be detected. A matrix-size mistake (e.g. multiplying a $3\times3$ by a 2-vector) does not crash the test program: it is reported as a failed test with the message "Eigen: wrong matrix size or index out of range".

**C++ troubleshooting.**

| Symptom | Likely cause | Fix |
|---|---|---|
| `cpp_support not found` | the course folder `cpp_support` is not next to / above your Lab code folder | copy it there, or configure with `-DSBEG_CPP_SUPPORT=<path>` |
| `No CMAKE_CXX_COMPILER could be found` | no C++ compiler installed | install it (see Requirements); on Windows run from the "x64 Native Tools Command Prompt" if Ninja cannot find `cl` |
| `cannot open file: .../cpp_cases.json` | Lab code folder moved after configuring | delete the `build` folder and configure again |
| `project_to_so3` wrong only for some samples | used `svd.matrixV()` as if it were $V^T$ | transpose it |
| Tests still show your old result | program not rebuilt | `cmake --build build` before `./build/lab_tests` |
