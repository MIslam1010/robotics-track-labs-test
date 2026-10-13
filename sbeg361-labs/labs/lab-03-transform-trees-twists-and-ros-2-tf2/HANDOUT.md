---
title: "Lab 3 — Transform trees, twists and ROS 2 tf2 visualisation"
subtitle: "SBEG 361 Introduction to Robotics — Week 4"
---

# Overview

| Item | Detail |
|---|---|
| Session | Session 4, 50 min in class + up to 2 h to finish (due before the Week 5 lab) |
| CLOs | 361-2 (compose poses in SE(3)); 361-3 (twists, screw axes, adjoint); 361-7 (implement and validate against a reference library and a middleware tool) |
| Part A (auto-tested) | Pure-Python transform tree and twist tools: `starter.py` (you edit), `test_lab.py` (19 auto-checks) |
| Part B (guided) | ROS 2 Jazzy tf2 + RViz: `ros2/nav_tf_broadcaster.py` publishes the same tree; you inspect it with `tf2_echo`, `view_frames` and RViz |
| Data | `nav_tree.json`, `tool_track.csv`, `eod_tree.json`; `make_data.py` shows how they were made (read it **after** the lab) |
| Reference | `modern_robotics` (`RpToTrans`, `TransInv`, `VecTose3`, `Adjoint`, `MatrixLog6`); ROS 2 `tf2_ros` |
| Reading | MR §3.3.1–3.3.2 (printed pp. 87–102): Eqs. 3.62–3.67, Example 3.19, Prop. 3.22, Example 3.23, Def. 3.24. Optional: MLS Ch. 2 §3.1 and §3.3 (pp. 36–39, 45–50; MLS orders twists $(v,\omega)$) |
| AI use | Allowed with disclosure (see Reflection) |
| Tier | 0 (simulation and recorded data only) |

The functions you write today join your `sbeg361_kinematics` library. Lab 4 (force–torque sensor frame change) reuses `adjoint`.

**Application context.**
*Biomedical:* in navigated spine surgery an optical tracker follows marker arrays on the patient's vertebra and on the screwdriver. The navigation software keeps a transform tree (tracker → patient reference → CT image → planned trajectory) and shows the tool tip on the CT in real time. You will reproduce that computation and, from the tracked screwdriver poses alone, recover the screw's **thread lead** and the **angular and positional deviation** from the surgical plan.
*Defence (protective use):* an explosive-ordnance-disposal (EOD) robot's software keeps the same kind of tree (odometry → base → mast camera → suspect package; base → gripper) to tell the operator where the package is relative to the gripper (Workshop Problem 3).
All data are **synthetic, with illustrative values**.

# Learning objectives

By the end of the lab you can:

1. Implement $T^{-1}$ (MR Eq. 3.64), the action of $T$ on points (Eq. 3.65), $[\mathcal V]$ (Eq. 3.71) and $[\mathrm{Ad}_T]$ (Def. 3.20), and validate them against `modern_robotics`.
2. Build a tf2-style transform tree and answer "where is frame $\{x\}$ seen from frame $\{y\}$?" for any two frames, reproducing MR Example 3.19.
3. Extract body and spatial twists from a sampled pose trajectory ($[\mathcal V_b] = T^{-1}\dot T$, $[\mathcal V_s] = \dot TT^{-1}$) and interpret them as a screw $\{q,\hat s,h\}$.
4. Publish and inspect a transform tree in ROS 2 (tf2, RViz) and relate tf2's conventions to MR notation.

# Pre-lab check (complete before the session; 5 questions)

1. Compute $T^{-1}$ for $T = (\mathrm{Rot}(\hat z,90^\circ),\ (1,0,0))$.
2. A tf2 message has `header.frame_id = "tracker"` and `child_frame_id = "tool_marker"`. Which MR matrix does it carry: $T_{\text{tracker},\text{tool\_marker}}$ or its inverse?
3. Write the $4\times4$ matrix $[\mathcal V]$ for $\mathcal V = (0, 0, 1, 2, 0, 0)$.
4. What is the pitch of $\mathcal V = (0, 0, 2, 0, 0, 1)$?
5. tf2 and SciPy store quaternions as $(x, y, z, w)$. In which order does MR App. B write them?

*Answers (for self-check):* 1. $(\mathrm{Rot}(\hat z,-90^\circ),\ (0, 1, 0))$, i.e. $\begin{bmatrix}0&1&0&0\\-1&0&0&1\\0&0&1&0\\0&0&0&1\end{bmatrix}$. 2. $T_{\text{tracker},\text{tool\_marker}}$: the pose of the child in the parent. 3. $\begin{bmatrix}0&-1&0&2\\1&0&0&0\\0&0&0&0\\0&0&0&0\end{bmatrix}$. 4. $h = \omega^{\mathsf T}v/\|\omega\|^2 = 2/4 = 0.5$ m/rad. 5. Scalar first, $(q_0, q_1, q_2, q_3) = (w, x, y, z)$.

# Setup

```bash
cd "<your Lab code folder>"   # or your Classroom 50 repo
source ~/Documents/Edu/Lectures/Robotics/MR-Studio/.venv/bin/activate   # or the course Docker image / JupyterHub
python -c "import numpy, scipy, modern_robotics; print('ok')"
LAB_IMPL=starter python -m pytest -q test_lab.py               # 19 failures (NotImplementedError) is expected
```

On Windows (PowerShell) use `$env:LAB_IMPL="starter"; python -m pytest -q test_lab.py`. Rules: do **not** call `modern_robotics`, `scipy.spatial.transform` or `np.linalg.inv` inside `starter.py`; they are only for checking. The helpers already written in `starter.py` (`vec_to_so3`, `so3_to_vec`, the loaders, `_ancestors`, `_common`, `path`, `axis_angle_between`, `point_line_distance`) may be used.

**Tree convention (same as tf2).** An edge `parent → child` stores $T_{\text{parent},\text{child}}$. `tree.lookup(target, source)` returns $T_{\text{target},\text{source}}$, the pose of `source` seen from `target`. This is the argument order of tf2's `buffer.lookup_transform(target, source, time)` and of `ros2 run tf2_ros tf2_echo target source`.

# Part A — Procedure (auto-tested)

Run the tests after each step. A checkpoint is complete when all listed tests pass; ask a TA to sign it off.

## Step 1 — SE(3) basics (8 min)

Implement `rp_to_trans`, `trans_to_rp`, `trans_inv`, `is_se3`, `transform_point`.

**Checkpoint 1:** `test_rp_to_trans_and_back`, `test_trans_inv_matches_mr`, `test_is_se3`, `test_mr_fig_3_14_frames_and_point` (MR Fig. 3.14: $T_{bc} = T_{sb}^{-1}T_{sc}$ and $v_b = (0,0,1.5) \mapsto v_s = (1.5,-2,0)$), `test_mr_fig_3_15_fixed_vs_body_frame`, `test_distance_preserved`.

## Step 2 — se(3) and the adjoint (5 min)

Implement `vec_to_se3`, `se3_to_vec`, `adjoint`.

**Checkpoint 2:** `test_vec_to_se3_matches_mr`, `test_adjoint_matches_mr_and_properties` (checks MR Eqs. 3.77–3.78 and $[\mathrm{Ad}_T\mathcal V] = T[\mathcal V]T^{-1}$).

## Step 3 — Twists and screws (10 min)

Implement `twists_from_T_Tdot` (MR Eqs. 3.81–3.82), `screw_to_twist` and `twist_to_screw` (MR §3.3.2.2, Def. 3.24). In `twist_to_screw` return the axis point closest to the origin, $q = \omega\times v/\|\omega\|^2$ (MLS Eq. 2.43, p. 47), $h = \infty$ and `q = None` for a pure translation, and raise `ValueError` for the zero twist.

**Checkpoint 3:** `test_twists_from_T_Tdot_mr_example_3_23` (MR Example 3.23: $\mathcal V_s = (0,0,2,-2,-4,0)$, $\mathcal V_b = (0,0,-2,2.8,4,0)$), `test_screw_to_twist_and_back` (the car's screw axis passes through $q = (2,-1,0)$ with $h = 0$), `test_pure_translation_and_zero_twist`, `test_pitch_is_frame_invariant`.

## Step 4 — The transform tree (12 min)

Implement `quat_xyzw_to_rot` (reuse Lab 2's `quat_to_rot`: paste it into `starter.py` as a private helper or write MR Eq. B.12 inline, since this starter does not contain it; mind the order $(x,y,z,w)$) and the methods `TransformTree.add`, `TransformTree._to_ancestor` and `TransformTree.lookup`.

- `add` must reject a second parent for the same child, a cycle, `parent == child`, and a matrix that is not in SE(3) (`ValueError`). Re-adding the **same** edge updates it; this is how the tracked tool will move.
- `_to_ancestor(frame, ancestor)` walks up from `frame` and multiplies on the **left**: $T_{\text{anc},f} = T_{\text{anc},\ldots}\cdots T_{\text{parent}(f),f}$.
- `lookup(target, source)` $= T_{\text{common},\text{target}}^{-1}\,T_{\text{common},\text{source}}$, where `common = self._common(target, source)` is their lowest common ancestor.

**Checkpoint 4:** `test_tree_mr_example_3_19` (builds the tree $a\to d$, $d\to b$, $d\to e$, $b\to c$ of MR Example 3.19 and must return MR's $T_{ce}$, p. 95; path `c → b → d → e`), `test_tree_rejects_bad_edges`, `test_quaternion_order_and_json_loader`, `test_eod_tree_matches_workshop_problem_3`.
For the EOD tree, also print `eod.lookup("odom", "package")[:3, 3]`; you should get $(6.262,\ 2.613,\ 0.3)$ m.

## Step 5 — Navigated pedicle-screw insertion (15 min)

```python
import numpy as np, starter as lab
nav = lab.load_tree_json("nav_tree.json")        # theatre, tracker, patient_ref, ct_image, plan_entry, tool_tip
t, Ts = lab.load_track_csv("tool_track.csv")      # 401 poses T_tracker_tool_marker, 50 Hz, 8 s
nav.add("tracker", "tool_marker", Ts[0])          # update this edge for each sample k
```

a. **Tool tip on the CT.** Compute `nav.lookup("ct_image", "tool_tip")` at $t = 0$. The tip is at $(142.0,\ 84.03,\ 210.26)$ mm in image coordinates. Express it in the plan frame (`lookup("plan_entry", "tool_tip")`): it is $1.00$ mm from the planned entry point. Write down the chain of frames that `path` returns and explain why the theatre frame is not needed.
b. **Depth along the plan.** Loop over all samples, update the `tracker → tool_marker` edge and record the tip in the plan frame. Plot its $z$ (depth along the planned trajectory) and its lateral distance $\sqrt{x^2+y^2}$ against time. At $t = 8$ s you should read $(0,\ 0.826,\ 4.997)$ mm: 5.0 mm deep, and the lateral error has changed from 1.00 to 0.83 mm. Why does a straight insertion change the lateral error?
c. **Twists from the tracker.** Implement `twists_from_samples` (central differences, then take the skew-symmetric part of the rotation block). Plot the six components of $\mathcal V_b$ and of $\mathcal V_s$ against time. $\mathcal V_b$ is constant, about $(0,\ 0,\ -1.5705,\ 0,\ 0.07853,\ -0.000625)$, while $\mathcal V_s$ (in tracker coordinates) is also constant. Explain both facts using MR Prop. 3.22 and the fact that the screwdriver performs a constant screw motion.
d. **Screw parameters.** Apply `twist_to_screw` to the mean $\mathcal V_b$: $\dot\theta \approx 1.5705$ rad/s, $\hat s = (0,0,-1)$ (the shaft, in the marker frame), $q_b = (0.05,\ 0,\ 0)$ m. Convert the pitch to a **thread lead** in mm per turn ($2\pi h$); you should get $2.500$ mm. The exact speed is $\pi/2 = 1.5708$ rad/s. Explain the $1.6\times10^{-4}$ relative error of the central difference.
e. **Deviation from the plan.** Transform the mean spatial twist into the CT frame, $\mathcal V_{ct} = [\mathrm{Ad}_{T_{ct,\text{tracker}}}]\mathcal V_{\text{tracker}}$, find its screw axis, and compare it with the plan's $\hat z$-axis (`lookup("ct_image", "plan_entry")`): the angle between the axes is $2.00^\circ$ and the planned entry point is $1.00$ mm from the actual axis. Would you accept this screw? Typical clinical grading tolerates a few millimetres of cortical breach; state your criterion.

**Checkpoint 5:** `test_tool_tip_in_ct_at_start`, `test_body_twist_from_tracking_data`, `test_screw_axis_and_lead_of_the_screwdriver`. Finally run the whole suite: `LAB_IMPL=starter python -m pytest -q test_lab.py` must show **19 passed**.

# Part B — ROS 2 tf2 and RViz (guided, not auto-tested)

Use the course ROS 2 Jazzy Docker image or an Ubuntu 24.04 machine with ROS 2 Jazzy (Course syllabus §11 Software setup). If ROS 2 is unavailable, complete Part A and answer B3 and B6 on paper; a TA will run the demo once for the group.

**B1 — Publish the tree.**

```bash
source /opt/ros/jazzy/setup.bash
cd ros2 && python3 nav_tf_broadcaster.py        # static edges on /tf_static, tool replayed on /tf at 50 Hz
ros2 topic echo /tf_static --once                    # second terminal: inspect one message
```

Find the `patient_ref → ct_image` entry and check that its translation equals the one in `nav_tree.json`.

**B2 — RViz.** Run `rviz2`, set *Global Options → Fixed Frame* to `theatre`, *Add → TF*, and tick *Show Names*. You should see the tracker above the table and the tool marker turning about the screwdriver shaft. Switch the Fixed Frame to `ct_image`, then to `tool_marker`. What changes on the screen, and what does not? (Screenshot for your report.)

**B3 — Conventions.** Read `nav_tf_broadcaster.py`. Answer the `EXERCISE B3` comment: why must `tool_marker`'s parent be `tracker`, and what would tf2 do if a second node published `patient_ref → tool_marker` as well? (Compare with your `add` method's `ValueError`.) Why does the script convert the rotation to a quaternion in $(x,y,z,w)$ order?

**B4 — Compare tf2 with your tree.**

```bash
ros2 run tf2_ros tf2_echo patient_ref plan_entry     # static: must match your lookup to 1e-6 m
ros2 run tf2_ros tf2_echo plan_entry tool_tip        # dynamic: depth z grows 0 -> 5 mm, then the replay restarts
```

Compare the first output with `nav.lookup("patient_ref", "plan_entry")` (translation, and the rotation after converting your $R$ to a quaternion). For the second, check that the $z$-translation stays within $0$–$5$ mm and $y$ between $0.826$ and $1.000$ mm (the values of Part A, step b).

**B5 — The tree as a picture.** Run `ros2 run tf2_tools view_frames`. It writes a PDF of the tree. Compare it with `nav.path("ct_image", "tool_tip")`.

**B6 — Add a frame from the command line.** A needle-guide tip 50 mm above the planned entry point:

```bash
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z -0.05 --qx 0 --qy 0 --qz 0 --qw 1 \
    --frame-id plan_entry --child-frame-id guide_tip
```

Explain from the plan frame's definition (+$\hat z$ points into the bone) why $z = -0.05$ is *above* the entry point. Then add the same edge to your Python tree and check `lookup("ct_image", "guide_tip")` against `tf2_echo ct_image guide_tip`.

**B7 (optional) — A listener node.** Write a 15-line `rclpy` node with a `tf2_ros.Buffer` and `TransformListener` that prints the tip depth along the plan (`buffer.lookup_transform("plan_entry", "tool_tip", rclpy.time.Time())`) at 5 Hz.

# Deliverables (auto-checked notebook lab; not a formal report)

1. `starter.py` with all functions implemented; the full test suite passes.
2. A short notebook or PDF (≤ 3 pages): the two plots from Step 5b, the twist plots from Step 5c, the numbers and answers from Steps 5a–5e, and one RViz screenshot with answers to B2–B6 (or the paper answers to B3 and B6).
3. The reflection below.

Marking (lab category): tests 50 %, plots and answers 35 %, reflection 15 %. Be prepared for a 2-minute oral check: a TA will point to any line of your code and ask you to explain it.

# Reflection (≤ 200 words)

- Your tree stores one parent per frame, as tf2 does. What would go wrong in surgery if two different registrations of the same CT were both allowed into the tree?
- The twist from tracking data is only as good as the tracker. If each pose had 0.1 mm of noise, which of $\dot\theta$, $h$ and $q$ would you expect to suffer most from central differencing, and how could you reduce the effect?
- In the EOD tree, which edges come from measurement and which from calibration or design? Which one would you re-check first if the gripper missed the package by 5 cm?
- **AI-use note (required):** state which AI tools you used (or "none"), what you asked, and what you kept or changed. You must be able to explain every line you submit.

# Troubleshooting

| Symptom | Likely cause and fix |
|---|---|
| `ModuleNotFoundError: starter` | Run pytest from inside your Lab code folder, or keep `test_lab.py` next to `starter.py`. |
| Every test fails with `NotImplementedError` | Expected until you implement the functions; set `LAB_IMPL=starter`, not `solution`. |
| `trans_inv` test fails only in the position column | You wrote $-p$ or $-Rp$; Eq. (3.64) needs $-R^{\mathsf T}p$. |
| Example 3.19 lookup is the inverse of MR's answer | Arguments swapped: `lookup(target, source)` returns $T_{\text{target},\text{source}}$, so $T_{ce}$ is `lookup("c", "e")`. |
| `_to_ancestor` gives wrong rotations for chains of length ≥ 2 | You multiplied on the right; walking **up** the tree, each parent transform goes on the **left**. |
| Quaternion test fails, matrix looks transposed or permuted | You read $(x,y,z,w)$ as $(w,x,y,z)$. tf2/SciPy are scalar-last; MR is scalar-first. |
| `twists_from_samples` returns non-constant $\mathcal V_b$ | You used a forward difference with $T_k$ instead of the central difference about $T_k$, or you mixed up $T^{-1}\dot T$ ($\mathcal V_b$) and $\dot TT^{-1}$ ($\mathcal V_s$). |
| Lead comes out as 15.7 mm | You multiplied $h$ by $2\pi$ twice, or used $\|v\|$ instead of $\omega^{\mathsf T}v/\|\omega\|^2$. |
| RViz shows "No transform from [x] to [theatre]" | The broadcaster is not running, or the Fixed Frame name is misspelled; check with `ros2 run tf2_ros tf2_monitor`. |
| `tf2_echo` waits forever for the tool | The dynamic edge is only published while `nav_tf_broadcaster.py` runs; static edges are latched on `/tf_static`. |

# C++ track

You may do Part A of this lab in C++17 instead of Python. The functions, data files, checkpoints and deliverables are the same; the auto-checks compare your results with `mrcpp` (the C++ port of `modern_robotics` in the course folder `cpp_support`), with the worked examples of MR §3.3 and with the numbers of the Python reference, so both tracks are marked against identical values. The plots and written answers of Step 5 may be produced with the Python helpers (`load_tree_json`, `load_track_csv`) or from your C++ results; only the auto-checks differ.

**Files** (in the `cpp` sub-folder of your Lab code folder): `starter.cpp` (your work), `lab.hpp` (declarations and documentation — do not change), `test_lab.cpp` (auto-checks, doctest), `cpp_cases.json` (reference numbers), `CMakeLists.txt`. The data files are the same `nav_tree.json`, `eod_tree.json` and `tool_track.csv` of the Lab code folder. `cpp_support` (Course resources) must sit next to your lab folders or in a parent folder of them (the build finds it automatically up to 6 levels up). Optional for Part B: `ros2/cpp/` holds an `rclcpp` version of `nav_tf_broadcaster.py` (see below).

**Requirements.** A C++17 compiler — Windows: Visual Studio Build Tools (2022 or 2026), workload "Desktop development with C++"; macOS: `xcode-select --install`; Ubuntu: `sudo apt install build-essential`. CMake and Ninja come with `pip install cmake ninja` (both are in the course `requirements.txt`).

**Build and run** (from your Lab code folder):

```bash
python run_lab.py 3 --cpp                      # configure, build, run, print Score: k/n
# or directly:
cmake -S cpp -B build -G Ninja
cmake --build build
./build/lab_tests                              # Windows: build\lab_tests.exe
./build/lab_tests -tc="*trans_inv*,*is_se3*"   # only some tests (like pytest -k)
```

With the untouched starter, the program builds and all 19 test cases fail with `TODO: <function>`: this is expected. Rebuild (`cmake --build build`) after every change.

**Functions to write** (in `starter.cpp`, same order as in Python): `rp_to_trans`, `trans_to_rp`, `trans_inv`, `is_se3`, `transform_point`, `transform_points` (Step 1); `vec_to_se3`, `se3_to_vec`, `adjoint` (Step 2); `twists_from_T_Tdot`, `screw_to_twist`, `twist_to_screw` (Step 3); `quat_xyzw_to_rot` and the `TransformTree` methods `add`, `_to_ancestor`, `lookup` (Step 4); `twists_from_samples` (Step 5c). Given: `vec_to_so3`, `so3_to_vec`, `TransformTree::frames`, `_ancestors`, `_common`, `path`, `transform_from_tf`, `load_tree_json`, `load_track_csv`, `axis_angle_between`, `point_line_distance`. Do not use `mrcpp` or Eigen's general `.inverse()` inside `starter.cpp`.

**Checkpoints.** 1: `./build/lab_tests -tc="*rp_to_trans*,*trans_inv*,*is_se3*,*fig_3_1*,*distance*"`; 2: `-tc="*vec_to_se3*,*adjoint*"`; 3: `-tc="*Tdot*,*screw_to_twist*,*pure_translation*,*pitch*"`; 4: `-tc="*tree_*,*quaternion*,*eod*"`; 5: all 19 test cases pass (`./build/lab_tests`). The C++ test cases have the same names as the 19 Python tests.

**Differences from the Python version.**

| Python | C++ |
|---|---|
| NumPy `(4, 4)` array, 3-vector, 6-vector twist | `Eigen::Matrix4d`, `Eigen::Vector3d`, `lab::Vector6d` (6×6 adjoint: `lab::Matrix6d`) |
| quaternion list `[x, y, z, w]` | `Eigen::Vector4d(x, y, z, w)` — still scalar last |
| `transform_point(T, x)` with `x` of shape `(N, 3)` | a separate function `transform_points(T, X)`, `X` an N×3 `Eigen::MatrixXd` (one point per row) |
| `trans_to_rp`, `twists_from_T_Tdot` return a tuple | `std::pair`: `auto [R, p] = lab::trans_to_rp(T);` |
| `(N, 4, 4)` pose array; `(N-2, 6)` twist arrays | `std::vector<Eigen::Matrix4d>`; `Eigen::MatrixXd` with one twist per row |
| `twist_to_screw` returns a dict | returns a `lab::Screw` struct with the same field names: `sc.S`, `sc.thetadot`, `sc.h`, `sc.s_hat`, `sc.q` |
| `q = None` (pure translation) | `sc.q` is a `std::optional<Eigen::Vector3d>`: empty for a pure translation (`sc.q.reset()`); read it as `*sc.q` |
| `np.inf` | `lab::INF` (`std::numeric_limits<double>::infinity()`); test with `std::isinf(h)` |
| `screw_to_twist(None, s, np.inf, 0.05)` | pass any `q`, e.g. `Eigen::Vector3d::Zero()`: it is ignored when `h` is infinite |
| `raise ValueError(...)` | `throw std::invalid_argument("...")` |
| `KeyError` / `LookupError` (given `_common`) | `lab::KeyError` / `lab::LookupError` (declared in `lab.hpp`) |
| `tree.frames` (property), `self.parent`, `self.T` (dicts) | `tree.frames()` (a `std::set`), `parent`, `T` (`std::map`; read with `.at(name)`) |
| `np.linalg.inv` (forbidden in `trans_inv`) | `.inverse()` (equally forbidden) |

`is_se3` takes an `Eigen::MatrixXd` (any size) so that a wrong size can be detected. A matrix-size mistake (e.g. a 4×4 times a 3-vector) does not crash the test program: it is reported as a failed test with the message "Eigen: wrong matrix size or index out of range".

**Part B in C++ (optional, not auto-tested).** `ros2/cpp/` is a small `ament_cmake` package (`sbeg361_nav_tf`) whose node `nav_tf_broadcaster` publishes exactly what `nav_tf_broadcaster.py` publishes; the B3 question is marked in its source as well. In the ROS 2 Jazzy environment:

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/ros2_ws/src && ln -s "<your Lab code folder>/ros2/cpp" ~/ros2_ws/src/sbeg361_nav_tf
cd ~/ros2_ws && colcon build --packages-select sbeg361_nav_tf && source install/setup.bash
ros2 run sbeg361_nav_tf nav_tf_broadcaster --ros-args -p lab_dir:="<your Lab code folder>"
```

Then continue with B1–B6 as written above (B7 may be written with `rclcpp`, `tf2_ros::Buffer` and `tf2_ros::TransformListener`).

**C++ troubleshooting.**

| Symptom | Likely cause | Fix |
|---|---|---|
| `cpp_support not found` | the course folder `cpp_support` is not next to / above your Lab code folder | copy it there, or configure with `-DSBEG_CPP_SUPPORT=<path>` |
| `No CMAKE_CXX_COMPILER could be found` | no C++ compiler installed | install it (see Requirements); on Windows run from the "x64 Native Tools Command Prompt" if Ninja cannot find `cl` |
| `cannot open file: .../cpp_cases.json` or `.../nav_tree.json` | Lab code folder moved after configuring | delete the `build` folder and configure again |
| `too many arguments provided to function-like macro` | a template argument list such as `T.topRightCorner<3, 1>()` inside `CHECK_...(...)` | store it in a variable first |
| Tree results are wrong or garbage after `auto M = A * B;` | `auto` keeps an Eigen expression, not a matrix | write the type: `Eigen::Matrix4d M = A * B;` |
| `bad optional access` | you read `*sc.q` for a pure translation, or `twist_to_screw` never set `sc.q` | set `sc.q = ...` whenever `w.norm() > TOL`; check `sc.q.has_value()` before `*sc.q` |
| `test_tree_rejects_bad_edges` fails on a `CHECK_THROWS_AS` | `add` threw `std::logic_error` / `std::runtime_error` instead of `std::invalid_argument` | throw `std::invalid_argument` |
| Tests still show your old result | program not rebuilt | `cmake --build build` before `./build/lab_tests` |
