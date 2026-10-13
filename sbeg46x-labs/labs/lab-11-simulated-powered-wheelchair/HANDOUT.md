---
title: "Lab 11 — Simulated powered wheelchair: odometry, EKF landmark localisation and trajectory-tracking control (Formal report 3)"
subtitle: "SBEG 46X Medical Robotics — Week 12"
---

# Overview

| Item | Detail |
|---|---|
| Session | Session 4 computational lab, 50 min in class + about 3 h of independent work for the report |
| CLOs | **46X-7** (design state estimators — EKF — and apply wheeled-mobile-robot kinematics, odometry and localisation to assistive and mobile medical robots); **46X-10** (technical report, honest disclosure of tool use) |
| Reading | *Modern Robotics* (MR) §13.2.1 (pp. 517–521, MR Eqs. 13.6–13.10), §13.3.1 (pp. 523–529, MR Eqs. 13.12–13.21), §13.3.4 (pp. 544–547, MR Eqs. 13.29–13.31), §13.4 (pp. 548–550, MR Eqs. 13.32–13.36), §13.5 (pp. 550–553). Corke, *Robotics, Vision and Control*: §6.1 (pp. 111–116, Eqs. 6.1–6.7), §6.2 (pp. 116–120, Eqs. 6.8–6.16), §6.4 (pp. 123–125, SLAM). Week 10 lab handout (Kalman filter notation); this week's lecture notes and Workshop 11 |
| Files (your Lab code folder) | `starter.py` (your code), `test_lab.py` (auto-checks), `course_models.py` (the wheelchair-mounted arm `srs_7r_feeding`, copied from Course resources: Course robot models and hardware safety checklist), `data/wheelchair.json` (parameters), `data/ward_log.csv` (encoder log with ground truth), `data/ward_meas.csv` (landmark readings), `data/expected.json` |
| Hardware | **Simulation.** Optional: the ROS 2 / Nav2 demonstration at the end (course ROS 2 Docker image or Ubuntu 24.04 with ROS 2 Jazzy). Teams with the optional mobile-base project scenario run the same code on the base in project hardware session 2 |
| Assessment | Auto-checked code (tests) + **Formal report 3** (IEEE two-column, ≤ 6 pages, template and rubric below), due **before the Week 13 lab practical** |
| AI use | Allowed for code **with disclosure** (AI-use note in the report, appendix A); you must be able to explain every line in an oral check. Derivations, numbers, figures and conclusions in the report are your own and come from your runs |

## Why this lab matters

A powered wheelchair with a little autonomy — "take me to the nurses' station", "line up with the bed", "go back to the
charging dock" — must know **where it is** to within a few centimetres and must **follow a path** through doorways
without clipping the frame or the user's feet. Its wheel encoders give a smooth, fast, but drifting estimate
(odometry); markers on the ward walls give absolute but noisy and occasionally wrong readings; an extended Kalman filter
fuses both, and a nonholonomic tracking controller turns the estimate into wheel commands. In this lab you build each
piece, find out how much each one matters, and see that a controller is only as good as the estimate it is fed. The
same chain — odometry, landmark EKF, tracking — runs on hospital-logistics robots, casualty-evacuation vehicles and
humanitarian demining robots (Lecture B), and it is what ROS 2 Nav2 packages for you (optional demo).

## Learning objectives

By the end of the lab you can:

1. build the wheel matrices $H(0)$ and $F$ of omnidirectional and differential-drive bases and convert between wheel
   speeds and chassis twists (MR §13.2–13.3);
2. implement exact odometry (MR §13.4), and separate and quantify its systematic and random error growth;
3. implement an EKF that fuses odometry with range-bearing readings of known markers, including the Jacobians,
   bearing wrapping and outlier gating, and judge whether its covariance is consistent with its errors;
4. implement and compare the reference-point and nonlinear trajectory-tracking laws (MR Eqs. 13.21, 13.29–13.31) with
   wheel-speed saturation, fed by the true pose, odometry or the EKF;
5. (optional) compute the whole-body Jacobian of a wheelchair-mounted arm (MR §13.5) and relate it to user safety;
6. report the results in an IEEE-style technical report with a justified recommendation.

# Pre-lab check (complete before the session, 5 questions)

1. A diff-drive has $r = 0.17$ m, $d = 0.28$ m. Give $u_L$, $u_R$ for $v = 0.6$ m/s, $\omega = 0.4$ rad/s, and write the
   $3\times2$ matrix $F$ of MR Eq. 13.34.
2. Odometry: at $q_k = (0, 0, 0)$ the encoders report $\Delta\theta = (2.0, 2.0)$ rad with the chair of question 1.
   Give $q_{k+1}$. Why does MR Eq. 13.35 need two cases?
3. For the range-bearing reading of a marker at $m = (3, 4)$ from $q = (0, 0, 0)$, give $(\rho, \beta)$ and the
   Jacobian $H$ for the state order $(\phi, x, y)$.
4. A reading has innovation $\nu = (0.30\ \text{m}, 0.01\ \text{rad})$ and $S = \operatorname{diag}(0.004, 0.0004)$.
   Compute $d^2 = \nu^{\mathsf T}S^{-1}\nu$. Is it accepted by a 99 % gate with 2 degrees of freedom ($d^2 \le 9.21$)?
5. In MR Eq. 13.31, what are $v$ and $\omega$ when the error is zero, and what happens to the law as
   $\phi_e \to \pm\pi/2$?

# The simulated wheelchair and ward

| Quantity | Nominal model (odometry, EKF, controller) | "True" chair in the simulator |
|---|---|---|
| Drive-wheel radius | $r = 0.17$ m (both) | $r_L = 0.1692$ m, $r_R = 0.1706$ m |
| Half-track ($\{b\}$ at the axle midpoint) | $d = 0.28$ m | $d = 0.285$ m (tyre contact patches) |
| Slip | filter model: wheel-distance variance $k_s\lvert r\Delta\theta_i\rvert$, $k_s = 4\times10^{-5}$ m | $k_{\text{slip}} = 2\times10^{-5}$ m |
| Encoders | 4096 counts per wheel revolution (quantised cumulative angles) | — |
| Limits | $\lvert u_i\rvert \le 9$ rad/s, $\lvert v\rvert \le 1.2$ m/s (indoor) | — |
| Control / odometry period | $h = 0.02$ s | — |
| Sensor | lidar + 8 retro-reflective markers with IDs (map in `wheelchair.json`); one scan every 0.2 s; range ≤ 5 m; $\sigma_\rho = 0.05$ m, $\sigma_\beta = 1°$; 3 % of readings are reflections (range too short by 0.5–1.5 m) | same |
| Route | ward loop around the nurses' station: 6 m straight, left arc of radius 1.5 m, 4 m straight, … (29.42 m per lap), two laps at 0.6 m/s (98 s) | — |

All values are illustrative. The filter starts at the docking station with $\sigma = (2°, 2\ \text{cm}, 2\ \text{cm})$.
The recorded run (`ward_log.csv`, `ward_meas.csv`) was made with the chair following the route on the true pose; the
provided `simulate()` runs the closed loop with **your** controller and estimator (same noise sequences for every
estimator, so comparisons are fair).

# Setup

```bash
cd "<your Lab code folder>"
LAB_IMPL=starter python -m pytest -q test_lab.py     # at the start: 28 tests fail with NotImplementedError, 4 data tests pass
```

Use the course Python environment (or the Docker image / JupyterHub). Work in `starter.py` and keep the function names
and signatures. The state is ordered $(\phi, x, y)$ as in MR; Corke writes $(x, y, \theta)$ — permute rows and columns
when you compare with his equations.

# Procedure and checkpoints

### Checkpoint A — Wheel kinematics and odometry (≈ 15 min)

1. `wheel_row` (MR Eq. 13.6 at $\phi = 0$) and `H0_matrix`; the tests rebuild MR Eqs. 13.9 and 13.10 from the wheel
   geometry.
2. `diffdrive_F` (MR Eq. 13.34) and `wheel_speeds` (MR p. 528).
3. `odometry_step` (MR Eqs. 13.35–13.36, both cases, including $v_{by}$ for omni/mecanum bases) and `dead_reckoning`.
4. `python -m pytest -q test_lab.py -k "wheel or H0 or diffdrive or odometry_step or dead_reckoning"`. The odometry test
   compares your update with the matrix exponential $T_{sb'} = T_{sb}e^{[\mathcal V_{b6}]}$ (MR p. 549).

**Checkpoint question:** your dead reckoning of the log ends almost 3 m from the truth after 59 m. Is that mostly
random slip or wrong parameters? (Experiment A2 answers it.)

### Checkpoint B — EKF localisation (≈ 20 min)

1. `odometry_jacobians` ($F_q$, $F_u$; the test uses finite differences, including a straight move where
   $\omega = 0$ — use the series given in the docstring), `wheel_noise_cov`, `ekf_predict`.
2. `range_bearing`, `range_bearing_jacobian`, `ekf_update` (wrap the bearing innovation; gate with the squared
   Mahalanobis distance; Joseph form) and `ekf_localise` (predict with each encoder increment, then update with every
   reading of that sample).
3. `python -m pytest -q test_lab.py -k "jacobian or noise or ekf or range"`. One test feeds a marker straight behind the
   chair across the ±π cut; another feeds a reflection that the gate must reject.

**Checkpoint question:** how many readings of the log does your gate reject, and how many were generated as
reflections (about 3 % of 1396)? Are they the same readings?

### Checkpoint C — Tracking control (rest of the session, then at home)

1. `error_coordinates` (MR Eq. 13.30), `tracking_control` (MR Eq. 13.31), `point_controls` (MR Eq. 13.21),
   `point_tracking_control` (MR Eq. 13.29 with the feed-forward of the reference-point velocity) and `saturate_wheels`
   (one common scale factor — Workshop Problem 1c).
2. Run the whole suite: `python -m pytest -q test_lab.py` — all 32 tests must pass (Part D included; Part D is optional
   for the report but its two functions are short).
3. Do the experiments below. Save every figure with axis labels and units; draw the ward loop and the markers on the
   trajectory plots.

**Experiment A — Odometry and its error growth.**

* A1. Dead-reckon the log with the nominal model. **Figure 1:** true and odometry paths with the markers.
  **Figure 2:** position and heading error versus distance travelled (`distance_travelled`). Fit the growth on a
  log–log plot and compare the slope with Workshop Problem 3 (systematic $\propto s^2$, random $\propto s^{3/2}$).
* A2. Dead-reckon again with the "calibrated" parameters (the true $r_L$, $r_R$, $d$ of `wheelchair.json`; build
  your own $F$ with two radii). **Table 1:** final position and heading errors of A1 and A2. What fraction of the
  error was systematic? Which single parameter matters most? (Change one at a time.)

**Experiment B — EKF localisation.**

* B1. Run `ekf_localise` on the log. **Figure 3:** position error and heading error versus time with the
  $\pm3\sigma$ bounds from $P$; mark the rejected readings. Report the rms and maximum errors and the fraction of time
  the error lies inside $\pm3\sigma$. Is the filter consistent?
* B2. **Table 2:** rms/max position error, rms heading error and accepted/rejected readings for (i) the default filter,
  (ii) no gate (`use_gate=False`), (iii) only markers 1, 4 and 6 (`landmark_ids=[1, 4, 6]`), (iv) $k_s \times 0.1$ and
  (v) $k_s \times 10$. Explain each change: what does an outlier do without the gate, why does an over-confident filter
  ($k_s \times 0.1$) reject **good** readings, and how does the covariance behave between marker sightings in (iii)
  (Corke Fig. 6.8b)?

**Experiment C — Closed-loop tracking.**

* C1. `simulate(p, est, ctl)` for `est` in truth / odometry / EKF and `ctl` in nonlinear / point (initial pose error
  $(0.2\ \text{rad}, -0.4\ \text{m}, 0.3\ \text{m})$). **Figure 4:** the true paths of the three nonlinear runs over the
  ward map. **Table 3:** `tracking_metrics` of all six runs plus the localisation rms error. Explain why the controller
  with odometry "tracks perfectly" in its own estimate yet leaves the true chair metres off the route.
* C2. Start 2.8 rad away from the reference heading: `simulate(p, "truth", ctl, offset=[2.8, 0, 0])` for both laws.
  **Figure 5:** the first 10 s of both paths and $v(t)$. Which law turns the chair round, which drives the loop
  backwards, and which assumption of MR Eq. 13.31 is violated? What would you add to a real wheelchair controller
  before engaging autonomous tracking?
* C3. Change the gains $k_1, k_2, k_3$ (factor 0.5 and 2; `gains=dict(k1=..., ...)`) with the EKF. Report the rms
  error and the peak wheel speed, and relate the trade-off to user comfort (turning-rate spikes after each EKF update).

**Experiment D (optional) — The spoon and the chair.**

* D1. With `course_models.srs_7r_feeding()`, the mount `p['arm_mount']` (right armrest) and the feeding pose
  `theta_feed`, compute $J_e = [J_{\text{base}}\ J_{\text{arm}}]$. How fast does the spoon move if the chair spins at
  0.5 rad/s while the arm holds still? Compare with Practice Problem 2 (arm stretched forward). Propose an interlock.

**Expected values (to check yourself; TAs use them for calibration).**

| Quantity | Expected |
|---|---|
| Log | 4906 samples (98.1 s), 1396 readings, 58.85 m driven |
| A1, dead reckoning (nominal), final position / heading error | 2.960 m / −0.651 rad; after one lap 1.207 m; maximum 3.244 m |
| A1, final odometry pose $(\phi, x, y)$ | (11.921 rad, −1.618 m, 2.471 m) — truth ≈ (12.57, 0.01, 0.00) |
| A2, dead reckoning (true parameters), final position / heading error | 0.161 m / −0.016 rad |
| B1, EKF: rms / max position error, rms heading error | 8.2 mm / 22.0 mm, 0.0073 rad |
| B1, accepted / rejected readings; mean / max $\sqrt{\sigma_x^2 + \sigma_y^2}$ | 1345 / 51; 9.6 mm / 28.6 mm |
| B2 (ii) no gate: rms / max position error | 19.5 mm / 106 mm |
| B2 (iii) markers 1, 4, 6: rms / max position error, accepted / rejected | 30.1 mm / 111 mm, 414 / 15 |
| B2 (iv) $k_s \times 0.1$: rms position / heading error, rejected | 10.8 mm / 0.0119 rad, 70 |
| B2 (v) $k_s \times 10$: rms position error, mean $\sqrt{\sigma_x^2 + \sigma_y^2}$ | 9.5 mm, 14.2 mm |
| C1, true pose: rms / max position error ($t \ge 5$ s), nonlinear / point | 3.9 / 11.7 mm, 3.2 / 8.9 mm |
| C1, odometry: rms / max / final position error, nonlinear | 1.685 / 2.761 / 2.401 m |
| C1, EKF: rms / max position error, nonlinear / point; localisation rms | 13.2 / 31.8 mm, 13.0 / 28.9 mm; 9.3 mm |
| C1, peak wheel speed, nonlinear / point | 7.76 rad/s / 9.0 rad/s (saturated in 0.02 % of samples, at the start) |
| C2, heading error 2.8 rad: rms heading error, nonlinear / point | 3.138 rad (drives backwards, $v_{\min} = -0.637$ m/s) / 0.0037 rad |
| D1, spoon speed for a 0.5 rad/s spin (feeding pose) | 0.400 m/s |

<!-- library-block:begin -->
# Your `sbeg46x_medrob` library

When the tests pass, add the functions you wrote in Lab 11 to a new module `mobile.py` of your library. The functions
you write (20):

`wheel_row(r, gamma, beta, xw, yw)`, `H0_matrix(wheels)`, `diffdrive_F(r, d)`, `wheel_speeds(v, omega, r, d)`,
`odometry_step(q, dtheta, F)`, `dead_reckoning(q0, enc, F)`, `odometry_jacobians(q, dtheta, r, d)`,
`wheel_noise_cov(dtheta, r, k_s)`, `ekf_predict(q, P, dtheta, r, d, k_s)`, `range_bearing(q, m)`,
`range_bearing_jacobian(q, m)`, `ekf_update(q, P, z, m, R, gate=None)`, `ekf_localise(log, meas, p, ...)`,
`error_coordinates(q, qd)`, `tracking_control(q, qd, vd, wd, k1, k2, k3)`, `point_controls(phi, xr, yr, pdot)`,
`point_tracking_control(q, qd, vd, wd, xr, kp)`, `saturate_wheels(v, omega, r, d, u_max, v_max=None)`,
`base_jacobian(F, Tb0, T0e)`, `mobile_manip_jacobian(F, Tb0, Blist, M, theta)`.

Also copy the provided `wrap`, `rot2` and `pose_errors` into `mobile.py`. The `kf_predict` / `kf_update` of Lab 9
(`estimation.py`) are the linear special case of `ekf_predict` / `ekf_update`: keep both. Project teams with strand (g)
reuse this module. The full list of library functions of all labs is in the Week 13 lab handout.
<!-- library-block:end -->

# Optional — ROS 2 / Nav2 demonstration (ungraded, ≈ 30 min at home or in hardware session 2)

Nav2 packages the same chain for real robots: wheel odometry (`/odom`), a localiser that corrects it against a map
(AMCL — a particle filter, Corke §6.5 pp. 125–128 — instead of your EKF), global and local planners on costmaps, and a
path-tracking controller that outputs $(v, \omega)$ on `/cmd_vel`. In the course ROS 2 Docker image (or Ubuntu 24.04 with
ROS 2 Jazzy):

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False   # TurtleBot3 diff-drive in Gazebo + RViz
```

(Package and launch-file names follow the Nav2 "Getting Started" page for your ROS 2 distribution; if a name has
changed, use the one given there.) In RViz: give the initial pose with **2D Pose Estimate**, send a **Nav2 Goal**, and
watch the particle cloud contract. Then `ros2 topic echo /odom` and `ros2 topic echo /amcl_pose`, and
`ros2 topic echo /cmd_vel` while it drives. For the report (optional paragraph): map each Nav2 component to the
function you wrote today, and name one thing Nav2 does that your lab code does not (e.g. obstacle avoidance on a costmap,
recovery behaviours, a velocity smoother). Never run Nav2 on a real wheelchair with a person in it: a powered wheelchair
is a medical device (Week 13).

# Deliverables

1. `starter.py` renamed `lab11_<ID>.py`, passing `LAB_IMPL=lab11_<ID> python -m pytest -q test_lab.py` (32 tests), and
   the 20 functions added to `mobile.py` of your library.
2. A script or notebook `experiments.py` / `.ipynb` that regenerates every figure and table of the report.
3. **Formal report 3** (PDF, IEEE two-column template, ≤ 6 pages excluding appendices) with Figures 1–5 and Tables 1–3
   (plus D1 and the Nav2 paragraph if you did them).
4. A reflection (≤ 200 words, appendix B) and an AI-use note (appendix A).

# Formal report 3 — template

Use the IEEE conference two-column template (LaTeX `IEEEtran` or the Word version). Title: *Localisation and
trajectory tracking of a powered wheelchair: odometry, landmark-based extended Kalman filtering and nonholonomic
feedback control*. Suggested length per section in brackets.

| Section | What to write |
|---|---|
| **Abstract** (≤ 150 words) | Problem, method (exact diff-drive odometry, EKF with range-bearing markers and gating, nonlinear and reference-point tracking laws), key numbers (odometry drift over the route, EKF rms error and consistency, tracking error with each estimator), one-sentence conclusion. |
| **I. Introduction** (½ page) | Why an assistive wheelchair (or a hospital-logistics or casualty-evacuation vehicle) needs localisation and path tracking; the ward scenario; objectives and contributions. |
| **II. System model** (½–1 page) | Diff-drive kinematics (MR Eqs. 13.15, 13.34), odometry (MR Eqs. 13.35–13.36), the error model (systematic: radii, track; random: slip, quantisation), the sensor model and the map. State that the values are illustrative and which parameters the filter does **not** know. |
| **III. Estimator and controller design** (1 page) | EKF: state, $F_q$, $F_u$, $Q_u$, $H$, $R$, gain (write $L$ as in Week 10 and the notes; the code calls it `K`), gating, bearing wrapping, Joseph form, initialisation; tracking: error coordinates, MR Eq. 13.31, reference-point law, wheel saturation with a common factor. Cite MR and Corke with sections, equations and pages. |
| **IV. Verification** (¼ page) | Unit tests (what each checks), comparison with the expected values, and one independent check of your own (e.g. a hand EKF step as in Workshop Problem 4, or the error growth law of Workshop Problem 3 against Experiment A). |
| **V. Results** (1½ pages) | Figures 1–5 and Tables 1–3. Every figure is referred to in the text and has a caption that says what to see. |
| **VI. Discussion** (1 page) | Systematic vs random odometry error and what calibration buys; consistency of the EKF ($3\sigma$ bounds, effect of $k_s$, gate); marker density and placement; why tracking quality is limited by the estimator; the heading-reversal case and how you would prevent it; comfort (turning-rate spikes) and safety (footrest sweep, door frames); limits of the model (planar, no dynamics, known data association, static ward, no people). If you did D1 or Nav2, one paragraph each. |
| **VII. Conclusion** (¼ page) | Answer the objectives in 3–5 sentences, with a recommendation for the wheelchair's localisation and control design. |
| **References** | MR (sections, equations, pages), Corke (sections, equations, pages), the Nav2 documentation if used; IEEE citation style. |
| **Appendix A — AI-use note** | Tool, what you asked, what you kept or changed, how you verified it (or "No AI tools were used."). |
| **Appendix B — Reflection** | ≤ 200 words, prompts below. |

# Formal report rubric (100 points)

| Criterion | Points | Excellent (90–100 %) | Proficient (70–89 %) | Developing (40–69 %) | Missing / incorrect (0–39 %) |
|---|---|---|---|---|---|
| Abstract and introduction | 10 | Clinical (and, if relevant, defence) motivation with concrete accuracy needs (doorway clearance, docking); measurable objectives; key numbers in the abstract | Clear but generic; few numbers | Vague objectives | Missing |
| Model, estimator and controller design | 20 | Correct kinematics and exact odometry; error sources separated; EKF Jacobians derived (not copied) for the $(\phi, x, y)$ order; gating and wrapping justified; tracking laws and saturation explained with their assumptions ($v_d \ne 0$, $\lvert\phi_e\rvert < \pi/2$) | Minor slips in derivation or notation | Equations listed without explanation | Wrong or missing |
| Verification evidence | 10 | All tests pass; expected values reproduced; at least one independent check of your own, quantified | Tests pass, independent check weak | Some tests fail or no check | No verification |
| Results (figures and tables) | 25 | Figures 1–5 and Tables 1–3 complete, labelled, with units, map and $\pm3\sigma$ bounds drawn and captions; numbers consistent with the expected values; error-growth slopes reported | One figure or table weak | Several missing or unreadable | Missing |
| Discussion, safety and recommendation | 20 | Systematic vs random error quantified; filter consistency and tuning discussed; estimator–controller coupling explained; heading-reversal and comfort/safety issues with concrete mitigations; justified design recommendation; model limits | Recommendation with partial justification | Descriptive only | Missing |
| Writing, format, references | 10 | IEEE format, ≤ 6 pages, concise, correct citations with pages | Minor format issues | Over length or poor structure | Not in format |
| AI-use note, reflection, reproducibility | 5 | Honest AI note, thoughtful reflection, `experiments.py` regenerates every figure and table | One item weak | Items superficial | Missing |

Oral check: a TA may ask you to explain any line of your code or any figure (2–3 minutes); failure to explain caps the
code mark at 50 %.

# Reflection prompts (appendix B, ≤ 200 words)

* Which result surprised you most (for example, that a controller fed by odometry reports almost zero error while the
  real chair is 2 m off the route, or that a better-tuned "confident" filter rejects good readings), and why?
* If you were the wheelchair user, what would you want the chair to do — and to tell you — when its position
  uncertainty grows (no markers in view) or a reading is rejected?
* **AI-use note:** if you used an AI assistant, which suggestion did you reject or correct, and how did you find out it
  was wrong (a test, a hand calculation, a plot)?

# Safety note

This lab is simulation only. If your project runs this code on a mobile base (optional Tier 3 scenario) or you test
Nav2 on a physical robot: never with a person seated on or carried by the robot; run the exact command first in
simulation; limit speed (≤ 0.3 m/s for first runs) and turning rate in the base's own firmware **in addition to** your
`saturate_wheels`; clear the area, keep a spotter with the emergency stop who is not operating the laptop, and stop
autonomous motion whenever the localisation covariance exceeds a set threshold. Follow Course resources: Course robot
models and hardware safety checklist.

# Troubleshooting

| Symptom | Likely cause and fix |
|---|---|
| Odometry test fails only when turning | You used the Euler update, or rotated $\Delta q_b$ by $\phi_{k+1}$ instead of $\phi_k$ (MR Eq. 13.36). |
| Odometry test fails for the mecanum case | $v_{by}$ missing in MR Eq. 13.35, or the $\omega = 0$ branch drops $v_{by}$. |
| `odometry_jacobians` fails for the straight move | Division by $\omega \approx 0$: use the series of the docstring for $\lvert\omega\rvert < 10^{-4}$. |
| Jacobian test fails by a sign in $H$ | $H$ is the derivative with respect to the **robot** position: $\partial\rho/\partial x = -\Delta x/\rho$. |
| EKF diverges after the chair turns past ±180° | Bearing innovation not wrapped, or you wrapped $\phi$ inside the state but not consistently; wrap only innovations and errors. |
| EKF rms error ≈ 20 mm instead of 8 mm | The gate is off or uses $R$ instead of $S$. |
| `ekf_localise` matches only roughly | Order matters: predict with `enc[k+1] - enc[k]`, then all readings with `k + 1`, in file order. |
| Tracking test fails at the first samples | `saturate_wheels` clips one wheel instead of scaling both; or $\lvert v_d\rvert$ vs $v_d$ mixed up in MR Eq. 13.31. |
| `point_tracking_control` turns the wrong way | MR Eq. 13.21 divides the second row by $x_r$; the feed-forward $\dot p_d$ includes $x_r\omega_d$. |
| `ModuleNotFoundError: starter` | Run `pytest` from inside your Lab code folder, or set `LAB_IMPL` to your file name without `.py`. |

# Pre-lab answers (released after the session)

1. $u_L = (0.6 - 0.112)/0.17 = 2.871$ rad/s, $u_R = 0.712/0.17 = 4.188$ rad/s;
   $F = 0.17\begin{bmatrix}-1/0.56 & 1/0.56\\ 1/2 & 1/2\\ 0 & 0\end{bmatrix} = \begin{bmatrix}-0.30357 & 0.30357\\ 0.085 & 0.085\\ 0 & 0\end{bmatrix}$.
2. $\mathcal V_b = (0, 0.34, 0)$, so $q_{k+1} = (0, 0.34, 0)$. The general formula divides by $\omega_{bz}$; for a
   straight move its limit is $(0, v_{bx}, v_{by})$.
3. $\rho = 5$, $\beta = \operatorname{atan2}(4, 3) = 0.9273$ rad;
   $H = \begin{bmatrix}0 & -0.6 & -0.8\\ -1 & 0.16 & -0.12\end{bmatrix}$.
4. $d^2 = 0.09/0.004 + 0.0001/0.0004 = 22.5 + 0.25 = 22.75 > 9.21$: **rejected**.
5. $(v, \omega) = (v_d, \omega_d)$ (pure feed-forward); the controls grow without bound as $\phi_e \to \pm\pi/2$
   ($1/\cos\phi_e$ and $\tan\phi_e$), so the law assumes $\lvert\phi_e\rvert < \pi/2$ (MR p. 546).

# C++ track

You may do this lab in C++17 instead of Python. The functions, data files, checkpoints, experiments, expected values, report and deliverables are the same (deliverable 1 is `cpp/starter.cpp` instead of `starter.py`; deliverable 2 is your experiments code in C++ plus your plotting script); the auto-checks compare your results with `mrcpp` (the C++ port of `modern_robotics` in the course folder `cpp_support`), with the same closed forms and finite differences as `test_lab.py`, with `data/expected.json` and with the numbers of the Python reference, so both tracks are marked against identical values.

**Files** (in the `cpp` sub-folder of your Lab code folder): `starter.cpp` (your work), `lab.hpp` (declarations and documentation — do not change), `helpers.cpp` (the provided helpers: `load_params`, `landmark_table`, `load_log`, `load_meas`, `wrap`, `rot2`, `reference_trajectory`, `pose_errors`, `distance_travelled`, `simulate`, `tracking_metrics` and `sim_noise` — do not change), `course_models.hpp` / `course_models.cpp` (the feeding arm `srs_7r_feeding`, numbers identical to `course_models.py` — do not change), `test_lab.cpp` (auto-checks, doctest), `cpp_cases.json` (reference numbers and the random inputs of `test_lab.py`), `sim_noise_1211.json` (the slip and sensor-noise sequences of `simulate` for seed 1211), `plot_results.py` (Figures 1–4), `CMakeLists.txt`. The data files are the same files in `data/`. `cpp_support` (Course resources) must sit next to your lab folders or in a parent folder of them (the build finds it automatically up to 6 levels up).

**Requirements.** A C++17 compiler — Windows: Visual Studio Build Tools (2022 or 2026), workload "Desktop development with C++"; macOS: `xcode-select --install`; Ubuntu: `sudo apt install build-essential`. CMake and Ninja come with `pip install cmake ninja` (both are in the course `requirements.txt`).

**Build and run** (from your Lab code folder):

```bash
python run_lab.py 11 --cpp                     # configure, build, run, print Score: k/n
# or directly:
cmake -S cpp -B build -G Ninja
cmake --build build
./build/lab_tests                              # Windows: build\lab_tests.exe
./build/lab_tests -tc="*wheel_row*,*H0*"       # only some tests (like pytest -k)
python cpp/plot_results.py                     # Figures 1-4 (after the tests of Parts A-C pass)
```

With the untouched starter, the program builds and all 25 test cases fail with `TODO: <function>`: this is expected. Rebuild (`cmake --build build`) after every change. A full run takes about half a second.

**Functions to write** (in `starter.cpp`, same order as in Python, 20 functions): `wheel_row`, `H0_matrix`, `diffdrive_F`, `wheel_speeds`, `odometry_step`, `dead_reckoning` (A1–A6); `odometry_jacobians`, `wheel_noise_cov`, `ekf_predict`, `range_bearing`, `range_bearing_jacobian`, `ekf_update`, `ekf_localise` (B1–B7); `error_coordinates`, `tracking_control`, `point_controls`, `point_tracking_control`, `saturate_wheels` (C1–C5); `base_jacobian`, `mobile_manip_jacobian` (D1–D2, optional). The helpers of `helpers.cpp` and the arm of `course_models.hpp` are given, as the provided section of `starter.py` and `course_models.py`; `simulate` calls your functions exactly as the Python one does. As in Python, you may call `mrcpp` in Part D (`mr::fkin_body`, `mr::jacobian_body`, `mr::adjoint`, `mr::trans_inv`).

**Checkpoints.** A: `./build/lab_tests -tc="*wheel_row*,*H0*,*diffdrive*,*wheel_speeds*,*odometry_step*,*dead_reckoning*,*data_files*,*reference_route*,*ground_truth*"` (10 test cases); B: `-tc="*odometry_jacobians*,*noise_cov*,*ekf*,*range_bearing*,*wrap_and*"` (8); C: `-tc="*error_coordinates*,*tracking_control*,*point*,*saturate*,*closed_loop*"` (6; `test_closed_loop_runs` also needs Parts A and B); D: `-tc="*base_jacobian*"` (1); finally all 25 test cases pass (`./build/lab_tests`). The Python track counts 32 tests because pytest counts each parameter of `test_odometry_step_matches_matrix_exponential` (2), `test_odometry_jacobians_finite_differences` (5) and `test_closed_loop_runs` (3) separately; the C++ track checks the same cases inside one test case each, with the same names. Difference: in C++ the four data tests also call one of your functions (`test_data_files_consistent` checks the first encoder increment of the log with `wheel_speeds`; `test_reference_route_closes` checks that the reference never needs more than $u_{\max}$ with `wheel_speeds`; `test_log_ground_truth_moves_like_a_diff_drive` checks one `odometry_step` against the true pose to 2 mm; `test_wrap_and_landmarks` checks `range_bearing` of a marker straight behind the chair), so they pass only in Checkpoints A and B, not with the untouched starter.

**Running the experiments and printing the tables.** Add a temporary `TEST_CASE` at the end of `test_lab.cpp` (include `<iostream>`; remove it before you submit), for example:

```cpp
TEST_CASE("my_results") {
    const lab::Params p = lab::load_params();
    const lab::Log log = lab::load_log();
    const std::vector<lab::Reading> meas = lab::load_meas();
    Eigen::MatrixXd enc(log.t.size(), 2), qt(log.t.size(), 3);
    enc << log.enc_L, log.enc_R;
    qt << log.phi, log.x, log.y;
    Eigen::MatrixXd Fc(3, 2);  // A2: odometry with the true radii and half-track
    Fc << -p.truth.r_L / (2 * p.truth.d), p.truth.r_R / (2 * p.truth.d), p.truth.r_L / 2, p.truth.r_R / 2, 0.0, 0.0;
    const lab::PoseErrors ec = lab::pose_errors(lab::dead_reckoning(qt.row(0).transpose(), enc, Fc), qt);
    std::cout << "A2 final errors: " << ec.e_pos.tail(1) << " m, " << ec.e_phi.tail(1) << " rad\n";
    const lab::EkfRun b = lab::ekf_localise(log, meas, p, std::vector<int>{1, 4, 6});  // B2 (iii)
    std::cout << "B2 (iii): accepted " << b.n_acc << ", rejected " << b.n_rej << "\n";
    const lab::EkfRun b4 = lab::ekf_localise(log, meas, p, std::nullopt, true, 0.1 * p.ekf.k_s);  // B2 (iv)
    std::cout << "B2 (iv): rejected " << b4.n_rej << "\n";
    lab::SimOptions rev;  // C2: start 2.8 rad off the reference heading
    rev.offset = Eigen::Vector3d(2.8, 0.0, 0.0);
    const lab::SimResult c2 = lab::simulate(p, "truth", "nonlinear", rev);
    std::cout << "C2 nonlinear: rms heading error " << lab::tracking_metrics(c2).rms_phi
              << ", v_min " << c2.v.minCoeff() << "\n";
    Eigen::MatrixXd f5(c2.t.size(), 4);  // Figure 5 data
    f5 << c2.t, c2.q_true.topRows(c2.t.size()).rightCols(2), c2.v;
    sbeg::write_csv("lab11_c2_nonlinear.csv", {"t", "x", "y", "v"}, f5);
    lab::SimOptions g;  // C3: gains x 0.5 with the EKF
    g.gains = {{"k1", 1.0}, {"k2", 2.0}, {"k3", 1.5}};
    const lab::TrackingMetrics m = lab::tracking_metrics(lab::simulate(p, "ekf", "nonlinear", g));
    std::cout << "C3 (x 0.5): rms " << m.rms_pos << " m, peak wheel speed " << m.peak_u << " rad/s\n";
}
```

The reference solution prints 0.16091 m, −0.0159511 rad; 414 / 15; 70; 3.13815, −0.636792 — the values of the expected-values table. For the figures, `./build/lab_tests` writes `lab11_cpp_odometry.csv` (A1), `lab11_cpp_ekf.csv` and `lab11_cpp_ekf_readings.csv` (B1: errors, $\sigma$ values and the $d^2$ of every reading) and `lab11_cpp_track_truth.csv`, `lab11_cpp_track_odometry.csv`, `lab11_cpp_track_ekf.csv` (C1, nonlinear law) into the folder you run it from, each one when the functions it uses work; `python cpp/plot_results.py` saves Figures 1–4 (`lab11_fig1_paths.png` … `lab11_fig4_tracking.png`) and prints the error-growth slope of A1 and the $3\sigma$ fraction of B1. Plot Figure 5 from your own CSV file (as `lab11_c2_nonlinear.csv` above) with a short Python script; plotting stays in Python.

**Differences from the Python version.**

| Python | C++ |
|---|---|
| parameter dict `p["chair"]["r"]`, `p["ekf"]["k_s"]`, `p["route"]["segments"]` | `lab::Params` with the same names: `p.chair.r`, `p.ekf.k_s`, `p.route.segments` (`RouteSegment{kind, a, b}`); `p.landmarks` is a matrix with rows `[id, x, y]` |
| `load_log()` dict of arrays; `load_meas()` list of dicts | `lab::Log` (`log.t`, `log.enc_L`, …, `Eigen::VectorXd`); `std::vector<lab::Reading>` (`z.k`, `z.id`, `z.range`, `z.bearing`) |
| pose `q` (array of 3); N × 3 trajectories | `Eigen::Vector3d`; `Eigen::MatrixXd` with one **row** per sample (`Q.row(k).transpose()` is a pose) |
| wheels = list of tuples `(r, gamma, beta, xw, yw)` | `std::vector<lab::Wheel>`, e.g. `{{r, 0.0, 0.0, 0.0, d}, {r, 0.0, 0.0, 0.0, -d}}` |
| tuples `(u_L, u_R)`, `(v, omega)`, `(q-, P-)` | `std::pair` (`.first`, `.second`) |
| `(q_next, F_q, F_u)`; `(q+, P+, info)`; result dict of `ekf_localise` | structs `lab::OdometryJacobians` (`q_next, F_q, F_u`), `lab::EkfUpdate` (`q, P, nu, S, K, d2, accepted`; `K` is zero for a rejected reading, Python `None`), `lab::EkfRun` (`q, P, n_acc, n_rej, d2`) |
| `gate=None`, `v_max=None`, `landmark_ids=None`, `k_s=None` | `std::optional`: omit the argument or pass `std::nullopt`; pass a number or `std::vector<int>{1, 4, 6}` directly |
| `simulate(p, est, ctl, seed=..., offset=..., gains=dict(...))` | `lab::simulate(p, est, ctl, opt)` with `lab::SimOptions opt; opt.offset = Eigen::Vector3d(...); opt.gains = {{"k1", 1.0}};` (fields `seed, offset, landmark_ids, use_gate, k_s, gains`); result `lab::SimResult` with the same field names (`P` is a `std::vector`, `sat` a `std::vector<bool>`) |
| simulator noise `np.random.default_rng(seed)` | seed 1211 (tests and experiments) uses the NumPy sequences stored in `sim_noise_1211.json`, so the C++ runs reproduce the Python numbers; any other seed uses a C++ generator — valid noise, but different numbers from Python's |
| `course_models.srs_7r_feeding()` dict | `cm::srs_7r_feeding()` (fields `Slist, Blist, M, limits, qdot_max`) |
| `np.linalg.solve(S, nu)`, `np.linalg.inv(S)` | `S.partialPivLu().solve(nu)`, `S.inverse()` |
| plots with Matplotlib | CSV files written by `lab_tests` + the given `plot_results.py` |

A size mistake between fixed-size types (e.g. multiplying the 3 × 2 `Matrix32d` $F_u$ as if it were 2 × 3) is a compile error (`INVALID_MATRIX_PRODUCT` or `YOU_MIXED_MATRICES_OF_DIFFERENT_SIZES`). A size or index mistake with the dynamic-size types `Eigen::MatrixXd` / `Eigen::VectorXd` (e.g. a `diffdrive_F` that returns 2 × 3) does not crash the test program: it is reported as a failed test with the message "Eigen: wrong matrix size or index out of range".

**C++ troubleshooting.**

| Symptom | Likely cause | Fix |
|---|---|---|
| `cpp_support not found` | the course folder `cpp_support` is not next to / above your Lab code folder | copy it there, or configure with `-DSBEG_CPP_SUPPORT=<path>` |
| `No CMAKE_CXX_COMPILER could be found` | no C++ compiler installed | install it (see Requirements); on Windows run from the "x64 Native Tools Command Prompt" if Ninja cannot find `cl` |
| `cannot open file: .../cpp_cases.json`, `.../sim_noise_1211.json` or `.../data/...` | Lab code folder moved after configuring | delete the `build` folder and configure again |
| `too many arguments provided to function-like macro` | a comma inside `CHECK_...` that is not inside parentheses (e.g. `std::set<int>{1, 2}` or `M.block<3, 1>(0, 3)`) in your own test | put the argument in parentheses |
| wrong values only after a few steps, or random-looking numbers | an Eigen expression stored in `auto` (`auto P = F * P0 * F.transpose();`) | write the type: `Eigen::Matrix3d P = ...;` or call `.eval()` |
| `odometry_step` or `tracking_control` wrong by a large amount | `abs` instead of `std::abs` (integer version), or `1 / 2` integer division | use `std::abs`, write `0.5` or `1.0 / 2` |
| `test_closed_loop_runs` fails although the other tests pass | a small difference in `saturate_wheels` or `ekf_update` (e.g. the gate uses $R$ instead of $S$) is amplified by the closed loop | re-check C5 and B6 against `lab.hpp`; compare `tracking_metrics` with the expected-values table |
| Tests still show your old result | program not rebuilt | `cmake --build build` before `./build/lab_tests` |
