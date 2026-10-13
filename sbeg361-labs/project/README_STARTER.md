# SBEG 361 project starter code

Python models of the six project robots, a baseline checker and tests. Requires Python 3.11+, `numpy`,
`modern_robotics` and `pytest` (all in the course Docker image).

| File | Purpose |
|---|---|
| `kintools.py` | Small kinematics toolkit in *Modern Robotics* notation: screw axes, PoE FK (MR Eq. 4.14), $J_s$, $J_b$ (MR Eqs. 5.11, 5.22), point Jacobian, manipulability $\mu_1,\mu_2,\mu_3$ (MR §5.4), statics $\tau = J^{\mathsf T}\mathcal F$ (MR Eq. 5.26), damped body-frame numerical IK with a task mask (MR §6.2.2), resolved-rate step (MR Eq. 6.7) |
| `scenarios.py` | Robot models (screw axes, $M$, limits) and nominal tasks for S1–S6 — **do not edit**; build redesigned robots by calling the model functions with new arguments |
| `baseline.py` | Loads each robot and checks the baseline design: `python baseline.py` (all) or `python baseline.py 4` |
| `team_design.py` | **Your work**: implement `evaluate_sN()` for your scenario (TEAM tests and redesigned baseline FAILs) |
| `tests/test_models.py` | Model checks (screw axes, $T(0)=M$, body form, Jacobians by finite differences, MR Example 4.5, IK, statics) and baseline regression against `baseline_reference.json` |
| `tests/test_team.py` | Runs your `evaluate_sN()`; skipped until you implement it |
| `hwcompare.py` | Required hardware run: reads a CSV log of commanded vs measured joints (`t, cmd_1…cmd_n, meas_1…meas_n, stop`, degrees) and computes HW-2 – HW-6 (joint RMS/max error, tool-point error via PoE FK, joint-limit and speed margins with `--session H1` for the 25 % H1 limit, safe stop) |
| `tests/test_hwcompare.py` | Tests of `hwcompare.py` on known values and a synthetic log |

```bash
python baseline.py          # baseline status table (PASS / FAIL / TEAM)
pytest -q                   # 38 passed, 6 skipped before you start
python hwcompare.py --demo demo.csv    # write a synthetic hardware log
python hwcompare.py demo.csv --limits -165 165 -165 165 -165 165 -165 165 -165 165 -175 175 --rated 160 160 160 160 160 160
python hwcompare.py h1_log.csv --rated 160 160 160 160 160 160 --session H1   # HW-5 at 25 % of rated in H1
```

The numbers are those of the course arm ("Course resources: Course robot models and hardware safety checklist"):
rated joint speed 160 °/s on every joint and hardware joint limits ±165° (J1–J5), ±175° (J6); HW-4 then checks the
≥ 10° margin of checklist rule 1. `--session H1|H2` sets the HW-5 limit of checklist rule 3: ≤ 25 % of rated
(≤ 40 °/s) in H1, ≤ 50 % (≤ 80 °/s) otherwise (default `H2`). The synthetic demo log peaks at about 27 % of rated
because of its added noise, so it passes the default check and fails with `--session H1`.

All robots except the UR5 (MR Example 4.5) use illustrative values. To explore a model in MR Studio, use the
Robot Builder with the screw-axis table from `scenario_sheets.md`, or export `Slist` and `M` from `scenarios.py`.
