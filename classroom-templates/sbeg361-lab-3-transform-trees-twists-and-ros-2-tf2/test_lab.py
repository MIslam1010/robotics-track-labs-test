"""Auto-checks for SBEG 361 Lab 3 -- transform trees and twists.

Usage:   LAB_IMPL=starter  python -m pytest -q test_lab.py     (your work, file starter.py)
         LAB_IMPL=solution python -m pytest -q test_lab.py     (reference; default)
Ground truth: the official `modern_robotics` library (Lynch & Park), the worked examples of
MR Section 3.3 (Fig. 3.14, Fig. 3.15, Example 3.19, Example 3.23) and scipy.spatial.transform.
"""
import importlib
import os
import sys
from pathlib import Path

import numpy as np
import pytest
import modern_robotics as mr
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
lab = importlib.import_module(os.environ.get("LAB_IMPL", "starter"))

RNG = np.random.default_rng(361)
S2 = 1 / np.sqrt(2)


def random_T(seed):
    rng = np.random.default_rng(seed)
    return mr.RpToTrans(Rotation.random(random_state=seed).as_matrix(), rng.normal(size=3))


RANDOM_T = [random_T(s) for s in range(20)]

# MR Fig. 3.14 (printed p. 90)
T_SB_314 = mr.RpToTrans(np.array([[0, 0, 1], [0, -1, 0], [1, 0, 0.0]]), [0, -2, 0])
T_SC_314 = mr.RpToTrans(np.array([[-1, 0, 0], [0, 0, 1], [0, 1, 0.0]]), [-1, 1, 0])
T_BC_314 = mr.RpToTrans(np.array([[0, 1, 0], [0, 0, -1], [-1, 0, 0.0]]), [0, -3, -1])

# MR Example 3.19 (printed pp. 93-95)
T_DB = np.array([[0, 0, -1, 250], [0, -1, 0, -150], [-1, 0, 0, 200], [0, 0, 0, 1.0]])
T_DE = np.array([[0, 0, -1, 300], [0, -1, 0, 100], [-1, 0, 0, 120], [0, 0, 0, 1.0]])
T_AD = np.array([[0, 0, -1, 400], [0, -1, 0, 50], [-1, 0, 0, 300], [0, 0, 0, 1.0]])
T_BC = np.array([[0, -S2, -S2, 30], [0, S2, -S2, -40], [1, 0, 0, 25], [0, 0, 0, 1.0]])
T_CE_MR = np.array([[0, 0, 1, -75], [-S2, S2, 0, -260 * S2], [-S2, -S2, 0, 160 * S2], [0, 0, 0, 1.0]])


def load_nav():
    """Built inside each test (not as a fixture) so that the starter fails as a test, not an error."""
    return lab.load_tree_json(HERE / "nav_tree.json")


@pytest.fixture(scope="module")
def track():
    return lab.load_track_csv(HERE / "tool_track.csv")


# ------------------------------------------------------------- Checkpoint 1: SE(3) basics
def test_rp_to_trans_and_back():
    for T in RANDOM_T[:5]:
        R, p = mr.TransToRp(T)
        np.testing.assert_allclose(lab.rp_to_trans(R, p), T, atol=1e-12)
        R2, p2 = lab.trans_to_rp(T)
        np.testing.assert_allclose(R2, R, atol=1e-12)
        np.testing.assert_allclose(p2, p, atol=1e-12)


def test_trans_inv_matches_mr():
    for T in RANDOM_T:
        np.testing.assert_allclose(lab.trans_inv(T), mr.TransInv(T), atol=1e-12)
        np.testing.assert_allclose(lab.trans_inv(T) @ T, np.eye(4), atol=1e-12)


def test_is_se3():
    assert lab.is_se3(RANDOM_T[0])
    bad = RANDOM_T[0].copy(); bad[:3, :3] *= -1           # det R = -1 (a reflection)
    assert not lab.is_se3(bad)
    bad2 = RANDOM_T[0].copy(); bad2[3, 0] = 0.1           # wrong last row
    assert not lab.is_se3(bad2)
    assert not lab.is_se3(np.eye(3))


def test_mr_fig_3_14_frames_and_point():
    """T_bc = T_sb^-1 T_sc (MR p. 90) and v_b = (0, 0, 1.5) -> v_s."""
    T_bc = lab.trans_inv(T_SB_314) @ T_SC_314
    np.testing.assert_allclose(T_bc, T_BC_314, atol=1e-12)
    np.testing.assert_allclose(lab.transform_point(T_SB_314, [0, 0, 1.5]), [1.5, -2, 0], atol=1e-12)
    pts = RNG.normal(size=(7, 3))
    expected = (T_SB_314 @ np.c_[pts, np.ones(7)].T).T[:, :3]
    np.testing.assert_allclose(lab.transform_point(T_SB_314, pts), expected, atol=1e-12)


def test_mr_fig_3_15_fixed_vs_body_frame():
    """Pre-multiplication = fixed-frame displacement, post-multiplication = body-frame (MR Eqs. 3.66-3.67)."""
    T = lab.rp_to_trans(mr.MatrixExp3(mr.VecToso3([0, 0, np.pi / 2])), [0, 2, 0])
    T_sb = np.array([[0, 0, 1, 0], [0, -1, 0, -2], [1, 0, 0, 0], [0, 0, 0, 1.0]])
    np.testing.assert_allclose(T @ T_sb, [[0, 1, 0, 2], [0, 0, 1, 2], [1, 0, 0, 0], [0, 0, 0, 1]], atol=1e-12)
    np.testing.assert_allclose(T_sb @ T, [[0, 0, 1, 0], [-1, 0, 0, -4], [0, -1, 0, 0], [0, 0, 0, 1]], atol=1e-12)


def test_distance_preserved():
    """MR Prop. 3.18(a): ||Tx - Ty|| = ||x - y||."""
    x, y = RNG.normal(size=3), RNG.normal(size=3)
    for T in RANDOM_T[:5]:
        assert np.isclose(np.linalg.norm(lab.transform_point(T, x) - lab.transform_point(T, y)),
                          np.linalg.norm(x - y))


# ------------------------------------------------------------- Checkpoint 2: se(3) and adjoint
def test_vec_to_se3_matches_mr():
    for _ in range(5):
        V = RNG.normal(size=6)
        np.testing.assert_allclose(lab.vec_to_se3(V), mr.VecTose3(V), atol=1e-12)
        np.testing.assert_allclose(lab.se3_to_vec(mr.VecTose3(V)), V, atol=1e-12)


def test_adjoint_matches_mr_and_properties():
    for T1, T2 in zip(RANDOM_T[:10], RANDOM_T[10:]):
        np.testing.assert_allclose(lab.adjoint(T1), mr.Adjoint(T1), atol=1e-12)
        np.testing.assert_allclose(lab.adjoint(T1) @ lab.adjoint(T2), lab.adjoint(T1 @ T2), atol=1e-10)  # (3.77)
        np.testing.assert_allclose(np.linalg.inv(lab.adjoint(T1)), lab.adjoint(mr.TransInv(T1)), atol=1e-10)  # (3.78)
        V = RNG.normal(size=6)                     # [Ad_T V] = T [V] T^-1  (MR Def. 3.20)
        np.testing.assert_allclose(mr.VecTose3(lab.adjoint(T1) @ V), T1 @ mr.VecTose3(V) @ mr.TransInv(T1),
                                   atol=1e-10)


# ------------------------------------------------------------- Checkpoint 3: twists and screws
def test_twists_from_T_Tdot_mr_example_3_23():
    """MR Example 3.23 (printed pp. 99-100): the three-wheeled car."""
    T_sb = np.array([[-1, 0, 0, 4], [0, 1, 0, 0.4], [0, 0, -1, 0], [0, 0, 0, 1.0]])
    V_b = np.array([0, 0, -2, 2.8, 4, 0])
    Tdot = T_sb @ mr.VecTose3(V_b)
    V_s, V_b2 = lab.twists_from_T_Tdot(T_sb, Tdot)
    np.testing.assert_allclose(V_s, [0, 0, 2, -2, -4, 0], atol=1e-12)
    np.testing.assert_allclose(V_b2, V_b, atol=1e-12)
    np.testing.assert_allclose(mr.Adjoint(T_sb) @ V_b2, V_s, atol=1e-12)


def test_screw_to_twist_and_back():
    V = lab.screw_to_twist([2, -1, 0], [0, 0, 1], 0.0, 2.0)          # MR Example 3.23, V_s
    np.testing.assert_allclose(V, [0, 0, 2, -2, -4, 0], atol=1e-12)
    sc = lab.twist_to_screw(V)
    assert np.isclose(sc["thetadot"], 2) and np.isclose(sc["h"], 0)
    np.testing.assert_allclose(sc["q"], [2, -1, 0], atol=1e-12)
    np.testing.assert_allclose(sc["S"] * sc["thetadot"], V, atol=1e-12)
    for _ in range(10):                                              # random screws, q off the origin
        s = RNG.normal(size=3); s /= np.linalg.norm(s)
        q = RNG.normal(size=3); h = RNG.normal(); td = abs(RNG.normal()) + 0.1
        sc = lab.twist_to_screw(lab.screw_to_twist(q, s, h, td))
        assert np.isclose(sc["h"], h) and np.isclose(sc["thetadot"], td)
        np.testing.assert_allclose(sc["s_hat"], s, atol=1e-10)
        assert lab.point_line_distance(q, sc["q"], s) < 1e-10      # same line
        assert abs(sc["q"] @ s) < 1e-10                            # q is the point closest to the origin


def test_pure_translation_and_zero_twist():
    sc = lab.twist_to_screw([0, 0, 0, 0, 0, 0.05])
    assert np.isinf(sc["h"]) and sc["q"] is None and np.isclose(sc["thetadot"], 0.05)
    np.testing.assert_allclose(sc["S"], [0, 0, 0, 0, 0, 1], atol=1e-12)
    np.testing.assert_allclose(lab.screw_to_twist(None, [0, 0, 2], np.inf, 0.05), [0, 0, 0, 0, 0, 0.05])
    with pytest.raises(ValueError):
        lab.twist_to_screw(np.zeros(6))


def test_pitch_is_frame_invariant():
    V = RNG.normal(size=6)
    h = lab.twist_to_screw(V)["h"]
    for T in RANDOM_T[:5]:
        assert np.isclose(lab.twist_to_screw(mr.Adjoint(T) @ V)["h"], h)


# ------------------------------------------------------------- Checkpoint 4: transform tree
def test_tree_mr_example_3_19():
    tree = lab.TransformTree()
    tree.add("a", "d", T_AD)
    tree.add("d", "b", T_DB)
    tree.add("d", "e", T_DE)
    tree.add("b", "c", T_BC)
    np.testing.assert_allclose(tree.lookup("c", "e"), T_CE_MR, atol=1e-9)
    np.testing.assert_allclose(tree.lookup("a", "e"), T_AD @ T_DE, atol=1e-9)
    np.testing.assert_allclose(tree.lookup("a", "c"), T_AD @ T_DB @ T_BC, atol=1e-9)
    np.testing.assert_allclose(tree.lookup("e", "c"), mr.TransInv(T_CE_MR), atol=1e-9)
    np.testing.assert_allclose(tree.lookup("b", "b"), np.eye(4), atol=1e-12)
    assert tree.path("c", "e") == ["c", "b", "d", "e"]


def test_tree_rejects_bad_edges():
    tree = lab.TransformTree()
    tree.add("world", "a", RANDOM_T[0])
    tree.add("a", "b", RANDOM_T[1])
    tree.add("a", "b", RANDOM_T[2])                                   # update is allowed
    np.testing.assert_allclose(tree.lookup("a", "b"), RANDOM_T[2], atol=1e-12)
    with pytest.raises(ValueError):
        tree.add("world", "b", RANDOM_T[3])                          # second parent
    with pytest.raises(ValueError):
        tree.add("b", "world", RANDOM_T[3])                          # cycle
    with pytest.raises(ValueError):
        tree.add("b", "c", 2 * RANDOM_T[3])                          # not in SE(3)
    with pytest.raises(KeyError):
        tree.lookup("world", "nowhere")
    tree.add("other_root", "z", RANDOM_T[4])
    with pytest.raises(LookupError):
        tree.lookup("world", "z")


def test_quaternion_order_and_json_loader():
    nav = load_nav()
    for s in range(5):
        r = Rotation.random(random_state=s)
        np.testing.assert_allclose(lab.quat_xyzw_to_rot(r.as_quat()), r.as_matrix(), atol=1e-12)
    assert {"theatre", "tracker", "patient_ref", "ct_image", "plan_entry", "tool_marker",
            "tool_tip"} <= nav.frames


def test_eod_tree_matches_workshop_problem_3():
    eod = lab.load_tree_json(HERE / "eod_tree.json")
    T_ce = eod.lookup("gripper", "package")
    np.testing.assert_allclose(T_ce[:3, 3], [0.1, -0.1, 0.5], atol=1e-9)
    np.testing.assert_allclose(T_ce[:3, :3], [[0, 0, -1], [0, 1, 0], [1, 0, 0]], atol=1e-9)
    np.testing.assert_allclose(eod.lookup("base_link", "package")[:3, 3], [1.4, -0.1, 0.3], atol=1e-9)


# ------------------------------------------------------------- Checkpoint 5: surgical navigation
def test_tool_tip_in_ct_at_start(track):
    nav = load_nav()
    t, Ts = track
    nav.add("tracker", "tool_marker", Ts[0])
    T = nav.lookup("ct_image", "tool_tip")
    direct = (mr.TransInv(nav.lookup("tracker", "ct_image")) @ Ts[0] @ nav.lookup("tool_marker", "tool_tip"))
    np.testing.assert_allclose(T, direct, atol=1e-12)
    np.testing.assert_allclose(T[:3, 3] * 1000, [142.0, 84.035, 210.262], atol=1e-2)       # mm
    tip_plan = nav.lookup("plan_entry", "tool_tip")[:3, 3] * 1000
    assert np.isclose(np.linalg.norm(tip_plan), 1.0, atol=1e-3)                            # 1.0 mm entry error


def test_body_twist_from_tracking_data(track):
    t, Ts = track
    dt = t[1] - t[0]
    Vs, Vb = lab.twists_from_samples(Ts, dt)
    assert Vs.shape == Vb.shape == (len(t) - 2, 6)
    exact = mr.se3ToVec(mr.MatrixLog6(mr.TransInv(Ts[0]) @ Ts[1])) / dt     # constant screw motion
    np.testing.assert_allclose(Vb, np.tile(exact, (len(Vb), 1)), rtol=0, atol=5e-4)
    for k in (0, 150, len(Vb) - 1):                                          # V_s = [Ad_Tsb] V_b
        np.testing.assert_allclose(Vs[k], mr.Adjoint(Ts[k + 1]) @ Vb[k], atol=1e-9)


def test_screw_axis_and_lead_of_the_screwdriver(track):
    nav = load_nav()
    t, Ts = track
    Vs, Vb = lab.twists_from_samples(Ts, t[1] - t[0])
    sc_b = lab.twist_to_screw(Vb.mean(axis=0))
    assert np.isclose(sc_b["h"] * 2 * np.pi * 1000, 2.5, atol=0.01)          # lead in mm/rev
    assert np.isclose(sc_b["thetadot"], np.pi / 2, rtol=1e-3)
    np.testing.assert_allclose(sc_b["s_hat"], [0, 0, -1], atol=1e-6)
    np.testing.assert_allclose(sc_b["q"], [0.05, 0, 0], atol=1e-6)
    V_ct = lab.adjoint(nav.lookup("ct_image", "tracker")) @ Vs.mean(axis=0)
    sc = lab.twist_to_screw(V_ct)
    T_plan = nav.lookup("ct_image", "plan_entry")
    assert np.isclose(np.rad2deg(lab.axis_angle_between(sc["s_hat"], T_plan[:3, 2])), 2.0, atol=0.01)
    assert np.isclose(lab.point_line_distance(T_plan[:3, 3], sc["q"], sc["s_hat"]) * 1000, 1.0, atol=0.01)
