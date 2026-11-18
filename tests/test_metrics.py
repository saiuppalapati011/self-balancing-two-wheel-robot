"""Tests for the metrics module."""
import math
import numpy as np
import pandas as pd
import pytest
from balancing_robot.analysis.metrics import compute_metrics, SETTLING_THRESHOLD_RAD


def make_df(time, tilt_rad, controller="pid", status=None, x_m=None, left_torque_cmd=1.0, right_torque_cmd=1.0):
    n = len(time)
    if status is None:
        status = ["BALANCING"] * n
    if x_m is None:
        x_m = [0.0] * n
    df = pd.DataFrame({
        "experiment_id": ["E_test"] * n,
        "controller": [controller] * n,
        "run_id": ["test"] * n,
        "seed": [42] * n,
        "time_s": time,
        "tilt_rad": tilt_rad,
        "tilt_deg": [math.degrees(t) for t in tilt_rad],
        "tilt_rate_rad_s": [0.0] * n,
        "x_m": x_m,
        "x_vel_m_s": [0.0] * n,
        "left_wheel_vel_rad_s": [0.0] * n,
        "right_wheel_vel_rad_s": [0.0] * n,
        "left_torque_cmd_Nm": [left_torque_cmd] * n if isinstance(left_torque_cmd, (int, float)) else left_torque_cmd,
        "right_torque_cmd_Nm": [right_torque_cmd] * n if isinstance(right_torque_cmd, (int, float)) else right_torque_cmd,
        "left_torque_applied_Nm": [0.9] * n,
        "right_torque_applied_Nm": [0.9] * n,
        "disturbance_force_N": [0.0] * n,
        "status": status,
    })
    return df


def test_metrics_perfect_balance():
    """Perfect balance: all zeros → success, zero metrics."""
    time = np.linspace(0, 10, 2400)
    tilt = np.zeros_like(time)
    df = make_df(list(time), list(tilt))
    m = compute_metrics(df)
    assert m["success"] is True
    assert m["max_abs_tilt_rad"] == pytest.approx(0.0, abs=1e-6)
    assert m["rms_tilt_rad"] == pytest.approx(0.0, abs=1e-6)
    assert m["settling_time_s"] == pytest.approx(0.0, abs=1e-6)


def test_metrics_failed_run():
    """Run with FELL status at end."""
    time = [0.0, 0.1, 0.2]
    tilt = [0.05, 0.3, 0.9]
    status = ["BALANCING", "BALANCING", "FELL"]
    df = make_df(time, tilt, status=status)
    m = compute_metrics(df)
    assert m["success"] is False
    assert m["fall_time_s"] == pytest.approx(0.2, abs=1e-6)


def test_metrics_rms_tilt():
    """RMS of a constant tilt."""
    time = np.linspace(0, 5, 100)
    tilt = np.full_like(time, 0.1)  # constant 0.1 rad
    df = make_df(list(time), list(tilt))
    m = compute_metrics(df)
    assert m["rms_tilt_rad"] == pytest.approx(0.1, abs=1e-4)


def test_metrics_settling_time():
    """Settling time: tilt drops below threshold at known time."""
    time = np.linspace(0, 10, 1000)
    # Tilt starts at 0.1, decays to 0 after t=5s
    tilt = np.where(time < 5.0, 0.1, 0.0)
    df = make_df(list(time), list(tilt))
    m = compute_metrics(df)
    assert m["settling_time_s"] is not None
    assert m["settling_time_s"] <= 5.1  # settled around t=5


def test_metrics_empty_df():
    """Empty DataFrame should return failed metrics without error."""
    df = pd.DataFrame()
    m = compute_metrics(df)
    assert m["success"] is False


def test_metrics_never_settles():
    """System oscillates outside tolerance band and never settles."""
    time = np.linspace(0, 10, 1000)
    # Oscillate with offset so tilt is always >= 0.01 rad (0.57 deg), strictly exceeding 0.5 deg (0.008726 rad)
    tilt = 0.03 + 0.02 * np.sin(2.0 * np.pi * 1.0 * time)
    df = make_df(list(time), list(tilt))
    m = compute_metrics(df)
    assert m["success"] is True
    assert m["settling_time_s"] is None
    assert m["max_abs_tilt_rad"] == pytest.approx(0.05, abs=1e-3)


def test_metrics_settles_at_tolerance_boundary():
    """System settles exactly at the tolerance boundary (0.5 deg = SETTLING_THRESHOLD_RAD)."""
    time = np.linspace(0, 10, 1001)
    # Starts above boundary (0.02 rad), then drops exactly onto boundary at t=4.0s
    tilt = np.where(time < 4.0, 0.02, SETTLING_THRESHOLD_RAD)
    df = make_df(list(time), list(tilt))
    m = compute_metrics(df)
    assert m["success"] is True
    assert m["settling_time_s"] is not None
    assert m["settling_time_s"] == pytest.approx(4.0, abs=0.02)

    # When slightly above boundary (boundary + 1e-5), it must NOT settle
    tilt_above = np.where(time < 4.0, 0.02, SETTLING_THRESHOLD_RAD + 1e-5)
    df_above = make_df(list(time), list(tilt_above))
    m_above = compute_metrics(df_above)
    assert m_above["settling_time_s"] is None


def test_metrics_fall_midway():
    """Fall occurring midway through experiment (e.g. at t=3.5s out of 10s)."""
    time = np.linspace(0, 10, 101)
    tilt = np.where(time < 3.5, 0.03, 0.85)  # 0.85 rad ~ 48.7 deg > 45 deg fall limit
    status = ["BALANCING" if t < 3.5 else "FELL" for t in time]
    df = make_df(list(time), list(tilt), status=status)
    m = compute_metrics(df)
    assert m["success"] is False
    assert m["fall_time_s"] == pytest.approx(3.5, abs=0.1)
    assert m["settling_time_s"] is None
    assert m["max_abs_tilt_deg"] > 45.0


def test_metrics_single_sample_df():
    """Single-sample telemetry dataframe."""
    time = [0.0]
    tilt = [0.004]  # 0.004 rad < SETTLING_THRESHOLD_RAD
    df = make_df(time, tilt, x_m=[0.05])
    m = compute_metrics(df)
    assert m["success"] is True
    assert m["settling_time_s"] == pytest.approx(0.0, abs=1e-6)
    assert m["overshoot_rad"] == 0.0
    assert m["overshoot_deg"] == 0.0
    assert m["max_abs_tilt_rad"] == pytest.approx(0.004, abs=1e-6)
    assert m["rms_tilt_rad"] == pytest.approx(0.004, abs=1e-6)
    assert m["ss_tilt_error_rad"] == pytest.approx(0.004, abs=1e-6)
    assert m["final_x_m"] == pytest.approx(0.05, abs=1e-6)


def test_metrics_nonuniform_timestamps():
    """Telemetry with irregular, non-uniform timestamps."""
    time = [0.0, 0.02, 0.08, 0.25, 0.80, 2.10, 4.50, 7.00, 8.50, 10.00]
    # Tilt drops and settles permanently after t=4.5s
    tilt = [0.10, 0.09, 0.07, 0.05, 0.02, 0.015, 0.005, 0.003, 0.002, 0.001]
    df = make_df(time, tilt)
    m = compute_metrics(df)
    assert m["success"] is True
    assert m["settling_time_s"] == pytest.approx(4.50, abs=1e-5)
    # Steady state window is last 5 seconds: time >= (10.0 - 5.0) = 5.0s -> samples at 7.0, 8.5, 10.0
    expected_ss = np.mean([0.003, 0.002, 0.001])
    assert m["ss_tilt_error_rad"] == pytest.approx(expected_ss, abs=1e-5)


def test_metrics_re_exit_settling_band():
    """System enters settling band, exits it, and then settles permanently."""
    time = np.linspace(0, 10, 1001)
    # Starts high (0.05), briefly enters band between 2s and 3s, exits (0.02) between 3s and 6s,
    # and permanently settles (<0.005) after 6.0s.
    tilt = np.zeros_like(time)
    for idx, t in enumerate(time):
        if t < 2.0:
            tilt[idx] = 0.05
        elif t < 3.0:
            tilt[idx] = 0.004  # inside band
        elif t < 6.0:
            tilt[idx] = 0.025  # exited band
        else:
            tilt[idx] = 0.002  # permanently inside band
    df = make_df(list(time), list(tilt))
    m = compute_metrics(df)
    assert m["success"] is True
    # Settling time must be the FIRST time it stays inside for ALL future times (i.e. t=6.0s)
    assert m["settling_time_s"] == pytest.approx(6.0, abs=0.05)


def test_metrics_overshoot_with_negative_lean():
    """Negative initial tilt swinging to positive tilt correctly measures overshoot."""
    time = [0.0, 0.5, 1.0, 1.5, 2.0]
    # Initial tilt is -0.04 rad; swings to +0.065 rad, then settles
    tilt = [-0.04, -0.01, 0.065, 0.02, 0.002]
    df = make_df(time, tilt)
    m = compute_metrics(df)
    assert m["max_abs_tilt_rad"] == pytest.approx(0.065, abs=1e-6)
    # overshoot = max(0, max_abs_tilt - initial_abs_tilt) = 0.065 - 0.04 = 0.025
    assert m["overshoot_rad"] == pytest.approx(0.025, abs=1e-6)
