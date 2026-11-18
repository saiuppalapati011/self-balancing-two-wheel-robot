"""Tests for analysis I/O and plot generation."""
import math
import numpy as np
import pandas as pd
import pytest
import json
from pathlib import Path
import tempfile
from balancing_robot.analysis.io import save_run, load_run, load_metadata
from balancing_robot.analysis.plots import (
    plot_tilt_vs_time,
    plot_torque_vs_time,
    plot_position_drift,
    plot_tilt_sweep_map,
)
from balancing_robot.simulation import CSV_COLUMNS


def make_sample_rows(n=100, controller="pid", status="BALANCING", settle=True):
    rows = []
    for i in range(n):
        t = i * 0.05
        # Exponential decay that settles within settling band (0.5 deg ~ 0.0087 rad)
        decay = math.exp(-1.5 * t) if settle else 1.0
        tilt_val = 0.05 * decay
        rows.append({
            "experiment_id": "E_test",
            "controller": controller,
            "run_id": f"test_run_{controller}",
            "seed": 42,
            "time_s": t,
            "tilt_rad": tilt_val,
            "tilt_deg": math.degrees(tilt_val),
            "tilt_rate_rad_s": -0.05 * decay,
            "x_m": 0.02 * t,
            "x_vel_m_s": 0.02,
            "left_wheel_vel_rad_s": 0.5,
            "right_wheel_vel_rad_s": 0.5,
            "left_torque_cmd_Nm": 1.0,
            "right_torque_cmd_Nm": 1.0,
            "left_torque_applied_Nm": 0.9,
            "right_torque_applied_Nm": 0.9,
            "disturbance_force_N": 0.0,
            "status": status,
        })
    return rows


def test_save_and_load_run():
    rows = make_sample_rows()
    metadata = {"controller": "pid", "duration_s": 1.0, "dt": 0.01}
    with tempfile.TemporaryDirectory() as tmpdir:
        csv_path, json_path = save_run(rows, metadata, tmpdir, "test_run")
        assert csv_path.exists()
        assert json_path.exists()
        assert csv_path.stat().st_size > 0
        df = load_run(csv_path)
        assert len(df) == 100
        assert "tilt_rad" in df.columns
        meta = load_metadata(json_path)
        assert meta["controller"] == "pid"
        assert meta["python_version"]
        assert "git_commit" in meta
        assert "saved_at" in meta


def test_csv_has_expected_columns():
    rows = make_sample_rows()
    with tempfile.TemporaryDirectory() as tmpdir:
        csv_path, _ = save_run(rows, {}, tmpdir, "col_test")
        df = load_run(csv_path)
        for col in CSV_COLUMNS:
            assert col in df.columns, f"Missing column: {col}"


def test_plot_tilt_vs_time():
    rows = make_sample_rows()
    df = pd.DataFrame(rows)
    with tempfile.TemporaryDirectory() as tmpdir:
        out = Path(tmpdir) / "tilt.png"
        plot_tilt_vs_time({"PID": df}, str(out))
        assert out.exists()
        assert out.stat().st_size > 0


def test_plot_tilt_vs_time_multi_controller_with_annotations():
    """Verify tilt plot with both PID and LQR, testing settling annotations and tolerance band."""
    pid_rows = make_sample_rows(n=100, controller="pid", settle=True)
    lqr_rows = make_sample_rows(n=100, controller="lqr", settle=True)
    dfs = {"PID": pd.DataFrame(pid_rows), "LQR": pd.DataFrame(lqr_rows)}
    with tempfile.TemporaryDirectory() as tmpdir:
        out = Path(tmpdir) / "comparison_tilt.png"
        plot_tilt_vs_time(dfs, str(out), title="Comparison")
        assert out.exists()
        assert out.stat().st_size > 0


def test_plot_tilt_vs_time_unsettled_and_fallen_runs():
    """Verify plot handles runs that fell or never settled without raising errors."""
    unsettled_rows = make_sample_rows(n=100, controller="pid", settle=False)
    fallen_rows = make_sample_rows(n=20, controller="none", status="FELL", settle=False)
    dfs = {"Unsettled": pd.DataFrame(unsettled_rows), "Fallen": pd.DataFrame(fallen_rows)}
    with tempfile.TemporaryDirectory() as tmpdir:
        out = Path(tmpdir) / "unsettled.png"
        plot_tilt_vs_time(dfs, str(out))
        assert out.exists()
        assert out.stat().st_size > 0


def test_plot_torque_vs_time():
    rows = make_sample_rows()
    df = pd.DataFrame(rows)
    with tempfile.TemporaryDirectory() as tmpdir:
        out = Path(tmpdir) / "torque.png"
        plot_torque_vs_time({"PID": df}, str(out))
        assert out.exists()
        assert out.stat().st_size > 0


def test_plot_position_drift_and_sweep_map():
    """Verify position drift and tilt sweep map plotting functions."""
    rows = make_sample_rows()
    df = pd.DataFrame(rows)
    with tempfile.TemporaryDirectory() as tmpdir:
        drift_out = Path(tmpdir) / "drift.png"
        plot_position_drift({"PID": df}, str(drift_out))
        assert drift_out.exists()
        assert drift_out.stat().st_size > 0

        map_out = Path(tmpdir) / "sweep_map.png"
        results = [
            {"controller": "pid", "initial_tilt_deg": 1.0, "success": True},
            {"controller": "pid", "initial_tilt_deg": 5.0, "success": True},
            {"controller": "lqr", "initial_tilt_deg": 1.0, "success": True},
            {"controller": "lqr", "initial_tilt_deg": 15.0, "success": True},
        ]
        plot_tilt_sweep_map(results, str(map_out))
        assert map_out.exists()
        assert map_out.stat().st_size > 0
