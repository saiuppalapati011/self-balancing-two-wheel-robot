"""
Plotting utilities for experiment results.

All plots read from saved CSV/JSON files only.
Consistent colours per controller: PID=blue, LQR=red, none=grey.
"""
from __future__ import annotations

import logging
import math
from pathlib import Path
from typing import Any

import matplotlib
matplotlib.use("Agg")  # headless backend
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from balancing_robot.analysis.metrics import SETTLING_THRESHOLD_RAD, _compute_settling_time

logger = logging.getLogger(__name__)

CONTROLLER_COLORS = {
    "pid": "#1f77b4",   # blue
    "lqr": "#d62728",   # red
    "none": "#7f7f7f",  # grey
    "PID": "#1f77b4",
    "LQR": "#d62728",
    "NONE": "#7f7f7f",
}
FIGURE_DPI = 300


def _get_color(controller: str) -> str:
    key = str(controller).strip()
    return CONTROLLER_COLORS.get(key, CONTROLLER_COLORS.get(key.lower(), "#2ca02c"))


def plot_tilt_vs_time(
    dfs: dict[str, pd.DataFrame],
    output_path: str | Path,
    title: str = "Tilt vs Time",
    xlabel: str = "Time (s)",
    ylabel: str = "Tilt (degrees)",
    show_settling_band: bool = True,
    annotate_settling: bool = True,
    settling_threshold_deg: float = 0.5,
) -> None:
    """Plot tilt angle vs time for one or more runs.

    Enhancements:
      - Shaded ±settling_threshold_deg band around 0°.
      - Horizontal reference line at zero tilt.
      - Settling-time annotation whenever a run settles within tolerance.
      - Publication-quality typography, grid, and legend styling.

    Args:
        dfs: dict of label → DataFrame (each with time_s, tilt_deg columns).
        output_path: Where to save the PNG.
        title, xlabel, ylabel: Figure labels.
        show_settling_band: Whether to render shaded settling band.
        annotate_settling: Whether to annotate settling time when detected.
        settling_threshold_deg: Settling tolerance in degrees (default: ±0.5°).
    """
    fig, ax = plt.subplots(figsize=(10, 5.5))

    # 1. Settling tolerance band
    if show_settling_band:
        ax.axhspan(
            -settling_threshold_deg,
            settling_threshold_deg,
            color="#2ca02c",
            alpha=0.15,
            label=f"Settling Band (±{settling_threshold_deg:.1f}°)",
            zorder=1,
        )

    # 2. Upright reference line at zero tilt
    ax.axhline(0, color="#333333", linewidth=0.9, linestyle="--", alpha=0.6, label="Upright Ref (0°)", zorder=2)

    threshold_rad = math.radians(settling_threshold_deg)

    # Plot each series
    for i, (label, df) in enumerate(dfs.items()):
        if df.empty or "time_s" not in df.columns or "tilt_deg" not in df.columns:
            continue
        ctrl = df["controller"].iloc[0] if "controller" in df.columns else label
        color = _get_color(ctrl)
        time = df["time_s"].values
        tilt_deg = df["tilt_deg"].values

        ax.plot(time, tilt_deg, label=label, color=color, linewidth=1.75, zorder=3)

        # 3. Settling-time annotation
        if annotate_settling:
            fell = any(s == "FELL" for s in df["status"]) if "status" in df.columns else False
            if not fell and len(tilt_deg) > 1:
                tilt_rad = df["tilt_rad"].values if "tilt_rad" in df.columns else np.radians(tilt_deg)
                t_settle = _compute_settling_time(time, tilt_rad, threshold_rad)
                if t_settle is not None:
                    ax.axvline(t_settle, color=color, linestyle=":", linewidth=1.2, alpha=0.75, zorder=2)

                    idx = int(np.argmin(np.abs(time - t_settle)))
                    y_pt = float(tilt_deg[idx])

                    offset_y = 20 if (i % 2 == 0) else -26
                    offset_x = 15 if t_settle < (time[-1] * 0.75) else -75

                    ax.annotate(
                        f"{label} Settled: {t_settle:.2f}s",
                        xy=(t_settle, y_pt),
                        xytext=(offset_x, offset_y),
                        textcoords="offset points",
                        fontsize=8.5,
                        fontweight="bold",
                        color=color,
                        bbox=dict(boxstyle="round,pad=0.25", fc="white", ec=color, alpha=0.92, lw=0.9),
                        arrowprops=dict(arrowstyle="->", color=color, lw=0.9, alpha=0.85),
                        zorder=5,
                    )

    ax.set_xlabel(xlabel, fontsize=11, fontweight="medium")
    ax.set_ylabel(ylabel, fontsize=11, fontweight="medium")
    ax.set_title(title, fontsize=12, fontweight="bold", pad=10)
    ax.legend(loc="best", frameon=True, framealpha=0.92, edgecolor="#cccccc")
    ax.grid(True, linestyle="--", alpha=0.4)
    _save(fig, output_path)


def plot_torque_vs_time(
    dfs: dict[str, pd.DataFrame],
    output_path: str | Path,
    title: str = "Torque vs Time",
    max_torque_limit: float = 10.0,
) -> None:
    """Plot total commanded torque vs time."""
    fig, ax = plt.subplots(figsize=(10, 5.5))
    for label, df in dfs.items():
        if df.empty or "time_s" not in df.columns:
            continue
        ctrl = df["controller"].iloc[0] if "controller" in df.columns else label
        color = _get_color(ctrl)
        torque_total = df["left_torque_cmd_Nm"] + df["right_torque_cmd_Nm"]
        ax.plot(df["time_s"], torque_total, label=label, color=color, linewidth=1.2)

    ax.axhline(max_torque_limit, color="black", linestyle=":", linewidth=0.8, alpha=0.5, label=f"Torque Limit (±{max_torque_limit:.0f} Nm)")
    ax.axhline(-max_torque_limit, color="black", linestyle=":", linewidth=0.8, alpha=0.5)
    ax.axhline(0, color="#666666", linestyle="--", linewidth=0.7, alpha=0.4)

    ax.set_xlabel("Time (s)", fontsize=11, fontweight="medium")
    ax.set_ylabel("Total Torque (Nm)", fontsize=11, fontweight="medium")
    ax.set_title(title, fontsize=12, fontweight="bold", pad=10)
    ax.legend(loc="best", frameon=True, framealpha=0.92, edgecolor="#cccccc")
    ax.grid(True, linestyle="--", alpha=0.4)
    _save(fig, output_path)


def plot_position_drift(
    dfs: dict[str, pd.DataFrame],
    output_path: str | Path,
    title: str = "Horizontal Position Drift vs Time",
) -> None:
    """Plot chassis x position vs time."""
    fig, ax = plt.subplots(figsize=(10, 5.5))
    for label, df in dfs.items():
        if df.empty or "time_s" not in df.columns or "x_m" not in df.columns:
            continue
        ctrl = df["controller"].iloc[0] if "controller" in df.columns else label
        color = _get_color(ctrl)
        ax.plot(df["time_s"], df["x_m"], label=label, color=color, linewidth=1.6)

    ax.axhline(0, color="black", linewidth=0.8, linestyle="--", alpha=0.5, label="Target Position (0 m)")
    ax.set_xlabel("Time (s)", fontsize=11, fontweight="medium")
    ax.set_ylabel("Horizontal Position (m)", fontsize=11, fontweight="medium")
    ax.set_title(title, fontsize=12, fontweight="bold", pad=10)
    ax.legend(loc="best", frameon=True, framealpha=0.92, edgecolor="#cccccc")
    ax.grid(True, linestyle="--", alpha=0.4)
    _save(fig, output_path)


def plot_tilt_sweep_map(
    results: list[dict],
    output_path: str | Path,
    title: str = "Initial Tilt Success Map",
) -> None:
    """Plot success/fail map vs initial tilt angle.

    Args:
        results: List of dicts with keys: controller, initial_tilt_deg, success.
        output_path: Where to save.
    """
    fig, ax = plt.subplots(figsize=(10, 5.5))
    controllers = sorted(set(r["controller"] for r in results))
    for ctrl in controllers:
        ctrl_results = sorted(
            (r for r in results if r["controller"] == ctrl),
            key=lambda result: float(result["initial_tilt_deg"]),
        )
        tilts = [float(r["initial_tilt_deg"]) for r in ctrl_results]
        successes = [1 if r["success"] else 0 for r in ctrl_results]
        color = _get_color(ctrl)
        ax.plot(tilts, successes, "o-", label=ctrl.upper(), color=color, linewidth=2.0, markersize=8)

    ax.set_xlabel("Initial Tilt Angle (degrees)", fontsize=11, fontweight="medium")
    ax.set_ylabel("Outcome", fontsize=11, fontweight="medium")
    ax.set_title(title, fontsize=12, fontweight="bold", pad=10)
    ax.set_yticks([0, 1])
    ax.set_yticklabels(["Fell", "Balanced"], fontsize=10, fontweight="bold")
    ax.legend(loc="best", frameon=True, framealpha=0.92, edgecolor="#cccccc")
    ax.grid(True, linestyle="--", alpha=0.4)
    _save(fig, output_path)


def _save(fig: plt.Figure, path: str | Path) -> None:
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(p, dpi=FIGURE_DPI, bbox_inches="tight")
    plt.close(fig)
    logger.info("Saved plot: %s", p)
