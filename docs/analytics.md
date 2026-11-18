# Analytics Engineering Guide & System Documentation

**Role:** Member 5 – Analytics Engineer
**Project:** Self-Balancing Two-Wheel Robot (CSAI4014 – Project P09, Group G01, Batch B1)
**Package:** `src/balancing_robot/analysis/`

---

## 1. Overview of the Analytics Pipeline

The analytics pipeline processes raw telemetry produced by the PyBullet simulation, evaluates deterministic control-performance metrics, generates publication-quality figures for project reports, and compiles summary benchmark tables comparing PID and LQR controllers.

```
+-------------------------------------------------------------------------------+
|                             PyBullet Simulation Loop                          |
|                       (dt = 1/240 s, 240 Hz Physics & Control)                 |
+-------------------------------------------------------------------------------+
                                        |
                                        v
+-------------------------------------------------------------------------------+
|                       Raw Telemetry Storage (I/O Layer)                       |
|          results/data/<experiment_id>/<run_id>.csv & <run_id>.json            |
+-------------------------------------------------------------------------------+
                                        |
                                        v
+-------------------------------------------------------------------------------+
|                        Metrics Engine (metrics.py)                            |
|             Success, Settling Time, RMS Tilt, Drift, Torque Usage             |
+-------------------------------------------------------------------------------+
                       /                                 \
                      v                                   v
+----------------------------------+    +---------------------------------------+
|  Benchmark Tables (tables/*.csv) |    | Visualization Engine (plots.py)       |
|  - E01 Uncontrolled              |    | - Shaded ±0.5° settling band          |
|  - E02/E03 Nominal Comparison    |    | - Settling-time annotations           |
|  - E04 Multi-tilt Comparison     |    | - Torque saturation limits            |
|  - E05 Disturbance Rejection     |    | - Position drift profiles             |
|  - E06 Basin of Attraction Sweep |    | - Success/failure outcome maps        |
+----------------------------------+    +---------------------------------------+
                      \                                   /
                       v                                 v
+-------------------------------------------------------------------------------+
|                        Reporting & Terminal Summaries                         |
|             scripts/summarize_results.py & scripts/generate_phase3_assets.py  |
+-------------------------------------------------------------------------------+
```

---

## 2. Telemetry Storage: CSV and JSON Formats

Every simulation run records high-frequency state trajectories into a CSV file accompanied by a JSON sidecar recording complete runtime provenance.

### 2.1 CSV Telemetry Schema

The simulation outputs tabular data with 18 standardized columns:

| Column | Unit | Type | Description |
|---|---|---|---|
| `experiment_id` | - | string | Identifier of the experiment (e.g. `E01`, `E04`, `E_manual`) |
| `controller` | - | string | Active controller (`pid`, `lqr`, `none`) |
| `run_id` | - | string | Unique run name (e.g. `e02_pid_seed42`) |
| `seed` | - | integer | Random seed for reproducibility |
| `time_s` | s | float | Elapsed simulation time from $0$ to $t_{\text{duration}}$ |
| `tilt_rad` | rad | float | Pitch angle $\theta$ (0 = upright, $>0$ forward tilt) |
| `tilt_deg` | deg | float | Pitch angle converted to degrees |
| `tilt_rate_rad_s` | rad/s | float | Pitch angular velocity $\dot{\theta}$ |
| `x_m` | m | float | Chassis horizontal position along world $+X$ |
| `x_vel_m_s` | m/s | float | Chassis linear horizontal velocity $\dot{x}$ |
| `left_wheel_vel_rad_s` | rad/s | float | Left wheel rotational speed $\omega_L$ |
| `right_wheel_vel_rad_s` | rad/s | float | Right wheel rotational speed $\omega_R$ |
| `left_torque_cmd_Nm` | N·m | float | Commanded torque to left wheel motor |
| `right_torque_cmd_Nm` | N·m | float | Commanded torque to right wheel motor |
| `left_torque_applied_Nm` | N·m | float | Actual reaction torque measured from left joint |
| `right_torque_applied_Nm` | N·m | float | Actual reaction torque measured from right joint |
| `disturbance_force_N` | N | float | External horizontal force applied to chassis base |
| `status` | - | string | Operational status: `BALANCING` or `FELL` |

### 2.2 JSON Provenance Sidecar

Saved alongside the CSV (e.g., `run_id.json`), the metadata captures environmental parameters and configuration snapshots:

* `experiment_id`, `run_id`, `controller`, `seed`, `duration_s`, `dt`, `initial_tilt_deg`
* `saved_at`: Timezone-aware ISO-8601 timestamp
* `git_commit`: Active git commit SHA at time of execution
* `python_version`: Python interpreter version
* `pybullet_version`: PyBullet engine version
* `configs`: Complete dictionary snapshots of robot physical constants, simulation settings, controller gains ($K_p, K_i, K_d$ or $\mathbf{Q}, R$), and disturbance parameters.

---

## 3. Data Loading and Saving Workflow

The module [`balancing_robot.analysis.io`](file:///Users/amitsingh/Downloads/self-balancing-two-wheel-robot-main/src/balancing_robot/analysis/io.py) provides thread-safe, verified I/O routines:

* **Saving:** `save_run(rows, metadata, output_dir, run_id) -> tuple[Path, Path]`
  Writes `output_dir / f"{run_id}.csv"` and `output_dir / f"{run_id}.json"`. It enriches metadata with environment provenance and validates non-empty payloads.
* **Loading:** `load_run(csv_path) -> pd.DataFrame`
  Reads CSV telemetry into a pandas DataFrame, casting timestamps and state trajectories to float64.
* **Loading Metadata:** `load_metadata(json_path) -> dict`
  Parses runtime configurations and verification parameters.

---

## 4. Quantitative Performance Metrics

All performance metrics are computed in [`balancing_robot.analysis.metrics`](file:///Users/amitsingh/Downloads/self-balancing-two-wheel-robot-main/src/balancing_robot/analysis/metrics.py) using `compute_metrics(df, duration_s=None)`.

### 4.1 Success / Failure Status
* A robot is declared **fallen** if at any point the pitch angle exceeds the critical fall threshold:
  $$|\theta(t)| \ge \theta_{\text{fall}} = 45^\circ \approx 0.785398\text{ rad}$$
* If any row records `status == "FELL"`, `success = False` and `fall_time_s` records the timestamp of first failure:
  $$t_{\text{fall}} = \min \{ t \mid \text{status}(t) = \text{"FELL"} \}$$
* If no fall occurs throughout the run, `success = True` and `fall_time_s = None`.

### 4.2 Maximum Absolute Tilt
Peak angular excursion experienced during the trajectory:
$$\theta_{\max} = \max_{t \in [0, t_{\text{end}}]} |\theta(t)|$$
Reported in both radians (`max_abs_tilt_rad`) and degrees (`max_abs_tilt_deg`).

### 4.3 Settling Time
The earliest timestamp $t_s$ after which the pitch angle remains strictly within the tolerance band $\pm 0.5^\circ$ ($\theta_{\text{thresh}} = \frac{\pi}{360}\text{ rad} \approx 0.00872665\text{ rad}$) for **all future times**:
$$t_s = \min \left\{ t \;\middle|\; |\theta(t')| \le \theta_{\text{thresh}} \quad \forall t' \ge t \right\}$$
* If the trajectory never enters the band, leaves the band later, or topples, `settling_time_s = None`.

### 4.4 Steady-State Tilt Error
Mean absolute tilt evaluated over the final $5\text{ seconds}$ of the simulation run ($T_{\text{ss}} = 5.0\text{ s}$):
$$e_{\text{ss}} = \frac{1}{N_{5\text{s}}} \sum_{t_i \ge (t_{\text{end}} - 5.0)} |\theta(t_i)|$$
Reported as `ss_tilt_error_rad` and `ss_tilt_error_deg`.

### 4.5 Overshoot
For an initial tilt displacement $\theta_0 = |\theta(0)|$, overshoot quantifies transient deflection beyond the initial disturbance:
$$\text{Overshoot} = \max\left(0,\, \theta_{\max} - \theta_0\right)$$
Reported as `overshoot_rad` and `overshoot_deg`.

### 4.6 Root Mean Square (RMS) Tilt
Measures overall tilt deviation over the entire simulation duration:
$$\text{RMS}_{\theta} = \sqrt{\frac{1}{N} \sum_{i=1}^N \theta^2(t_i)}$$

### 4.7 Wheel Velocity Metrics
* **Peak Wheel Speed:** $v_{\omega, \max} = \max_{t} \max\left( |\omega_L(t)|, |\omega_R(t)| \right)$
* **RMS Wheel Speed:** $\text{RMS}_{\omega} = \sqrt{\frac{1}{2N} \sum_{i=1}^N \left( \omega_L^2(t_i) + \omega_R^2(t_i) \right)}$

### 4.8 Horizontal Displacement Metrics
* **Final Position:** $x_{\text{final}} = x(t_{\text{end}})$ (`final_x_m`)
* **Peak Displacement:** $x_{\max} = \max_t |x(t)|$ (`peak_x_m`)
* **Final Linear Speed:** $v_{x,\text{final}} = |\dot{x}(t_{\text{end}})|$ (`final_speed_m_s`)

### 4.9 Torque and Saturation Metrics
Total commanded torque applied across both wheels is $u(t) = \tau_L^{\text{cmd}}(t) + \tau_R^{\text{cmd}}(t)$:
* **Peak Total Torque:** $\tau_{\max} = \max_t |u(t)|$ (`peak_torque_Nm`)
* **RMS Total Torque:** $\text{RMS}_\tau = \sqrt{\frac{1}{N} \sum_{i=1}^N u^2(t_i)}$ (`rms_torque_Nm`)
* **Actuator Saturation Fraction:** Fraction of simulation steps where total commanded torque operated at or near actuator limits ($|u(t)| \ge 9.5\text{ N}\cdot\text{m}$ out of the $10.0\text{ N}\cdot\text{m}$ ceiling):
  $$\text{Fraction}_{\text{sat}} = \frac{1}{N} \sum_{i=1}^N \mathbf{1}_{|u(t_i)| \ge 9.5}$$

---

## 5. Visualization Generation

Plotting utilities are implemented in [`balancing_robot.analysis.plots`](file:///Users/amitsingh/Downloads/self-balancing-two-wheel-robot-main/src/balancing_robot/analysis/plots.py) and export publication-ready 300 DPI figures to `report/figures/`:

1. **`plot_tilt_vs_time(dfs, output_path, ...)`**:
   * **Settling Tolerance Band:** Renders a soft green shaded band (`axhspan`) spanning $[-0.5^\circ, +0.5^\circ]$.
   * **Upright Reference Line:** A dashed horizontal line at $0^\circ$.
   * **Settling-Time Annotation:** Automatically identifies when each controller permanently settles and annotates the timestamp and curve location with high-contrast colored callouts.
   * **Color Convention:** Consistent palette: PID = Blue (`#1f77b4`), LQR = Red (`#d62728`), Uncontrolled = Grey (`#7f7f7f`).
2. **`plot_torque_vs_time(dfs, output_path, ...)`**:
   * Displays total commanded torque $u(t)$ alongside the $\pm 10\text{ N}\cdot\text{m}$ saturation boundaries.
3. **`plot_position_drift(dfs, output_path, ...)`**:
   * Contrasts translational motion over time with reference at $x = 0\text{ m}$.
4. **`plot_tilt_sweep_map(results, output_path, ...)`**:
   * Binary success/fall outcome map across initial tilt angles ($1^\circ$ to $30^\circ$).

---

## 6. Benchmark Table Generation

Summary tables are produced by `scripts/generate_phase3_assets.py` from raw telemetry files in `results/data/`:

* `results/tables/E01_uncontrolled_metrics.csv`: Open-loop fall time ($0.542\text{ s}$) and toppling velocity.
* `results/tables/E02_E03_nominal_comparison.csv`: Baseline $30\text{ s}$ balancing from $3^\circ$ tilt comparing PID vs LQR.
* `results/tables/E04_comparison_table.csv`: Matched tilt comparisons at $2^\circ, 5^\circ, 10^\circ, 15^\circ$.
* `results/tables/E05_disturbance_metrics.csv`: $10\text{ N}$ impulse response showing peak transient deflection and post-disturbance drift.
* `results/tables/E06_tilt_sweep.csv`: Metrics across 12 initial tilts from $1^\circ$ through $30^\circ$.

---

## 7. Operational Commands

### 7.1 Running Analytics Tests
To execute unit tests for the metrics engine and plotting subsystem:
```bash
# Run all analytics tests (19 test cases)
pytest tests/test_metrics.py tests/test_io_and_plots.py -v

# Run the complete test suite (unit + physics)
pytest tests/ -m "not slow" -v
```

### 7.2 Generating Comparison Summary
To print a formatted terminal summary comparing PID and LQR benchmarks:
```bash
# Summarize default results/tables/
python scripts/summarize_results.py

# Summarize custom tables directory
python scripts/summarize_results.py --tables-dir results/tables
```

### 7.3 Regenerating Figures and Tables from Raw CSVs
To regenerate all benchmark tables and report figures from saved telemetry without running simulations:
```bash
python scripts/generate_phase3_assets.py
```
