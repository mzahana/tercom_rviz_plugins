# tercom_rviz_plugins

Custom RViz2 panel plugins for real-time diagnostics of the TERCOM GPS-denied navigation system.
Provides three dockable panels and a reusable sparkline widget, all built with Qt5 and registered
via `pluginlib` so RViz2 discovers them automatically.

---


## Requirements

### ROS 2 dependencies (resolved by `rosdep`)

| Package | Provided by |
|---------|-------------|
| `rclcpp` | `ros-humble-rclcpp` |
| `rviz_common` | `ros-humble-rviz-common` |
| `pluginlib` | `ros-humble-pluginlib` |
| `std_msgs` | `ros-humble-std-msgs` |
| `geometry_msgs` | `ros-humble-geometry-msgs` |
| `visualization_msgs` | `ros-humble-visualization-msgs` |

### System dependencies

| Library | Ubuntu package | Required for |
|---------|---------------|-------------|
| Qt5 Widgets | `qtbase5-dev` | All panels |
| Qt5 Charts | `libqt5charts5-dev` | `ErrorHistoryPanel` only |


In case the following is not installed:
```bash
apt-get install -y qtbase5-dev libqt5charts5-dev
```

---

## Build


```bash
# Build this package only
colcon build --packages-select tercom_rviz_plugins

# Build together with tercom_nav
colcon build --packages-select tercom_nav tercom_rviz_plugins
```



---

## Panels

### FilterStatusPanel

Monitors the Error-State Kalman Filter (ESKF) in real time.

| Section | Content |
|---------|---------|
| **State badge** | Colour-coded label: `INITIALIZING` (grey) · `GPS_INIT` (blue) · `RUNNING` (green) · `DEGRADED` (orange) · `FAILED` (red) |
| **NIS sparkline** | Last N Normalised Innovation Squared samples with a χ² threshold line at 5.99 (2-DOF, 95 %). Turns red when the filter is inconsistent. |
| **Health gauges** | Position σ (limit 50 m) and innovation norm (limit 3.0) shown as colour-coded progress bars (green < 60 % · yellow < 85 % · red ≥ 85 %). |
| **IMU bias grid** | Accelerometer and gyroscope bias components (m/s² and rad/s). Normal white · orange > 0.02 · red > 0.05. |

**Subscribed topics**

| Topic | Type | Description |
|-------|------|-------------|
| `/tercom/eskf_node/state` | `std_msgs/String` | Filter state string |
| `/tercom/eskf_node/health` | `std_msgs/Float32MultiArray` | `[σ_pos, innov_norm]` |
| `/tercom/diagnostics_node/nis_history` | `std_msgs/Float32MultiArray` | Rolling NIS window |
| `/tercom/diagnostics_node/bias_accel` | `geometry_msgs/Vector3Stamped` | Accel bias (m/s²) |
| `/tercom/diagnostics_node/bias_gyro` | `geometry_msgs/Vector3Stamped` | Gyro bias (rad/s) |

---

### TercomQualityPanel

Shows the TERCOM terrain-correlation pipeline status and per-match quality metrics.

| Section | Content |
|---------|---------|
| **Pipeline status** | Text label (`WAITING_SENSORS` · `COLLECTING N/M` with progress bar · `MATCHING`) |
| **Match metrics** | Four colour-coded progress bars: MAD (m), Discrimination, Roughness (m), Noise scale |
| **Decision badge** | `ACCEPTED` (green) · `ACCEPTED (marginal)` (yellow) · `REJECTED` (red) |
| **Session stats** | Running count of accepted / rejected fixes and per-criterion rejection breakdown |

**Rejection thresholds**

| Metric | Reject when |
|--------|------------|
| MAD | > 30 m |
| Discrimination | < 1.02 |
| Roughness | < 5 m |

**Subscribed topics**

| Topic | Type | Description |
|-------|------|-------------|
| `/tercom/tercom_node/status` | `std_msgs/String` | Pipeline state string |
| `/tercom/tercom_node/match_quality` | `std_msgs/Float32MultiArray` | `[MAD, discrimination, roughness, noise_scale]` |

---

### ErrorHistoryPanel

Live navigation-error charts with ground-truth comparison.

| Section | Content |
|---------|---------|
| **Horizontal error chart** | Three rolling `QLineSeries`: current (blue) · RMS (red dashed) · max (orange dotted) + dashed threshold at 50 m |
| **Vertical error chart** | Signed altitude error (purple) + dashed threshold at 20 m |
| **Summary table** | Now / RMS / Max / Mean values colour-coded: green < 20 m · yellow 20–50 m · red > 50 m |

**Message layout** (`std_msgs/Float32MultiArray`)

```
error_history_chart:  [N_h, N_v, h_0…h_(N_h-1), v_0…v_(N_v-1), thresh_h, thresh_v]
error_history_stats:  [now, rms, max, mean]
```

X-axis tick spacing: 0.5 s per sample. Y-axis auto-scales to 1.2 × the maximum value.

**Subscribed topics**

| Topic | Type | Description |
|-------|------|-------------|
| `/tercom/diagnostics_node/error_history_chart` | `std_msgs/Float32MultiArray` | Packed chart data (see above) |
| `/tercom/diagnostics_node/error_history_stats` | `std_msgs/Float32MultiArray` | Summary statistics |

---

### ProfilingPanel

Displays execution time and call frequency for every timed component across all three
`tercom_nav` nodes.  Use this panel to baseline performance on a new compute platform and to
spot bottlenecks before they affect navigation quality.

#### Column definitions

| Column | What it measures |
|--------|-----------------|
| **Exec (ms)** | Average wall-clock time the function body spent running (from `start()` to `stop()` inside the callback). This is pure computation time — time spent blocked on I/O, sleeping, or waiting for the next message is **not** included. |
| **Rate (Hz)** | Average call frequency — how many times per second the function is *invoked*, computed from the rolling average of inter-call intervals. |

These two numbers are **completely independent**:

- `cb_synced` showing `0.05 ms / 9.9 Hz` means the callback fires ~10 times per second
  (driven by sensor message rate), but each invocation only spends 0.05 ms doing work.
  The remaining ~99.95 ms per cycle is idle waiting time between messages — that is normal
  and expected.
- `run_matching` might show `180 ms / 9.9 Hz`. That means matching is being triggered at the
  same sensor rate but each match takes 180 ms. If exec time approaches or exceeds
  `1 / Rate`, the node cannot keep up and will start dropping callbacks.

**Rule of thumb**: `Exec (ms) × Rate (Hz) / 1000 > 0.8` (i.e. > 80 % CPU utilisation on a
single core) is the threshold at which a component becomes a bottleneck.

#### Color coding (Exec column)

| Color | Meaning |
|-------|---------|
| Green | Exec time is within the expected fast range for this component |
| Amber | Exec time is elevated — monitor closely |
| Red | Exec time exceeds the performance budget — bottleneck |
| Dark grey | No data received yet (component not running) |

Per-component thresholds are tuned to realistic expectations:

| Component | Green limit | Red above |
|-----------|------------|-----------|
| tercom: cb_synced | 5 ms | 20 ms |
| tercom: run_matching | 100 ms | 500 ms |
| eskf: cb_imu | 1 ms | 5 ms |
| eskf: cb_tercom_fix | 2 ms | 10 ms |
| eskf: cb_altitude | 1 ms | 5 ms |
| diag: timer_error | 10 ms | 50 ms |
| diag: timer_paths | 20 ms | 100 ms |
| diag: timer_stats | 5 ms | 20 ms |

**Subscribed topics**

| Topic | Type | Description |
|-------|------|-------------|
| `/tercom/diagnostics_node/profiling` | `std_msgs/Float32MultiArray` | 16-float aggregated timing message (see below) |

**Message layout** (`/tercom/diagnostics_node/profiling`, 16 floats)

```
[0-1]   tercom_node : cb_synced      → [exec_ms, hz]
[2-3]   tercom_node : run_matching   → [exec_ms, hz]
[4-5]   eskf_node   : cb_imu         → [exec_ms, hz]
[6-7]   eskf_node   : cb_tercom_fix  → [exec_ms, hz]
[8-9]   eskf_node   : cb_altitude    → [exec_ms, hz]
[10-11] diag_node   : timer_error    → [exec_ms, hz]
[12-13] diag_node   : timer_paths    → [exec_ms, hz]
[14-15] diag_node   : timer_stats    → [exec_ms, hz]
```

The aggregated topic is assembled by `diagnostics_node`, which subscribes to
`/tercom/tercom_node/timing` (4 floats) and `/tercom/eskf_node/timing` (6 floats) and adds
its own three timer measurements before publishing.

---

## SparklineWidget (internal)

A lightweight `QWidget` subclass that renders a line chart with `QPainter` — no QtCharts
dependency. Intended for use inside other panels.

```cpp
#include "tercom_rviz_plugins/sparkline_widget.hpp"

auto* w = new tercom_rviz_plugins::SparklineWidget(parent);
w->setLabel("NIS");
w->setData(samples, /*threshold=*/5.99f);  // call from Qt main thread
```

---


## Usage

### Launch the full stack

```bash
ros2 launch tercom_nav tercom_nav.launch.py \
  params_file:=$(ros2 pkg prefix tercom_nav)/share/tercom_nav/config/taif_test4_params.yaml
```

### Open the dashboard

```bash
rviz2 -d $(ros2 pkg prefix tercom_nav)/share/tercom_nav/config/rviz_tercom.rviz
```

The pre-configured `rviz_tercom.rviz` file docks all three panels automatically:

| Panel | Position |
|-------|----------|
| Displays (built-in) | Left dock |
| Views (built-in) | Left dock |
| Filter Status | Side dock |
| TERCOM Quality | Side dock |
| Error History | Side dock |

### Add panels manually

If you open a blank RViz2 session, add panels via **Panels → Add New Panel** and select:

- `tercom_rviz_plugins/FilterStatusPanel`
- `tercom_rviz_plugins/TercomQualityPanel`
- `tercom_rviz_plugins/ErrorHistoryPanel`
- `tercom_rviz_plugins/ProfilingPanel`

---

## Package structure

```
tercom_rviz_plugins/
├── CMakeLists.txt
├── package.xml
├── plugin_description.xml          # pluginlib registration
├── README.md
├── include/tercom_rviz_plugins/
│   ├── sparkline_widget.hpp
│   ├── filter_status_panel.hpp
│   ├── tercom_quality_panel.hpp
│   ├── error_history_panel.hpp
│   └── profiling_panel.hpp
└── src/
    ├── sparkline_widget.cpp
    ├── filter_status_panel.cpp
    ├── tercom_quality_panel.cpp
    ├── error_history_panel.cpp
    └── profiling_panel.cpp
```

---

## Design notes

- **Thread safety**: All rclcpp subscription callbacks emit Qt signals. Widget updates happen
  exclusively in Qt main-thread slots, satisfying Qt's thread-affinity requirement.
- **CMake AUTOMOC**: `set(CMAKE_AUTOMOC ON)` is required so Qt's MOC tool processes `Q_OBJECT`
  macros in headers automatically.
- **Plugin registration**: `PLUGINLIB_EXPORT_CLASS` at the bottom of each `.cpp` file, plus
  `pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)` in
  `CMakeLists.txt`, makes the panels discoverable by RViz2 at runtime without code changes.
