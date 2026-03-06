# hospital-agv-sim

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Sim](https://img.shields.io/badge/Sim-Ignition%20Gazebo%20Fortress-orange.svg)](https://gazebosim.org/docs/fortress)
[![Metrology](https://img.shields.io/badge/Research-Noise%20Models-gold.svg)](docs/RESEARCH.md)
[![License](https://img.shields.io/badge/License-MIT-lightgrey.svg)](LICENSE)

ROS 2 / Ignition Gazebo hospital simulation for the AgileX Tracer 2.  
The point is simple: make the simulated sensors misbehave in ways that look like the real hardware, so localization and SLAM fail in simulation for the same reasons they fail on the robot.

---

## Why this repo exists

In early tests with the real Tracer 2 in hospital corridors, localization worked nicely in “clean” simulation but degraded quickly on the robot. EKF and LiDAR-based SLAM were clearly too confident:

- simulated IMU had almost no bias or drift,
- LiDAR ranges were too perfect,
- wheel odometry did not accumulate the yaw error you actually see after a few tight turns.

This package adds three things on top of a standard ROS 2 simulation:

1. A hospital-scale world with corridors, corners and occlusions that look like a real building instead of a toy map.
2. A URDF/xacro model of the Tracer 2 with a consistent TF tree and sensor frames.
3. A C++ noise pipeline that listens to ideal Gazebo topics and republishes ROS 2 topics with realistic noise and covariances.

The idea is to tune your localization stack once on this noisy simulation and then move to the robot with fewer surprises.

---

## What’s in the noise pipeline

At a high level:

- per-sensor noise nodes (IMU, 2D LiDAR, 3D LiDAR, wheel odometry),
- parameters tied to datasheets where possible, and explicitly marked estimates where they are not,
- covariance matrices filled in so the EKF can trust or distrust each sensor in the right places.

Details and references live in `docs/RESEARCH.md`, but the short version is:

| Sensor              | Model idea                                      | Why it looks like this            |
|---------------------|-------------------------------------------------|-----------------------------------|
| IMU (WT901C)        | Gauss–Markov bias + white noise + quantization | matches MPU‑9250 spec + literature |
| 2D LiDAR (A2M12)    | Range‑proportional Gaussian                     | triangulation geometry            |
| 3D LiDAR (Mid‑360)  | Radial Gaussian, distance dependent             | Livox range precision curves      |
| Wheel odometry      | Slip + yaw random walk                          | Borenstein‑style differential drive model |

The exact numbers, conversions and citations están documentados en `RESEARCH.md`. El README solo te da la intuición de alto nivel.

---

## Design choices (in plain language)

- **Noise layer instead of “tuning harder”**  
  You can always try to save a bad simulation setup by over‑tuning SLAM parameters, but then that tuning doesn’t transfer. Here the goal is the opposite: make the sensor topics themselves more honest, so the same EKF/SLAM configuration works in simulation and on the robot with minimal tweaks.

- **C++ instead of Python once the prototype is clear**  
  The first version of the noise nodes was in Python. At 100 Hz+ IMU rates on a Jetson, the GIL shows up in timing plots. Rewriting the hot path in C++ keeps latency and jitter low enough to not be the bottleneck.

- **Realism vs. calibration**  
  IMU and LiDAR models follow datasheets and standard stochastic models. Odometry parameters are in a realistic range but are not the result of a full calibration campaign. The target is to reproduce typical failure modes, not to perfectly match one individual robot.

---

## Architecture and data flow

High-level data flow for each sensor:

```text
Gazebo plugin  -->  /gz/topic
                      |
                      v
ros_gz_bridge   -->  /sensor_raw
                      |
                      v
C++ noise node  -->  /sensor
```

- Gazebo publishes ideal data.
- `ros_gz_bridge` exposes it to ROS 2.
- The noise node subscribes to `*_raw`, applies the chosen model, and publishes standard `sensor_msgs` with covariances filled in.

More detail, including exact formulas and covariance entries, is in `docs/architecture.md`.

---

## Visuals and tech stack

![Sensor Noise Pipeline](docs/noisy_diagram.png)

Comparison of the Livox Mid-360 point cloud in RViz2: **Raw Simulation** (left) vs. **Realistic Noise Model** (right). As distance increases, the point cloud spreads radially in a way that looks much closer to the real sensor.

<div align="center">
  <img src="docs/livox_noisy_vs_raw.gif" width="100%" alt="Livox Noise Model Comparison">
</div>

| Component        | Version / notes                                |
|-----------------|-----------------------------------------------|
| ROS 2           | Humble                                        |
| Simulator       | Ignition Gazebo Fortress                      |
| Language        | C++17                                         |
| Tested on       | Ubuntu 22.04, Jetson Orin                     |

---

## Getting started

### Prerequisites

- ROS 2 Humble installed and sourced.
- `colcon` available.
- Ignition Gazebo Fortress working.
- A ROS 2 workspace (for example `~/ros2_ws`).

### Build

```bash
cd ~/ros2_ws/src
git clone https://github.com/Narcis-Abella/hospital-agv-sim.git

cd ..
rosdep install --from-paths src --ignore-src -r -y

colcon build --packages-select hospital-agv-sim
source install/setup.bash
```

### Run

```bash
# Hospital world + Tracer 2 + noise nodes
ros2 launch hospital-agv-sim simulation.launch.py headless:=false
```

Check the launch file for available options.

---

## Repository structure

```text
.
├── CMakeLists.txt      # Build configuration
├── package.xml         # ROS 2 package manifest
├── README.md           # Project overview and usage
├── LICENSE             # MIT license
├── docs/               # Research notes and architecture
│   ├── RESEARCH.md
│   └── architecture.md
├── launch/             # Launch files
├── src/                # C++ noise nodes
├── scripts/            # Legacy Python nodes (kept for reference)
├── urdf/               # Robot and sensor xacros
├── meshes/             # Meshes for the robot and environment
├── models/             # Gazebo models
└── worlds/             # Hospital simulation worlds
```

---

## License

MIT License. See `LICENSE` for details.

---

## Acknowledgements

- Hospital world for the original ROS 1 project by [`javicensaez/tracerSencillo`](https://github.com/javicensaez/tracerSencillo).
- ROS 2 migration, sensor stack and noise nodes by Narcis Abella.


