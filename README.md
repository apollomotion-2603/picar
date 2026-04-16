# RC Car Autonomous Driving (1/16 Scale)

![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange)
![acados](https://img.shields.io/badge/acados-v0.3.5-brightgreen)
![Status](https://img.shields.io/badge/Status-Active-success)

## 📌 Project Overview
An autonomous 1/16 scale RC car project developed as a university thesis, with the goal of creating a robust educational and commercial platform for universities and researchers. The system leverages **ROS2 Jazzy**, **Gazebo Harmonic**, and an **acados NMPC** (Nonlinear Model Predictive Control) for a complete autonomous driving pipeline.

**Core Pipeline:**  
`Camera Image` → `Lane Detection (BEV)` → `EKF Sensor Fusion` → `NMPC Optimization` → `Vehicle Control`

### Hardware Architecture
- **Vehicle Base:** 1/16 Scale RC Car (2.5 kg, 0.22m wheelbase)
- **Onboard Compute:** Raspberry Pi 5, communicating with a Base Station Laptop
- **Sensors:** Pi Camera 3 Wide (IMX708), BNO055 IMU, Wheel Encoders
- **Microcontroller:** Nucleo F411RE for servo (steering) and ESC (drive motor) PWM control

### Software Stack
- **OS:** Ubuntu 24.04 (Laptop), RPi OS 64-bit (Raspberry Pi 5)
- **Framework:** ROS2 Jazzy (native on laptop, Docker on Pi)
- **Simulation:** Gazebo Harmonic v8.10
- **Optimization:** acados v0.3.5 (RTI + HPIPM QP Solver for fast 1–5ms execution)

---

## 🏗️ Sub-Systems & Packages

The workspace is organized into following primary ROS2 packages under `/src`:

| Package | Description | Key Modules/Nodes |
|---------|-------------|-----------|
| `perception_pkg` | Translates raw camera perspectives to BEV, runs adaptive thresholding, and fits cubic arc-length polynomials. Publishes `/perception/lane_state`. | `perception_node`, `visualizer_node` |
| `mpc_pkg` | Calculates optimized steering and velocity paths assessing lateral offset, heading error, and velocity bounds constraints via NMPC. | `mpc_node` |
| `ekf_pkg` | Fuses IMU + wheel speeds + camera observations into continuous `[n, alpha, v]` states. Handles fail-safe auto-resets. | `ekf_node`, `reset_node` |
| `lane_msgs` | Custom ROS2 interfaces spanning message passing across perception and state telemetry tracking. | `LaneState.msg`, `VehicleState.msg` |
| `gazebo_ackermann...` | Dedicated Gazebo setups hosting various SDF maps, meshes, Xacro configs, and bridge settings. | `vehicle_controller` |

---

## 🚀 Getting Started

### Prerequisites
Make sure to have **ROS2 Jazzy** configured safely, **Gazebo Harmonic** installed, and the [**acados**](https://github.com/acados/acados) solver library natively built tracking `~/acados`.

### Build Workspace
```bash
# Navigate to the workspace
cd ~/main/1_projects/1_autonomous_car_research/ros2_ws

# Optional: Clean builds if any CMakeCache collision occurs
rm -rf build/ install/

# Compile all packages using colcon (symlink install to update Python nodes faster)
colcon build --symlink-install

# Source your workspace
source install/setup.bash
```

### Launch the Simulation
The simulation integrates Gazebo, physics controllers, perception pipelines, MPC, and visualization seamlessly into a single launch command. Note: `ROS_DOMAIN_ID=42` is standard across this environment.

```bash
export ROS_DOMAIN_ID=42
source install/setup.bash

# Option A: Map 1 (Oval Track)
ros2 launch gazebo_ackermann_steering_vehicle sim_full_launch.py

# Option B: Map 2 (Complex Track: Chicane & S-curve)
ros2 launch gazebo_ackermann_steering_vehicle sim_full_launch.py map:=2

# Option C: Map 3 (Double Lane Change)
ros2 launch gazebo_ackermann_steering_vehicle sim_full_launch.py map:=3
```

### Run-Time Control & Telemetry
A multi-pane visualizer and debugging metrics will spawn automatically alongside the environment. Additionally, you manually issue manual keyboard inputs targeting the `reset_node` standard TTY input:
- `s` + `<Enter>` : START / Start rolling
- `x` + `<Enter>` : STOP
- `q` + `<Enter>` : RESET to spawn origin

---

## 📅 Roadmap Pipeline

| Phase | Milestone Description | Status |
|-------|-----------------------|--------|
| **1** | **Simulation core.** Establish Perception, NMPC, EKF in Gazebo with closed cycles. | ✅ **DONE** |
| **1.5a** | Expended simulated tracks, verifying Chicane/S-Curve, complex changes. | ✅ **DONE** |
| **1.5b** | **Hardware Calibration.** Tune adaptive lane detection over physical environments and establish RC 0.3 m/s tracking integration. | 🔄 **IN PROGRESS** |
| **2** | **SLAM.** Construct Visual Odometry, map-state particle filtering and custom Lane Graph modeling. | 📅 *PLANNED* |
| **3** | **Navigation.** Bring Nav2 onto the vehicle map stack alongside object avoidance nodes. | 📅 *PLANNED* |
| **4**| **Control Center GUI.** Form a high-level UI component via Foxglove Dashboard removing ROS2 terminal needs. | 📅 *PLANNED* |

---

## 👥 Contributors & Responsibilities

- **Phi (apollomotion)** - Architecture formulation, SLAM/State-Estimation, App Navigation Pipelines, Simulation configs, Master Thesis.
- **Nhã** - MPC mathematical integration components, Hardware-Level Firmware and constraints structuring.
