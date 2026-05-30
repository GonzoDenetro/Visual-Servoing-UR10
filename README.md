# UR10 Position-Based Visual Servo Control (PBVS) — ROS2

<p align="center">
  <img src="docs/rqt_graph.png" alt="ROS2 Node Graph" width="750"/>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/ROS2-Humble-blue?logo=ros" alt="ROS2 Humble"/>
  <img src="https://img.shields.io/badge/Python-3.10-yellow?logo=python" alt="Python"/>
  <img src="https://img.shields.io/badge/OpenCV-4.x-green?logo=opencv" alt="OpenCV"/>
  <img src="https://img.shields.io/badge/Gazebo-Classic-orange" alt="Gazebo"/>
  <img src="https://img.shields.io/badge/Robot-UR10-lightgrey" alt="UR10"/>
</p>

---

## 📋 Table of Contents

- [Overview](#-overview)
- [Theory — Position-Based Visual Servoing](#-theory--position-based-visual-servoing)
- [System Architecture](#-system-architecture)
- [Package Structure](#-package-structure)
- [Installation](#-installation)
- [Roadmap](#-roadmap)
- [References](#-references)

---

## Overview

This repository implements a **Position-Based Visual Servo (PBVS)** control system for a **Universal Robots UR10** 6-DOF manipulator using **ROS2 Humble**. The system uses a wrist-mounted camera (**eye-in-hand** configuration) to detect an **ArUco marker** and autonomously drive the robot to a desired pose relative to the marker.

The pipeline is organized into five independent ROS2 nodes covering forward kinematics, geometric Jacobian computation, ArUco-based pose estimation, PBVS control law, and hardware bridging. The system has been developed and validated in simulation using **Gazebo Classic** and is designed for direct deployment on the physical UR10 robot.

**Key features:**
- Full PBVS control loop implemented from mathematical first principles
- Modular ROS2 architecture — each node is independently testable
- ArUco marker detection using OpenCV 4.x
- Geometric Jacobian computed analytically from DH parameters

---

## Theory — Position-Based Visual Servoing

### What is Visual Servoing?

Visual servo control uses computer vision data in the feedback loop to control robot motion. The goal is to minimize an error `e(t)` between observed visual features `s` and desired features `s*`:

```
e(t) = s(m(t), a) - s*
```

where `m(t)` is the set of image measurements and `a` represents additional system parameters (camera intrinsics, 3D model of the object).

### PBVS Control Law

This implementation uses the **second PBVS scheme** from Chaumette & Hutchinson (2006), which defines the visual feature vector as:

```
s = (c*_tc, θu)
```

where:
- `c*_tc ∈ ℝ³` — position of the current camera frame expressed in the desired camera frame
- `θu ∈ ℝ³` — axis-angle representation of the relative rotation `R = Rc*c`
- `s* = 0` — desired configuration (error = 0 when robot reaches goal)

#### Interaction Matrix

The interaction matrix for this formulation is block-diagonal:

```
Le = [ R    0   ]
     [ 0   Lθu  ]
```

where `Lθu` is the Jacobian of the axis-angle mapping. Exploiting the properties `R⁻¹ = Rᵀ` and `Lθu⁻¹ · θu = θu`, the control law simplifies to:

```
vc  = -λ · R · c*_tc     (linear camera velocity)
ωc  = -λ · θu            (angular camera velocity)
```

where `λ > 0` is the control gain.

#### Error Geometry

The transformation error `T_{c*←c}` is computed as:

```
T_{c*←c} = T_desired ⊗ inv(T_current)
```

This is the SE(3) equivalent of the algebraic difference `e = s* - s`. When the robot reaches the desired pose:

```
T_current = T_desired  →  T_{c*←c} = I₄  →  e = 0  →  q̇ = 0
```

#### Joint Velocity Mapping

The camera spatial velocity `vc_spatial = [vc; ωc] ∈ ℝ⁶` is mapped to joint velocities through the pseudo-inverse of the geometric Jacobian:

```
q̇ = J⁺(q) · vc_spatial
```

#### Control Pipeline Summary

```
ArUco Detection (solvePnP)
        ↓
T_current = T_{c←marker}
        ↓
T_error = T_desired ⊗ inv(T_current)
        ↓
Extract: c*_tc = T_error[:3, 3]
         θu   = axis_angle(T_error[:3, :3])
        ↓
vc  = -λ · R · c*_tc
ωc  = -λ · θu
        ↓
Saturate vc, ωc
        ↓
q̇ = J⁺(q) · [vc; ωc]
        ↓
Saturate q̇
        ↓
Publish to /forward_velocity_controller/commands
```

---

## System Architecture


### Node Summary

| Node | Package | Subscribes | Publishes |
|---|---|---|---|
| `forward_kinematics_node` | `cinematica` | `/joint_states` | `/ur10/transforms`, `/ur10/end_effector_pose` |
| `jacobian_node` | `cinematica` | `/ur10/transforms` | `/ur10/jacobian` |
| `aruco_pose_estimation_node` | `percepcion` | `/wrist_camera/image_raw`, `/wrist_camera/camera_info` | `/ur10/aruco_pose`, `/ur10/aruco_detected`, `/ur10/aruco_debug_image` |
| `control_law` | `control` | `/ur10/aruco_pose`, `/ur10/aruco_detected`, `/ur10/jacobian` | `/ur10/joint_velocities` |
| `velocity_bridge_node` | `ur10_pbvs_sim` | `/ur10/joint_velocities` | `/forward_velocity_controller/commands` |

---

## Package Structure

```
ros2_ws/src/
│
├── cinematica/                         # Forward kinematics & Jacobian
│   ├── cinematica/
│   │   ├── forward_kinematics_node.py  # DH-based FK, publishes transforms
│   │   └── jacobian_node.py            # Geometric Jacobian computation
│   ├── package.xml
│   └── setup.py
│
├── percepcion/                         # Visual perception
│   ├── percepcion/
│   │   └── aruco_pose_estimation_node.py  # ArUco detection + solvePnP
│   ├── package.xml
│   └── setup.py
│
├── control/                            # PBVS control law
│   ├── control/
│   │   └── pbvs_control_node.py        # PBVS control law implementation
│   ├── package.xml
│   └── setup.py
│
└── ur10_pbvs_sim/                      # Simulation package
    ├── urdf/
    │   └── ur10_camera.urdf.xacro      # UR10 + wrist camera
    ├── config/
    │   ├── ur10_controllers.yaml       # ros2_control configuration
    │   └── initial_positions.yaml      # Robot initial joint positions
    ├── launch/
    │   ├── spawn_ur.launch.py          # Gazebo + UR10 + controllers
    │   └── pbvs_sim.launch.py          # Full pipeline launch
    ├── worlds/
    │   └── pbvs_world.world            # Gazebo world with ArUco target
    ├── ur10_pbvs_sim/
    │   └── velocity_bridge_node.py     # Velocity topic bridge
    ├── package.xml
    └── setup.py
```

---



## External Repositories
These external repositories where used for simulation of the UR10

```bash
# Universal Robots URDF description
git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git

# Universal Robots Gazebo simulation
git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation.git
```


## Roadmap

- [x] Forward kinematics node (DH parameters)
- [x] Geometric Jacobian node
- [x] ArUco pose estimation node (solvePnP)
- [x] PBVS control law node
- [x] Velocity bridge node
- [x] Gazebo simulation environment
- [x] Wrist camera URDF + Gazebo plugin
- [x] ArUco marker Gazebo world model
- [ ] Full closed-loop Gazebo simulation validation
- [ ] Convergence metrics and trajectory plots
- [ ] Camera intrinsic calibration (usb_cam + camera_calibration)
- [ ] Eye-in-hand extrinsic calibration (easy_handeye)
- [ ] Physical UR10 deployment (ur_ros2_driver)
- [ ] IBVS implementation for comparison

---

## 📚 References

1. F. Chaumette and S. Hutchinson, "Visual servo control, Part I: Basic approaches," *IEEE Robotics & Automation Magazine*, vol. 13, no. 4, pp. 82–90, Dec. 2006.
2. F. Chaumette and S. Hutchinson, "Visual servo control, Part II: Advanced approaches," *IEEE Robotics & Automation Magazine*, vol. 14, no. 1, pp. 109–118, Mar. 2007.
3. S. Hutchinson, G. Hager, and P. Corke, "A tutorial on visual servo control," *IEEE Trans. Robot. Automat.*, vol. 12, no. 5, pp. 651–670, Oct. 1996.
4. J. Wu et al., "A survey of learning-based control of robotic visual servoing systems," *Journal of the Franklin Institute*, vol. 359, no. 1, pp. 556–577, 2022.
5. Universal Robots, ROS2 Driver. [Online]. Available: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver
6. Open Robotics, ROS2 Humble Documentation. [Online]. Available: https://docs.ros.org/en/humble/

---

<p align="center">
  Developed as part of a visual servoing research project — ROS2 Humble · Gazebo Classic · UR10
</p>
