# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Purpose

This repository is intended to deploy a ROS 2 Humble grasping stack for a UR5e with:

- a Robotiq 2-Finger gripper 2F-85
- a RealSense depth camera D415
- tabletop perception
- GPD grasp detection
- MoveIt Task Constructor pick/place execution

The immediate goal is a Docker-based deployment that can bring up hardware first, then calibration, then perception, then grasping, then closed-loop pick/place.

## Development Commands

**DO NOT run `colcon build`, `colcon test`, or any `ros2` command on the host machine.** The ROS 2 environment is only available inside the Docker container. All build, test, and run commands must be executed within the container.

All development happens inside the Docker container. Start the container first, then run commands inside it.

```bash
# Build and start the Docker container
./run.sh

# Inside the container: rebuild the ROS 2 workspace
colcon build --symlink-install

# Run all tests (linting: flake8, pep257, ament_copyright)
colcon test
colcon test-result --verbose   # view results

# Launch by phase (recommended for incremental validation)
ros2 launch ur5e_pick_place_bringup phase_a_hardware.launch.py
ros2 launch ur5e_pick_place_bringup phase_b_calibration.launch.py
ros2 launch ur5e_pick_place_bringup phase_c_perception.launch.py
ros2 launch ur5e_pick_place_bringup phase_d_grasp.launch.py
ros2 launch ur5e_pick_place_bringup phase_e_motion.launch.py
ros2 launch ur5e_pick_place_bringup phase_f_loop.launch.py

# Launch the full system (all phases at once)
ros2 launch ur5e_pick_place_bringup system_bringup.launch.py

# Use fake hardware for testing without a physical robot
ros2 launch ur5e_pick_place_bringup phase_a_hardware.launch.py use_fake_hardware:=true

# Use wrist-mounted camera instead of fixed
ros2 launch ur5e_pick_place_bringup phase_a_hardware.launch.py camera_mount:=wrist

# Run individual nodes
ros2 run ur5e_tabletop_perception tabletop_perception_node
ros2 run ur5e_tabletop_perception supervisor_node
ros2 run ur5e_pick_place_mtc pick_place_server

# Visualize the workcell URDF
ros2 launch ur5e_workcell_description view_workcell.launch.py
```

## Packages

- `ur5e_pick_place_bringup` — Launch files (per-phase and full system) and configuration YAML files.
- `ur5e_tabletop_perception` — Python nodes for point cloud segmentation and the supervisor control loop.
- `ur5e_pick_place_mtc` — C++ MTC pick/place execution server.
- `ur5e_pick_place_interfaces` — Custom ROS 2 service definition (`PickPlace.srv`) for grasp-aware MTC requests.
- `ur5e_workcell_description` — URDF/xacro combining UR5e arm, Robotiq 2F-85 gripper, and RealSense D415 camera.

## Key Service Interface

The `PickPlace.srv` service (`ur5e_pick_place_interfaces`) carries:
- `grasp_pose` (PoseStamped) — TCP pose at the grasp location
- `pregrasp_approach_distance` — distance to approach before grasping
- `grasp_offset` — additional offset along approach direction
- `lift_distance` — distance to lift after grasping
- `place_pose` (PoseStamped) — target place pose
- `gripper_open_width` / `gripper_close_width` — gripper widths in meters

## Data Flow

```
RealSense D415
  → /camera/depth/color/points
TabletopPerceptionNode (transform, crop, RANSAC table removal)
  → /tabletop/segmented_cloud
SupervisorNode
  → /detect_grasps (GPD service call)
  → GPD grasp → TCP pose conversion + filtering/ranking
  → /execute_pick_place (PickPlace service call)
MTC PickPlaceServer (approach, grasp, lift, place, retreat)
  → MoveIt execution on UR5e + Robotiq gripper
```

## What Still Requires Hardware Validation

- Hand-eye calibration (currently a static TF placeholder; run Phase B to calibrate)
- Robotiq gripper controller tuning (joint limits, open/close widths)
- Workspace bounds and RANSAC thresholds (tune in `workspace.yaml` on real setup)
- GPD grasp quality thresholds (tune in `workspace.yaml` `grasp_filter` section)
- MTC planner timeouts and velocity scaling for the real robot

## Constraints And Assumptions

- ROS 2 Humble in docker on Ubuntu 24 bare metal host.
- Robotiq 2F-85 exposed at `/dev/robotiq`.
- Docker container with host networking, X11 forwarding, GPU access, and USB passthrough.
- Real robot IP: `192.168.1.251`.
- ROS Domain ID: default.
- Camera: supports both fixed eye-to-hand D415 (default) and wrist-mounted eye-in-hand D415 via `camera_mount:=wrist` launch argument.

## Working Rule For Future Changes

When modifying this repo, prefer changes that make each phase independently launchable and testable. Do not wire the forever loop first. Stabilize the stack in the order: hardware, calibration, perception, grasp generation, motion execution, supervisor loop.
