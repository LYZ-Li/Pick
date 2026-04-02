# AGENT.md

## Purpose

This repository is intended to deploy a ROS 2 Humble grasping stack for a UR5e with:

- a Robotiq 2-Finger gripper
- a RealSense depth camera d415
- tabletop perception
- GPD grasp detection
- MoveIt Task Constructor pick/place execution

The immediate goal is a Docker-based deployment that can bring up hardware first, then calibration, then perception, then grasping, then closed-loop pick/place.

## Current Repository State

The repo already contains the basic package split:

- `docker/`
- `ros2_ws/src/ur5e_pick_place_bringup`
- `ros2_ws/src/ur5e_tabletop_perception`
- `ros2_ws/src/ur5e_pick_place_mtc`

What is already present:

- Docker image based on `osrf/ros:humble-desktop`
- dependency import via `dependencies.repos`
- RealSense, easy_handeye2, MoveIt Task Constructor, and GPD dependencies listed
- a tabletop segmentation node
- a supervisor skeleton
- an MTC server skeleton
- a single bringup launch file scaffold
- early custom workcell packages for description, MoveIt launch wrapping, and gripper control

What is still placeholder or incomplete:

- UR driver launch path in `system_bringup.launch.py`
- MoveIt launch path in `system_bringup.launch.py`
- hand-eye is currently a temporary static TF
- no confirmed Robotiq driver/controller integration in repo
- MTC service currently uses `std_srvs/Trigger` instead of a grasp-aware request
- supervisor does not yet convert GPD grasps into robot TCP goals
- no confirmed calibrated camera-to-base transform pipeline
- no hardware-specific startup profile or deployment instructions yet

This means the repo is not yet ready for end-to-end execution on the real robot without additional integration work.

## Repo Map

- `docker/Dockerfile`
  Builds the ROS 2 environment and installs apt/pip dependencies.

- `docker/docker-compose.yml`
  Starts a privileged host-network container and mounts the local source tree.

- `ros2_ws/src/ur5e_pick_place_bringup/launch/system_bringup.launch.py`
  Intended top-level launch, but still contains placeholder launch file paths.

- `ros2_ws/src/ur5e_pick_place_bringup/config/workspace.yaml`
  Workspace crop, plane segmentation, grasp filter, and place pose parameters.

- `ros2_ws/src/ur5e_pick_place_bringup/config/handeye_static.yaml`
  Temporary camera extrinsics placeholder.

- `ros2_ws/src/ur5e_tabletop_perception/ur5e_tabletop_perception/tabletop_perception_node.py`
  Transforms the cloud into `base`, crops workspace, removes the table plane, publishes object cloud.

- `ros2_ws/src/ur5e_tabletop_perception/ur5e_tabletop_perception/supervisor_node.py`
  Skeleton loop for segmented cloud -> GPD -> execute pick/place.

- `ros2_ws/src/ur5e_pick_place_mtc/src/pick_place_server.cpp`
  Stub for MTC-driven pick/place execution.

## Deployment Target

The preferred operating model is:

1. Build one Docker image containing the ROS 2 stack.
2. Run it on a Linux host with:
   - host networking
   - USB access for RealSense
   - access to the UR robot network
   - display forwarding for RViz and calibration tools
3. Keep hardware bringup modular:
   - UR driver
   - gripper driver/controller
   - MoveIt
   - RealSense
   - hand-eye TF
   - perception
   - GPD
   - MTC execution
   - supervisor
4. Validate each phase independently before enabling the forever loop.

## Validation Phases

### Phase A: Hardware Only

Required outcome:

- UR driver starts successfully
- robot joint states and TF are valid
- Robotiq gripper opens/closes from ROS
- MoveIt starts and sees the robot correctly
- RealSense starts and publishes `/camera/depth/color/points`

Gate:

- Do not proceed until all hardware interfaces are stable inside Docker.

### Phase B: Calibration

Required outcome:

- ArUco or ChArUco board is mounted or printed
- `easy_handeye2` is run against the real hardware
- 15 to 25 samples are collected
- hand-eye result is solved and saved
- temporary static TF is replaced by the calibrated transform

Gate:

- Do not keep the static placeholder transform after calibration is complete.

### Phase C: Perception Verification

Required outcome:

- RealSense and calibrated hand-eye TF are active
- perception node publishes a segmented object cloud
- segmented output excludes most table points
- workspace crop and plane threshold are tuned on the real setup

Gate:

- Segmented cloud should mostly contain the target objects, not the table.

### Phase D: Grasp Verification

Required outcome:

- GPD service accepts the segmented cloud
- candidate grasps are visualized in RViz
- grasps look reasonable for a two-finger parallel gripper
- GPD and downstream filters are tuned for the hardware

Gate:

- Grasp proposals must be physically plausible for the Robotiq gripper.

### Phase E: Motion Verification

Required outcome:

- one hardcoded grasp pose can be executed cleanly
- approach, close, lift, place, and retreat succeed
- object attach/detach and gripper actuation are reliable

Gate:

- Do not connect live GPD output into execution until the hardcoded grasp path is stable.

### Phase F: Forever Loop

Required outcome:

- home or observe
- get latest segmented cloud
- detect grasps
- rank and filter
- execute pick/place
- reopen gripper if failed
- loop

Gate:

- Failure recovery and reset behavior must be defined before unattended looping.

## Expected Next Implementation Tasks

The next engineering steps in this repo should usually be:

1. Replace placeholder bringup launch references with the actual UR and MoveIt launch files used in this system.
2. Add explicit Robotiq gripper support:
   - driver/controller package selection
   - controller names
   - open/close command interface
3. Split top-level bringup into phase-friendly launch files instead of one all-in launch.
4. Replace the `Trigger`-based MTC API with a custom service carrying:
   - grasp pose
   - pregrasp offset
   - lift offset
   - place pose
   - gripper width or command parameters
5. Implement GPD grasp conversion from `GraspConfig` into TCP poses in the robot base frame.
6. Integrate `easy_handeye2` outputs into runtime TF publication.
7. Add operator documentation for calibration and verification.
8. Test the full stack on the real hardware incrementally by phase.

## Constraints And Assumptions

- This stack assumes ROS 2 Humble.
- The deployment host is Ubuntu 24 on bare metal.
- The gripper is a Robotiq 2F-85 exposed at `/dev/robotiq`.
- The current Docker setup assumes a Linux host. The compose file mounts `/dev`, `/dev/bus/usb`, and X11 sockets, which fits the chosen host setup.
- `docker-compose.yml` currently requests NVIDIA resources, but GPU may not be required for basic function. This should be confirmed against the target host.
- The real robot IP is `192.168.1.251`.
- ROS Domain ID stays at the default.
- Camera support should be configurable between:
  - a fixed eye-to-hand D415
  - a wrist-mounted D415
- Initial work should target the fixed workspace camera first.
- `system_bringup.launch.py` must keep robot IP, camera selection, and placeholder camera TF configurable.

## Recommended Upstream Stack

For the UR arm and baseline MoveIt integration, prefer the official Universal Robots ROS 2 packages:

- `ur_robot_driver`
- `ur_moveit_config`
- `ur_calibration`

For the arm description and workcell integration approach, follow the official Universal Robots custom workcell pattern and create local packages in this repo for:

- a custom workcell description
- a custom MoveIt config that includes the gripper and camera frames

For the Robotiq gripper, the preferred starting point is the maintained ROS 2 package set from PickNik:

- `ros2_robotiq_gripper`

Reason:

- the Universal Robots driver and MoveIt config are the canonical, stable upstream for UR5e on ROS 2 Humble
- the official `ur_moveit_config` only models the arm, so a custom workcell package is still required for a real pick cell
- PickNik's Robotiq ROS 2 repository is maintained for Humble and is a stronger starting point than ad hoc single-user drivers

For MoveIt Task Constructor:

- MTC is not bundled inside the base `ros-humble-moveit` metapackage
- MTC is a separate package set on Humble
- prefer binary installation for MTC where available instead of compiling it from source

## Working Rule For Future Changes

When modifying this repo, prefer changes that make each phase independently launchable and testable. Do not wire the forever loop first. Stabilize the stack in the order: hardware, calibration, perception, grasp generation, motion execution, supervisor loop.
