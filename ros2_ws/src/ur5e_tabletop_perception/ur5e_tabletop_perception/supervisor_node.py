#!/usr/bin/env python3
"""Supervisor node: closed-loop pick-place cycle.

Subscribes to the segmented object cloud, calls GPD for grasp detection,
converts GPD grasps to TCP poses, filters and ranks them, then sends
the best grasp to the MTC execution server via the PickPlace service.
"""
import math
import yaml
from typing import List, Optional

import numpy as np
import transforms3d

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2

from gpd_ros2_interfaces.srv import DetectGrasps

from ur5e_pick_place_interfaces.srv import PickPlace


class SupervisorNode(Node):
    def __init__(self):
        super().__init__('supervisor_node')

        self.declare_parameter('workspace_yaml', '')
        self.declare_parameter('segmented_cloud_topic', '/tabletop/segmented_cloud')
        self.declare_parameter('gpd_service', '/detect_grasps')
        self.declare_parameter('mtc_execute_service', '/execute_pick_place')

        workspace_yaml = self.get_parameter('workspace_yaml').value
        with open(workspace_yaml, 'r') as f:
            self.cfg = yaml.safe_load(f)

        self.grasp_cfg = self.cfg['grasp_filter']
        self.place_cfg = self.cfg['place_pose']

        self.latest_cloud: Optional[PointCloud2] = None

        cb_group = ReentrantCallbackGroup()

        self.cloud_sub = self.create_subscription(
            PointCloud2,
            self.get_parameter('segmented_cloud_topic').value,
            self.cloud_cb,
            1,
            callback_group=cb_group
        )

        self.gpd_client = self.create_client(
            DetectGrasps,
            self.get_parameter('gpd_service').value,
            callback_group=cb_group
        )

        self.exec_client = self.create_client(
            PickPlace,
            self.get_parameter('mtc_execute_service').value,
            callback_group=cb_group
        )

        while not self.gpd_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for GPD service...')
        while not self.exec_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for MTC execute service...')

        self.timer = self.create_timer(1.0, self.loop_once, callback_group=cb_group)
        self.busy = False
        self.failure_count = 0
        self.max_consecutive_failures = 5

        self.get_logger().info('SupervisorNode started.')

    def cloud_cb(self, msg: PointCloud2):
        self.latest_cloud = msg

    def loop_once(self):
        if self.busy:
            return
        if self.latest_cloud is None:
            self.get_logger().info('No segmented cloud yet.')
            return
        if self.failure_count >= self.max_consecutive_failures:
            self.get_logger().error(
                f'{self.failure_count} consecutive failures — pausing. '
                'Reset failure_count or restart the node.')
            return

        self.busy = True
        self.get_logger().info('Starting pick-place cycle...')

        req = DetectGrasps.Request()
        req.cloud = self.latest_cloud

        future = self.gpd_client.call_async(req)
        future.add_done_callback(self.on_grasps)

    def on_grasps(self, future):
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f'GPD call failed: {e}')
            self.failure_count += 1
            self.busy = False
            return

        grasps = list(resp.grasps.grasps) if hasattr(resp.grasps, 'grasps') else []
        if not grasps:
            self.get_logger().warn('No grasps detected.')
            self.failure_count += 1
            self.busy = False
            return

        tcp_poses = [self.gpd_grasp_to_tcp_pose(g) for g in grasps]
        scored = list(zip(grasps, tcp_poses))

        valid = self.filter_grasps(scored)
        if not valid:
            self.get_logger().warn('No valid grasps after filtering.')
            self.failure_count += 1
            self.busy = False
            return

        valid.sort(key=lambda pair: -pair[0].score.data)
        best_grasp, best_pose = valid[0]
        self.get_logger().info(
            f'Selected grasp with score {best_grasp.score.data:.3f} at '
            f'[{best_pose.pose.position.x:.3f}, '
            f'{best_pose.pose.position.y:.3f}, '
            f'{best_pose.pose.position.z:.3f}]')

        req = PickPlace.Request()
        req.grasp_pose = best_pose
        req.pregrasp_approach_distance = self.grasp_cfg['pregrasp_offset']
        req.grasp_offset = self.grasp_cfg['grasp_offset']
        req.lift_distance = self.grasp_cfg['lift_offset']
        req.place_pose = self.build_place_pose()
        req.gripper_open_width = 0.085
        req.gripper_close_width = 0.0

        future = self.exec_client.call_async(req)
        future.add_done_callback(self.on_execute_done)

    def gpd_grasp_to_tcp_pose(self, grasp) -> PoseStamped:
        """Convert a GPD GraspConfig to a PoseStamped TCP goal in base frame.

        GPD defines a grasp frame with three axes:
          - approach: direction the hand moves toward the object
          - binormal: direction along the gripper closing axis
          - axis: cross product (approach x binormal)

        The rotation matrix R = [binormal | axis | approach] maps from
        the GPD grasp frame to the base frame. We convert this to a
        quaternion for the TCP pose.
        """
        # GPD grasp position (already in base frame after perception transform)
        pos = grasp.position

        # Build rotation matrix from GPD axes [binormal, axis, approach]
        approach = np.array([grasp.approach.x, grasp.approach.y, grasp.approach.z])
        binormal = np.array([grasp.binormal.x, grasp.binormal.y, grasp.binormal.z])
        axis = np.array([grasp.axis.x, grasp.axis.y, grasp.axis.z])

        # Normalize
        approach = approach / (np.linalg.norm(approach) + 1e-9)
        binormal = binormal / (np.linalg.norm(binormal) + 1e-9)
        axis = axis / (np.linalg.norm(axis) + 1e-9)

        # Rotation matrix: columns are [binormal, axis, approach]
        rot = np.column_stack([binormal, axis, approach])

        # Ensure valid rotation matrix (closest proper rotation)
        u, _, vt = np.linalg.svd(rot)
        rot = u @ vt
        if np.linalg.det(rot) < 0:
            u[:, -1] *= -1
            rot = u @ vt

        # Convert to quaternion (transforms3d uses wxyz convention)
        quat_wxyz = transforms3d.quaternions.mat2quat(rot)

        pose = PoseStamped()
        pose.header.frame_id = self.cfg['workspace']['frame_id']
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = pos.x
        pose.pose.position.y = pos.y
        pose.pose.position.z = pos.z
        pose.pose.orientation.w = quat_wxyz[0]
        pose.pose.orientation.x = quat_wxyz[1]
        pose.pose.orientation.y = quat_wxyz[2]
        pose.pose.orientation.z = quat_wxyz[3]

        return pose

    def filter_grasps(self, scored: list) -> list:
        """Filter grasps by score, orientation, and workspace bounds."""
        cfg = self.grasp_cfg
        min_score = cfg['min_score']
        min_z_dot = cfg['min_z_axis_dot_world_z']
        max_pitch = math.radians(cfg['max_tcp_pitch_deg'])
        max_roll = math.radians(cfg['max_tcp_roll_deg'])

        valid = []
        for grasp, pose in scored:
            # Score filter
            if grasp.score.data < min_score:
                continue

            # Extract approach axis from grasp (should be roughly aligned with world Z
            # for a top-down grasp)
            approach = np.array([
                grasp.approach.x, grasp.approach.y, grasp.approach.z
            ])
            approach = approach / (np.linalg.norm(approach) + 1e-9)
            world_z = np.array([0.0, 0.0, 1.0])
            z_dot = abs(np.dot(approach, world_z))

            if z_dot < min_z_dot:
                continue

            # Orientation limits (roll/pitch of TCP relative to world)
            q = pose.pose.orientation
            quat_wxyz = [q.w, q.x, q.y, q.z]
            rot = transforms3d.quaternions.quat2mat(quat_wxyz)
            # Extract Euler angles (ZYX convention -> roll, pitch, yaw)
            roll = math.atan2(rot[2, 1], rot[2, 2])
            pitch = math.asin(-np.clip(rot[2, 0], -1.0, 1.0))

            if abs(roll) > max_roll or abs(pitch) > max_pitch:
                continue

            valid.append((grasp, pose))

        return valid

    def build_place_pose(self) -> PoseStamped:
        """Build the place pose from workspace config."""
        p = self.place_cfg
        pose = PoseStamped()
        pose.header.frame_id = p['frame_id']
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = p['x']
        pose.pose.position.y = p['y']
        pose.pose.position.z = p['z']
        pose.pose.orientation.x = p['qx']
        pose.pose.orientation.y = p['qy']
        pose.pose.orientation.z = p['qz']
        pose.pose.orientation.w = p['qw']
        return pose

    def on_execute_done(self, future):
        try:
            resp = future.result()
            if resp.success:
                self.get_logger().info(f'Pick-place succeeded: {resp.message}')
                self.failure_count = 0
            else:
                self.get_logger().warn(f'Pick-place failed: {resp.message}')
                self.failure_count += 1
        except Exception as e:
            self.get_logger().error(f'Execution call failed: {e}')
            self.failure_count += 1
        self.busy = False


def main():
    rclpy.init()
    node = SupervisorNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()
