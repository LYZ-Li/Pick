#!/usr/bin/env python3
import math
import time
import yaml
from typing import List, Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger

from gpd_ros2_interfaces.srv import DetectGrasps
from gpd_ros2_interfaces.msg import GraspConfigList

# Replace with your own service once created in ur5e_pick_place_mtc
from example_interfaces.srv import Trigger as ExecTrigger


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

        self.latest_cloud: Optional[PointCloud2] = None

        self.cloud_sub = self.create_subscription(
            PointCloud2,
            self.get_parameter('segmented_cloud_topic').value,
            self.cloud_cb,
            1
        )

        self.gpd_client = self.create_client(
            DetectGrasps,
            self.get_parameter('gpd_service').value
        )

        self.exec_client = self.create_client(
            ExecTrigger,
            self.get_parameter('mtc_execute_service').value
        )

        while not self.gpd_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for GPD service...')
        while not self.exec_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for MTC execute service...')

        self.timer = self.create_timer(1.0, self.loop_once)
        self.busy = False
        self.failure_count = 0

    def cloud_cb(self, msg: PointCloud2):
        self.latest_cloud = msg

    def loop_once(self):
        if self.busy:
            return
        if self.latest_cloud is None:
            self.get_logger().info('No segmented cloud yet.')
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
            self.busy = False
            return

        grasps = list(resp.grasps.grasps) if hasattr(resp.grasps, 'grasps') else []
        if not grasps:
            self.get_logger().warn('No grasps found.')
            self.busy = False
            return

        best = self.select_best_grasp(grasps)
        if best is None:
            self.get_logger().warn('No valid grasp after filtering.')
            self.busy = False
            return

        # Real implementation:
        # send best grasp + place pose to custom MTC service
        req = ExecTrigger.Request()
        future = self.exec_client.call_async(req)
        future.add_done_callback(self.on_execute_done)

    def select_best_grasp(self, grasps):
        # Placeholder ranking logic.
        # In a real build, convert GPD grasp to TCP pose and rank by:
        # - score
        # - top-down preference
        # - distance from table
        # - reachability / IK pre-check
        return grasps[0] if grasps else None

    def on_execute_done(self, future):
        try:
            resp = future.result()
            self.get_logger().info(f'Pick-place execution result: {resp.success}')
        except Exception as e:
            self.get_logger().error(f'Execution failed: {e}')
        self.busy = False


def main():
    rclpy.init()
    node = SupervisorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()