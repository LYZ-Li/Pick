#!/usr/bin/env python3
import copy
import math
import yaml
import numpy as np
import open3d as o3d

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

from sensor_msgs_py import point_cloud2


class TabletopPerceptionNode(Node):
    def __init__(self):
        super().__init__('tabletop_perception_node')

        self.declare_parameter('input_cloud_topic', '/camera/depth/color/points')
        self.declare_parameter('output_cloud_topic', '/tabletop/segmented_cloud')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('workspace_yaml', '')

        self.input_cloud_topic = self.get_parameter('input_cloud_topic').value
        self.output_cloud_topic = self.get_parameter('output_cloud_topic').value
        self.target_frame = self.get_parameter('target_frame').value
        workspace_yaml = self.get_parameter('workspace_yaml').value

        with open(workspace_yaml, 'r') as f:
            cfg = yaml.safe_load(f)

        self.workspace = cfg['workspace']
        self.table_plane = cfg['table_plane']

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cloud_sub = self.create_subscription(
            PointCloud2,
            self.input_cloud_topic,
            self.cloud_callback,
            1
        )

        self.cloud_pub = self.create_publisher(PointCloud2, self.output_cloud_topic, 1)

        self.latest_segmented_cloud = None
        self.latest_stamp = None

        self.get_logger().info('TabletopPerceptionNode started.')

    def cloud_callback(self, msg: PointCloud2):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                rclpy.time.Time()
            )
            cloud_in_base = do_transform_cloud(msg, tf)
        except Exception as e:
            self.get_logger().warn(f'TF transform failed: {e}')
            return

        xyz = []
        for p in point_cloud2.read_points(
            cloud_in_base,
            field_names=('x', 'y', 'z'),
            skip_nans=True
        ):
            xyz.append([p[0], p[1], p[2]])

        if len(xyz) < 100:
            return

        pts = np.asarray(xyz, dtype=np.float64)

        ws = self.workspace
        mask = (
            (pts[:, 0] >= ws['min_x']) & (pts[:, 0] <= ws['max_x']) &
            (pts[:, 1] >= ws['min_y']) & (pts[:, 1] <= ws['max_y']) &
            (pts[:, 2] >= ws['min_z']) & (pts[:, 2] <= ws['max_z'])
        )
        pts = pts[mask]
        if pts.shape[0] < 100:
            return

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)

        plane_model, inliers = pcd.segment_plane(
            distance_threshold=self.table_plane['distance_threshold'],
            ransac_n=self.table_plane['ransac_n'],
            num_iterations=self.table_plane['num_iterations']
        )

        obj_cloud = pcd.select_by_index(inliers, invert=True)
        obj_pts = np.asarray(obj_cloud.points)

        if obj_pts.shape[0] < 50:
            return

        out_msg = point_cloud2.create_cloud_xyz32(
            cloud_in_base.header,
            obj_pts.tolist()
        )
        out_msg.header.frame_id = self.target_frame

        self.latest_segmented_cloud = out_msg
        self.latest_stamp = self.get_clock().now()
        self.cloud_pub.publish(out_msg)


def main():
    rclpy.init()
    node = TabletopPerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
