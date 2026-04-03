#!/usr/bin/env python3
import rclpy
from aruco_msgs.msg import MarkerArray
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class ArucoMarkerTfBridge(Node):
    def __init__(self):
        super().__init__('aruco_marker_tf_bridge')

        self.declare_parameter('markers_topic', '/aruco_tracker/markers')
        self.declare_parameter('tracked_marker_id', 4)
        self.declare_parameter('output_frame', 'marker_frame')

        markers_topic = self.get_parameter('markers_topic').value
        self.tracked_marker_id = int(self.get_parameter('tracked_marker_id').value)
        self.output_frame = self.get_parameter('output_frame').value

        self.tf_broadcaster = TransformBroadcaster(self)
        self.last_missing_warn_ns = 0

        self.create_subscription(
            MarkerArray,
            markers_topic,
            self.marker_callback,
            10,
        )

    def marker_callback(self, msg: MarkerArray):
        for marker in msg.markers:
            if marker.id != self.tracked_marker_id:
                continue

            transform = TransformStamped()
            transform.header.stamp = marker.header.stamp
            transform.header.frame_id = marker.header.frame_id
            transform.child_frame_id = self.output_frame
            transform.transform.translation.x = marker.pose.pose.position.x
            transform.transform.translation.y = marker.pose.pose.position.y
            transform.transform.translation.z = marker.pose.pose.position.z
            transform.transform.rotation = marker.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)
            return

        now_ns = self.get_clock().now().nanoseconds
        if abs(now_ns - self.last_missing_warn_ns) >= 5_000_000_000:
            self.get_logger().warn(
                'Tracked marker id '
                f'{self.tracked_marker_id} '
                'was not present in the latest detection message.'
            )
            self.last_missing_warn_ns = now_ns


def main():
    rclpy.init()
    node = ArucoMarkerTfBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
