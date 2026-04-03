"""Manual Robotiq gripper command helper."""

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class GripperCommandNode(Node):
    """Publishes a single joint-position command for the Robotiq gripper."""

    def __init__(self, topic: str) -> None:
        super().__init__('gripper_command')
        self._publisher = self.create_publisher(Float64MultiArray, topic, 10)
        self._topic = topic

    def publish_position(self, position: float, wait_seconds: float) -> None:
        deadline = time.monotonic() + max(wait_seconds, 0.0)
        while time.monotonic() < deadline and self.count_subscribers(self._topic) == 0:
            rclpy.spin_once(self, timeout_sec=0.1)

        message = Float64MultiArray(data=[position])
        for _ in range(3):
            self._publisher.publish(message)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)


def _width_to_joint_position(
    width_m: float,
    max_width_m: float,
    closed_position: float,
) -> float:
    if max_width_m <= 0.0:
        raise ValueError('max_width_m must be positive')
    if width_m < 0.0 or width_m > max_width_m:
        raise ValueError(
            f'width_m must be between 0.0 and {max_width_m:.3f} meters'
        )

    closure_ratio = 1.0 - (width_m / max_width_m)
    return max(0.0, min(closed_position, closure_ratio * closed_position))


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Publish a one-shot command to the Robotiq gripper controller.'
    )
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument(
        '--width',
        type=float,
        help='Target jaw width in meters. Example: 0.020 for a slight opening.',
    )
    mode.add_argument(
        '--open',
        action='store_true',
        help='Command the gripper fully open.',
    )
    mode.add_argument(
        '--close',
        action='store_true',
        help='Command the gripper fully closed.',
    )
    mode.add_argument(
        '--joint-position',
        type=float,
        help='Send the raw Robotiq joint position directly.',
    )
    parser.add_argument(
        '--topic',
        default='/gripper_position_controller/commands',
        help='Topic used by the forward command controller.',
    )
    parser.add_argument(
        '--max-width',
        type=float,
        default=0.085,
        help='Maximum supported jaw width in meters.',
    )
    parser.add_argument(
        '--closed-position',
        type=float,
        default=0.7929,
        help='Robotiq joint value that corresponds to fully closed.',
    )
    parser.add_argument(
        '--wait-seconds',
        type=float,
        default=2.0,
        help='How long to wait for the controller subscriber before publishing.',
    )
    return parser.parse_args()


def main() -> None:
    args = _parse_args()

    if args.open:
        target_position = 0.0
    elif args.close:
        target_position = args.closed_position
    elif args.width is not None:
        target_position = _width_to_joint_position(
            width_m=args.width,
            max_width_m=args.max_width,
            closed_position=args.closed_position,
        )
    else:
        if math.isnan(args.joint_position):
            raise ValueError('joint-position must be a valid number')
        target_position = args.joint_position

    rclpy.init()
    node = GripperCommandNode(topic=args.topic)
    node.get_logger().info(
        f'Publishing Robotiq command {target_position:.4f} to {args.topic}'
    )
    node.publish_position(
        position=target_position,
        wait_seconds=args.wait_seconds,
    )
    node.destroy_node()
    rclpy.shutdown()
