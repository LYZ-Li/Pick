#!/usr/bin/env python3
from functools import partial

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from control_msgs.action import GripperCommand
from std_srvs.srv import Trigger


class GripperCommandServer(Node):
    def __init__(self):
        super().__init__('gripper_command_server')

        self.declare_parameter(
            'gripper_action_name',
            '/gripper/robotiq_gripper_controller/gripper_cmd',
        )
        self.declare_parameter('open_position', 0.0)
        self.declare_parameter('closed_position', 0.8)
        self.declare_parameter('max_effort', 50.0)

        action_name = self.get_parameter('gripper_action_name').value
        self._client = ActionClient(self, GripperCommand, action_name)

        self.create_service(Trigger, '/gripper/open', self._open_cb)
        self.create_service(Trigger, '/gripper/close', self._close_cb)

    def _open_cb(self, _, response):
        self._send_goal(self.get_parameter('open_position').value, response)
        return response

    def _close_cb(self, _, response):
        self._send_goal(self.get_parameter('closed_position').value, response)
        return response

    def _send_goal(self, position, response):
        if not self._client.wait_for_server(timeout_sec=2.0):
            response.success = False
            response.message = 'Gripper action server not available'
            return

        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(self.get_parameter('max_effort').value)

        future = self._client.send_goal_async(goal)
        future.add_done_callback(partial(self._on_goal_response, response=response))

        response.success = True
        response.message = f'Gripper command sent to position {position:.3f}'

    def _on_goal_response(self, future, response):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error('Gripper goal rejected')
                response.success = False
                response.message = 'Gripper goal rejected'
                return
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._on_result)
        except Exception as exc:
            self.get_logger().error(f'Failed to send gripper goal: {exc}')

    def _on_result(self, future):
        try:
            result = future.result().result
            self.get_logger().info(
                f'Gripper result: position={result.position:.3f} '
                f'reached={result.reached_goal} stalled={result.stalled}'
            )
        except Exception as exc:
            self.get_logger().error(f'Gripper result failed: {exc}')


def main():
    rclpy.init()
    node = GripperCommandServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
