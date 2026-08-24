#!/usr/bin/env python3
# Copyright (c) 2026
# SPDX-License-Identifier: MIT
"""Bridge /move_base (mbf_msgs/action/MoveBase) -> /navigate_to_pose (nav2_msgs/action/NavigateToPose).

Lets the ROS2 port of the ROS1 frontier_exploration package (which sends
mbf_msgs/action/MoveBase goals to /move_base) drive a robot through the Nav2
navigation stack. Goals, cancel requests and results are translated between the
two action interfaces.
"""

import threading

import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.action.server import CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from mbf_msgs.action import MoveBase
from nav2_msgs.action import NavigateToPose

# mbf_msgs/action/MoveBase result outcome codes (from the .action definition)
MBF_SUCCESS = 0
MBF_FAILURE = 10
MBF_CANCELED = 11
MBF_TF_ERROR = 16
MBF_INTERNAL_ERROR = 17


class MbfNav2Bridge(Node):
    """Expose mbf_msgs/action/MoveBase on /move_base, forwarding to Nav2."""

    def __init__(self):
        super().__init__('mbf_nav2_bridge')
        self._nav2_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._mbf_server = ActionServer(
            self,
            MoveBase,
            'move_base',
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )
        self.get_logger().info(
            'ready: /move_base (mbf_msgs/action/MoveBase) -> /navigate_to_pose '
            '(nav2_msgs/action/NavigateToPose)'
        )

    def _goal_callback(self, goal_request):
        p = goal_request.target_pose.pose.position
        self.get_logger().info(
            f'goal request: ({p.x:.2f}, {p.y:.2f}) frame={goal_request.target_pose.header.frame_id}'
        )
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().info('cancel request received')
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle):
        if not self._nav2_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Nav2 /navigate_to_pose action server is not available')
            goal_handle.abort()
            return MoveBase.Result(outcome=MBF_INTERNAL_ERROR,
                                   message='Nav2 navigate_to_pose server unavailable')

        nav2_goal = NavigateToPose.Goal()
        nav2_goal.pose = goal_handle.request.target_pose

        done = threading.Event()
        state = {'handle': None, 'response': None}

        def on_send(future):
            handle = future.result()
            state['handle'] = handle
            if handle is None:
                self.get_logger().error('Nav2 returned no goal handle')
                done.set()
                return
            if not handle.accepted:
                self.get_logger().error('Nav2 rejected the goal')
                done.set()
                return
            self.get_logger().info('Nav2 accepted the goal')
            get_result = handle.get_result_async()
            get_result.add_done_callback(on_result)

        def on_result(future):
            state['response'] = future.result()
            done.set()

        send_future = self._nav2_client.send_goal_async(nav2_goal)
        send_future.add_done_callback(on_send)

        # Wait for the Nav2 goal to finish, honoring MBF cancel requests.
        while rclpy.ok() and not done.wait(timeout=0.05):
            if goal_handle.is_cancel_requested:
                self.get_logger().info('cancel requested -> canceling Nav2 goal')
                handle = state['handle']
                if handle is not None:
                    handle.cancel_goal_async()
                done.wait(timeout=10.0)
                goal_handle.canceled()
                return MoveBase.Result(outcome=MBF_CANCELED, message='Goal canceled by client')

        if not rclpy.ok():
            goal_handle.abort()
            return MoveBase.Result(outcome=MBF_INTERNAL_ERROR, message='Node shutting down')

        response = state['response']
        if response is None:
            goal_handle.abort()
            return MoveBase.Result(outcome=MBF_INTERNAL_ERROR, message='No response from Nav2')

        status = response.status
        nav2_result = response.result
        result = MoveBase.Result()

        if status == GoalStatus.STATUS_SUCCEEDED:
            result.outcome = MBF_SUCCESS
            result.message = 'Goal reached'
        elif status == GoalStatus.STATUS_CANCELED:
            result.outcome = MBF_CANCELED
            result.message = 'Goal canceled'
        elif status == GoalStatus.STATUS_ABORTED:
            result.outcome = MBF_FAILURE
            result.message = nav2_result.error_msg if nav2_result else 'Nav2 aborted'
        else:
            result.outcome = MBF_FAILURE
            result.message = 'Nav2 returned status {}'.format(status)

        goal_handle.succeed()
        self.get_logger().info(f'goal finished: outcome={result.outcome} ({result.message})')
        return result


def main(args=None):
    rclpy.init(args=args)
    node = MbfNav2Bridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
