#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Jeonggeun Lim, Gilbert

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from turtlebot3_msgs.action import Turtlebot3 


class Turtlebot3Client(Node):

    def __init__(self):
        super().__init__('turtlebot3_client')
        self._action_client = ActionClient(self, Turtlebot3, 'turtlebot3')

        self.mode = 1.0
        self.area = 1.0
        self.count = 1

        self.mode, self.area, self.count = self.get_key()
        self.send_goal()

    def get_key(self):
        mode = str(input("mode: "))
        area = float(input("area: "))
        count = int(input("count: "))

        if mode == 's':
            mode = 1
        elif mode == 't':
            mode = 2
        elif mode == 'x':
            self.shutdown()
        else:
            self.get_logger().info("you selected wrong mode") 
            self.shutdown()

        return mode, area, count

    def send_goal(self):
        goal_msg = Turtlebot3.Goal()
        goal_msg.goal.x = float(self.mode)
        goal_msg.goal.y = float(self.area)
        goal_msg.goal.z = float(self.count)

        self._action_client.wait_for_server()

        self._send_goal_future = \
            self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.result))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.state))


def main(args=None):
    rclpy.init(args=args)

    action_client = Turtlebot3Client()

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()