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

import sys
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from turtlebot3_msgs.action import Turtlebot3


class Client(Node):
    def __init__(self):
        super().__init__('turtlebot3_client')
        self.get_logger().info('Wait for server')
        self.client()

    def getkey(self):
        # mode = str(input("mode: "))
        # area = str(input("area: "))
        # count = str(input("count: "))
        
        mode = float(input("mode: "))
        area = float(input("area: "))
        count = float(input("count: "))
        # if mode == 's':
        #     mode = 1
        # elif mode == 't':
        #     mode = 2
        # elif mode == 'c':
        #     mode = 3
        # elif mode == 'x':
        #     self.shutdown()
        # else:
        #     self.get_logger().info('you select wrong mode...')

        return mode, area, count

    def client(self):
        action_client = ActionClient(self, Turtlebot3, 'turtlebot3')

        mode, area, count = self.getkey()


        self.get_logger().info('test')
        action_client.wait_for_server()
        goal = Turtlebot3.Goal() 
        goal.goal.x = mode
        goal.goal.y = area
        goal.goal.z = count

        future = action_client.send_goal(goal)

        self.get_logger().info('send to goal')

        return future


def main(args=None):
    rclpy.init(args=args)
    turtlebot3_client = Client()
    rclpy.spin(turtlebot3_client)


if __name__ == '__main__':
    main()
