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
# Authors: Ryan Shim, Gilbert

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action import GoalResponse

from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

import numpy as np

from turtlebot3_msgs.action import Turtlebot3 


class Server(Node):

    def __init__(self):
        super().__init__('turtlebot3_server')

        self._action_server = ActionServer(
            self,
            Turtlebot3,
            'turtlebot3',
            self.execute_callback)

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.cmd_vel = Twist()

        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.get_odom,
            10)
        self.odom_sub

        self.get_state = self.create_subscription(
            JointState,
            'joint_states',
            self.get_state,
            10)
        self.get_state

        self.init_stats = True

        self.get_logger().info('Server On')

    def get_odom(self, odom):
        self.position = Point()
        self.position = odom.pose.pose.position

    def get_state(self, data):
        TICK2RAD = 0.001533981
        last_pos = 0.0
        diff_pos = 0.0
        cur_pos = 0.0
        encoder = 0

        cur_pos = data.position[0]
        diff_pos = cur_pos - last_pos
        encoder = encoder + (diff_pos / TICK2RAD)
        self.right_encoder = encoder

    def turn(self, angle):
        self.twist = Twist()

        if self.init_stats:
            self.init_right_encoder = self.right_encoder
            self.init_stats = False
        diff_encoder = (np.deg2rad(angle) * 0.080) / (0.207 / 4096)
        while (abs(self.init_right_encoder - self.right_encoder) < abs(diff_encoder)):
            if diff_encoder >= 0:
                self.twist.angular.z = -0.5
            else:
                self.twist.angular.z = 0.5

            self.cmd_vel_pub.publish(self.twist)
            # self.r.sleep()
        self.init_stats = True
        self.twist.angular.z = 0
        self.cmd_vel_pub.publish(self.twist)
        # self.r.sleep()

    def go_front(self, lenght, count):
        if count == 0:
            while self.position.x  < lenght:
                self.twist.linear.x = 0.1
                self.twist.angular.z = 0.0
                self.cmd_vel_pub.publish(self.twist)
                # self.r.sleep()
        elif count == 1:
            while self.position.y < lenght:
                self.twist.linear.x = 0.1
                self.twist.angular.z = 0.0
                self.cmd_vel_pub.publish(self.twist)
                # self.r.sleep()
        elif count == 2:
            while self.position.x > lenght:
                self.twist.linear.x = 0.1
                self.twist.angular.z = 0.0
                self.cmd_vel_pub.publish(self.twist)
                # self.r.sleep()
        else:
            while self.position.y > lenght:
                self.twist.linear.x = 0.1
                self.twist.angular.z = 0.0
                self.cmd_vel_pub.publish(self.twist)
                # self.r.sleep()
        self.twist.linear.x = 0.0
        self.cmd_vel_pub.publish(self.twist)
        # self.r.sleep()

    # def execute_callback(self, goal):
    def execute_callback(self):
        position = Point()
        self.twist = Twist()
        # self.r = rospy.Rate(15)
        # self.r1 = rospy.Rate(1)
        success = True
        # mode = int(goal.goal.x)

        
        self.get_logger().info('server test')

        mode = 1
        patrol_count = 1
        # patrol_count = int(goal.goal.z)
        circle_mode = True
        half_patrol = False
        circle_count = 0

        for i in range(patrol_count):
            if mode == 1:
                area = [0,0,0,0]
                # area[0] = goal.goal.y
                # area[1] = goal.goal.y
                area[0] = 1
                area[1] = 1
                for i in range(4):
                    self.go_front(area[i], i)
                    self.r1.sleep()
                    self.turn(-90)

            elif mode == 2:
                area = [0, 0, 0]
                # area[0] = goal.goal.y
                # area[1] = goal.goal.y
                area[0] = 1
                area[1] = 1
                for i in range(3):
                    self.go_front(area[i], i)
                    self.turn(-120)
            elif mode == 3:
                while(circle_mode):
                    # if self.position.x < -goal.goal.y / 2:
                    if self.position.x < -1 / 2:
                        half_patrol = True
                        count_flag = True
                    else:
                        self.twist.linear.x = 1 / 2
                        # self.twist.linear.x = goal.goal.y / 2
                        self.twist.angular.z = 0.5
                    if half_patrol == True and self.position.x > 0:
                        if count_flag == True:
                            circle_count = circle_count + 1
                            count_flag = False
                        half_patrol = False
                        if circle_count == patrol_count:
                            circle_mode = False
                            self.twist.linear.x = 0
                            self.twist.angular.z = 0
                    self.cmd_pub.publish(self.twist)
                    self.r.sleep()

        if success:
            self._result = 0
            self.get_logger().info('Succeeded' % self.action_name)
            self._as.set_succeeded(self._result)
        

def main(args=None):
    rclpy.init(args=args)
    turtlebot3_server = Server()
    rclpy.spin(turtlebot3_server)

    turtlebot3_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


# import rclpy
# from rclpy.action import ActionServer
# from rclpy.node import Node

# from action_tutorials_interfaces.action import Fibonacci


# class Turtlebot3Server(Node):

#     def __init__(self):
#         super().__init__('turtlebot3_server')
#         self._action_server = ActionServer(
#             self,
#             Fibonacci,
#             'fibonacci',
#             self.execute_callback)

#     def execute_callback(self, goal_handle):
#         self.get_logger().info('Executing goal...')
#         result = Fibonacci.Result()
#         return result


# def main(args=None):
#     rclpy.init(args=args)

#     turtlebot3_server = Turtlebot3Server()

#     rclpy.spin(turtlebot3_server)


# if __name__ == '__main__':
#     main()