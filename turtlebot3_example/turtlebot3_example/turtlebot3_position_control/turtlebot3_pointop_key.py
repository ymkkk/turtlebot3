
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
from rclpy.node import Node
import numpy
import math

from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry


class Turtlebot3PointOpKey(Node):

    def __init__(self):
        super().__init__('turtlebot3_pointop_key')
        
        self.goal_position = Point()
        self.goal_heading = 0.0
        self.position = Point()
        self.heading = 0.0 
        self.position_error = Point()
        self.heading_error = 0.0 

        self.angular_speed = 0.3
        self.linear_speed = 0.5

        self.get_key()

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.cmd_vel = Twist()

        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.get_odom,
            10)
        self.odom_sub

        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # self.get_logger().info('test')    

        self.position_error.x = self.goal_position.x - self.position.x
        self.position_error.y = self.goal_position.y - self.position.y

        distance = math.sqrt(pow(self.position_error.x, 2) + pow(self.position_error.y, 2))
        goal_direction = math.atan2(self.position_error.y, self.position_error.x)

        if distance > 0.1:
            path_angle = goal_direction - self.heading

            if path_angle < -math.pi:
                path_angle = path_angle + 2 * math.pi
            elif path_angle > math.pi:
                path_angle = path_angle - 2 * math.pi

            self.cmd_vel.angular.z = path_angle
            self.cmd_vel.linear.x = min(self.linear_speed * distance, 0.1)

            if self.cmd_vel.angular.z > 0:
                self.cmd_vel.angular.z = min(self.cmd_vel.angular.z, 1.5)
            else:
                self.cmd_vel.angular.z = max(self.cmd_vel.angular.z,  -1.5)
            self.cmd_vel_pub.publish(self.cmd_vel)
            
        else:
            self.heading_error = self.goal_heading - self.heading

            if self.heading_error < -math.pi:
                self.heading_error = self.heading_error+ 2 * math.pi
            elif self.heading_error > math.pi:
                self.heading_error = self.heading_error- 2 * math.pi

            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = self.heading_error

            if abs(self.heading_error * 180.0 / math.pi) < 1.0:
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = 0.0
                
        self.get_logger().info("distance: " + str(distance))
        self.get_logger().info("heading_angle: " + str(self.heading_error * 180.0 / math.pi))
        self.cmd_vel_pub.publish(self.cmd_vel)

    def get_odom(self, msg):
        odom = msg        
        self.position = msg.pose.pose.position
        _, _, self.heading = self.euler_from_quaternion(msg.pose.pose.orientation)
        # self.get_logger().info('heading: ' + str(self.heading))
        
    def get_key(self):
        self.goal_position.x = float(input("goal x: "))
        self.goal_position.y = float(input("goal y: "))
        
        self.goal_heading = float(input("goal heading: "))
        if self.goal_heading >= math.pi:
            self.goal_heading = self.goal_heading % (math.pi * 180.0 / math.pi)
        elif self.goal_heading <= -math.pi:
            self.goal_heading = -(-self.goal_heading % (math.pi * 180.0 / math.pi))

        self.goal_heading = self.goal_heading * math.pi / 180.0

        self.get_logger().info(str(self.goal_position.x) + str(self.goal_position.y) + str(self.goal_heading))

    def euler_from_quaternion(self, quat):
        """
        Convert quaternion (w in last place) to euler roll, pitch, yaw.

        quat = [x, y, z, w]
        """
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = numpy.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = numpy.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = numpy.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw




def main(args=None):
    rclpy.init()

    turtlebot3_pointop_key = Turtlebot3PointOpKey()
    
    rclpy.spin(turtlebot3_pointop_key)

    turtlebot3_pointop_key.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()