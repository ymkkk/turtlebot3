#!/usr/bin/env python3

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import sys
from rclpy.node import Node

from interactive_markers import InteractiveMarkerServer
import rclpy
from visualization_msgs.msg import InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import Marker

from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class SimpleMarker(Node):

    def __init__(self):
        super().__init__('simple_marker')

        self.odom = Odometry()

        qos = QoSProfile(depth=10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)

        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.get_odom,
            10)
        self.odom_sub

    def get_odom(self, odom):
        self.odom = odom

    def processFeedback(self, feedback):
        p = feedback.pose.position
        print(f'{feedback.marker_name} is now at {p.x}, {p.y}, {p.z}')

        twist = Twist()
        twist.linear.x = p.x - self.odom.pose.pose.position.x

        self.get_logger().info("p.x: " + str(p.x) + "odom_x: " + str(self.odom.pose.pose.position.x))

        self.cmd_vel_pub.publish(twist)

        
def main(args=None):
    rclpy.init(args=sys.argv)
    simple_marker = SimpleMarker()
   
    qos = QoSProfile(depth=10)
    simple_marker.cmd_vel_pub = simple_marker.create_publisher(Twist, 'cmd_vel', qos)

    # create an interactive marker server on the namespace simple_marker
    server = InteractiveMarkerServer(simple_marker, 'simple_marker')

    # create an interactive marker for our server
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = 'base_link'
    int_marker.name = 'my_marker'
    int_marker.description = 'Simple 1-DOF Control'

    # create a grey box marker
    box_marker = Marker()
    box_marker.type = Marker.CUBE
    box_marker.scale.x = 0.45
    box_marker.scale.y = 0.45
    box_marker.scale.z = 0.45
    box_marker.color.r = 0.0
    box_marker.color.g = 0.5
    box_marker.color.b = 0.5
    box_marker.color.a = 1.0

    # create a non-interactive control which contains the box
    box_control = InteractiveMarkerControl()
    box_control.always_visible = True
    box_control.markers.append(box_marker)

    # add the control to the interactive marker
    int_marker.controls.append(box_control)

    # create a control which will move the box
    # this control does not contain any markers,
    # which will cause RViz to insert two arrows
    rotate_control = InteractiveMarkerControl()
    rotate_control.name = 'move_x'
    rotate_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

    # add the control to the interactive marker
    int_marker.controls.append(rotate_control)

    # add the interactive marker to our collection &
    # tell the server to call processFeedback() when feedback arrives for it
    server.insert(int_marker, feedback_callback=simple_marker.processFeedback)

    # 'commit' changes and send to all clients
    server.applyChanges()

    rclpy.spin(simple_marker)
    server.shutdown()


if __name__ == '__main__':
    main()