#!/usr/bin/env python3

# Copyright 2022 Open Source Robotics Foundation, Inc.
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

"""
This utility script is used to get the robot coordinated (in robot map)
via rviz2. User can use the "publish point" feature, and click on the
map to get both `rmf_coordinate` and `robot_coordinate` on rviz2 map
"""

import sys
import argparse
import rclpy
from rclpy.node import Node
from .utils import RmfMapTransform
from geometry_msgs.msg import PointStamped

class ClickPointTransform(Node):
    def __init__(self, transform):
        super().__init__("click_point_transform")
        self.tf = transform
        self.subscription = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.clickpoint_callback,
            10)
        self.get_logger().info(f"Running click_point_transform")


    def clickpoint_callback(self, msg):
        x = msg.point.x
        y = msg.point.y
        gaussian_tf = RmfMapTransform(
            tx=self.tf[0], ty=self.tf[1],
            theta=self.tf[2], scale=self.tf[3])
        rx, ry, _ = gaussian_tf.to_robot_map([x, y, 0])
        print(f"\n\t rmf   coord (x, y) : [{x:.2f}, {y:.2f}]"
              f"\n\t robot coord (x, y) : [{rx:.2f}, {ry:.2f}]")


##########################################################################
##########################################################################

def main(argv=sys.argv):
    args_without_ros = rclpy.utilities.remove_ros_args(argv)
    parser = argparse.ArgumentParser(
        prog="click_point_transform",
        description="script to printout coordinates in"\
            " reference to robot_map, when click point on rviz2")
    parser.add_argument(
        "-tf", "--transform", required=True, nargs='+', type=float,
        help='transform in the form of [x, y, yaw, scale]')
    args = parser.parse_args(args_without_ros[1:])

    assert len(args.transform) == 4, "-tf should be [x, y, yaw, scale]"
    print(f"Starting click point transform...")
    rclpy.init()
    node = ClickPointTransform(args.transform)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
