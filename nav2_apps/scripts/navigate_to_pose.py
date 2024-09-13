#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
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

import time
from copy import deepcopy

from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import rclpy

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# RB-1 positions
rb_1_positions = {
    "init_position": [0.020047, -0.020043, -0.019467, 1.000000],
    "loading_position": [5.653875, -0.186439, -0.746498, 0.665388],
    "face_shipping_position": [2.552175, -0.092728, 0.715685, 0.698423]}


def main():
    init_position = "init_position"
    loading_position = "loading_position"
    face_shipping_position = "face_shipping_position"

    rclpy.init()

    navigator = BasicNavigator()

    # Set your demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = rb_1_positions[init_position][0]
    initial_pose.pose.position.y = rb_1_positions[init_position][1]
    initial_pose.pose.orientation.z = rb_1_positions[init_position][2]
    initial_pose.pose.orientation.w = rb_1_positions[init_position][3]
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to activate fully
    navigator.waitUntilNav2Active()

    loading_pose = PoseStamped()
    loading_pose.header.frame_id = 'map'
    loading_pose.header.stamp = navigator.get_clock().now().to_msg()
    loading_pose.pose.position.x = rb_1_positions[loading_position][0]
    loading_pose.pose.position.y = rb_1_positions[loading_position][1]
    loading_pose.pose.orientation.z = rb_1_positions[loading_position][2]
    loading_pose.pose.orientation.w = rb_1_positions[loading_position][3]
    print('Received request for item picking at ' + loading_position + '.')
    navigator.goToPose(loading_pose)

    # Do something during your route
    # (e.x. queue up future tasks or detect person for fine-tuned positioning)
    # Print information for workers on the robot's ETA for the demonstration
    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival at ' + loading_position +
                  ' for worker: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Got product from ' + loading_position +
              '! Bringing product to shipping destination (' + face_shipping_position + ')...')
        face_shipping_pose = PoseStamped()
        face_shipping_pose.header.frame_id = 'map'
        face_shipping_pose.header.stamp = navigator.get_clock().now().to_msg()
        face_shipping_pose.pose.position.x = rb_1_positions[face_shipping_position][0]
        face_shipping_pose.pose.position.y = rb_1_positions[face_shipping_position][1]
        face_shipping_pose.pose.orientation.z = rb_1_positions[face_shipping_position][2]
        face_shipping_pose.pose.orientation.w = rb_1_positions[face_shipping_position][3]
        navigator.goToPose(face_shipping_pose)

    elif result == TaskResult.CANCELED:
        print('Task at ' + loading_position +
              ' was canceled. Returning to staging point...')
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        navigator.goToPose(initial_pose)

    elif result == TaskResult.FAILED:
        print('Task at ' + loading_position + ' failed!')
        exit(-1)

    while not navigator.isTaskComplete():
        pass

    exit(0)


if __name__ == '__main__':
    main()