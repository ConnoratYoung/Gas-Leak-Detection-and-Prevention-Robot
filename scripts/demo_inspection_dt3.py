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

from copy import deepcopy

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
import math as m
rrad = 0.11

"""
Basic stock inspection demo. In this demonstration, the expectation
is that there are cameras or RFID sensors mounted on the robots
collecting information about stock quantity and location.
"""


def main():
    rclpy.init()

    navigator = BasicNavigator()		# Call Navigator

    # Inspection route - Added bearing as third parameter in matrix
    inspection_route = [
    	[0.0, 0.0, 0.0],			# Origin, added a 3rd coordinte for headings
        [1.0+rrad, 0.0, m.pi/2], 		# Botton Right Corner
        [1.0+rrad, 1.0+rrad, m.pi],		# Top Right Corner
        [0.0-rrad, 1.0+rrad, -m.pi/2],		# Top Left Corner
        [0.0-rrad, 0.0-rrad, 0.0]		# Bottom Left Corner (origin)
    ]		

    # Set our demo's initial pose - Commented out for DT3
    #initial_pose = PoseStamped()
    #initial_pose.header.frame_id = 'map'
    #initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    #initial_pose.pose.position.x = 0.3
    #initial_pose.pose.position.y = 0.3
    #initial_pose.pose.orientation.w = 0.0
    #initial_pose.pose.orientation.z = 1.0
    #navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active()

    # Send our route
    inspection_points = []
    inspection_pose = PoseStamped()
    inspection_pose.header.frame_id = 'map'
    inspection_pose.header.stamp = navigator.get_clock().now().to_msg()
    for pt in inspection_route:
        inspection_pose.pose.position.x = pt[0]			# Set x target to x of current waypoint
        inspection_pose.pose.position.y = pt[1]			# Set y target to y of current waypoint
        inspection_pose.pose.orientation.w = m.cos(pt[2]/2)	# Convert from euler to quaternion
        inspection_pose.pose.orientation.z = m.sin(pt[2]/2)	#		"
        inspection_points.append(deepcopy(inspection_pose))	# Append point to inspection pose
    navigator.followWaypoints(inspection_points)		# Follow waypoints

    # Do something during our route (e.x. AI to analyze stock information or upload to the cloud)
    # Simply the current waypoint ID for the demonstation
    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Executing current waypoint: ' +
                  str(feedback.current_waypoint + 1) + '/' + str(len(inspection_points)))

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Inspection of shelves complete! Returning to start...')
    elif result == TaskResult.CANCELED:
        print('Inspection of shelving was canceled. Returning to start...')
    elif result == TaskResult.FAILED:
        print('Inspection of shelving failed! Returning to start...')

    # go back to start
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    navigator.goToPose(initial_pose)
    while not navigator.isTaskComplete():
        pass

    exit(0)


if __name__ == '__main__':
    main()
    
