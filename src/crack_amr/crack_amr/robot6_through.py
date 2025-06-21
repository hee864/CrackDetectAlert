#!/usr/bin/env python3

# Copyright 2022 Clearpath Robotics, Inc.
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
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)
#이걸 go-to-pose로 전부 1,101,2,2-1,3,4,7,6-7 을 가는 걸 전부 go-to-pose로 하기 

import rclpy

from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator



def main():
    rclpy.init()

    navigator = TurtleBot4Navigator(namespace='robot7')


    navigator.waitUntilNav2Active()

    # Set goal poses
    goal_pose = []
    
    goal_pose.append(navigator.getPoseStamped([-2.39,0.143], TurtleBot4Directions.WEST))#1
    goal_pose.append(navigator.getPoseStamped([-1.77, 1.85], TurtleBot4Directions.WEST))#1-1
    goal_pose.append(navigator.getPoseStamped([-0.461, 2.68],  TurtleBot4Directions.SOUTH_WEST))#2
    goal_pose.append(navigator.getPoseStamped([-2.16, 4.15], TurtleBot4Directions.WEST ))#2-1
    goal_pose.append(navigator.getPoseStamped([-2.01, 5.61],  TurtleBot4Directions.NORTH))#3
    goal_pose.append(navigator.getPoseStamped([-0.457, 5.28], TurtleBot4Directions.WEST))#4
    goal_pose.append(navigator.getPoseStamped([-0.457,5.28],TurtleBot4Directions.NORTH))#7
    goal_pose.append(navigator.getPoseStamped([-0.392,0.227],TurtleBot4Directions.NORTH))#6-6

    # Undock
    # navigator.undock()

    # Follow Waypoints
    navigator.startFollowWaypoints(goal_pose)

    # Finished navigating, dock
    # navigator.dock()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
