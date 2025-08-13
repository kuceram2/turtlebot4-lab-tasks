#!/usr/bin/env python3

import rclpy

#from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator

import time
import sys
from my_turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator

def main():
    rclpy.init()
    namespace = sys.argv[1]
    navigator = TurtleBot4Navigator(namespace)



    # navigator.undock()
    # navigator.dock()
    navigator.get_logger().info(str(navigator.getDockedStatus()))
    time.sleep(5)
    navigator.dock()

    # Start on dock
    # if not navigator.getDockedStatus():
    #     navigator.info('Docking before intialising pose')
    #     navigator.dock()

    # Set initial pose
    # initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
    # navigator.setInitialPose(initial_pose)

    # Wait for Nav2
    navigator.waitUntilNav2Active()

    # Set goal poses
    goal_pose = navigator.getPoseStamped([-1.0, 0.0], TurtleBot4Directions.EAST)

    # Undock
    navigator.undock()

    # Go to each goal pose
    navigator.startToPose(goal_pose)

    navigator.dock()

    rclpy.shutdown()


if __name__ == '__main__':
    main()