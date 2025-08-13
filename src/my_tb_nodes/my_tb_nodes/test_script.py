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


    navigator.dock()
    # #navigator.undock()
    # navigator.drive_distance(0.1)
    # navigator.rotate_angle_deg(180)

    rclpy.shutdown()


if __name__ == '__main__':
    main()