#!/usr/bin/env python3
# This script shows the usage of relative movements, this way you can 'hard code' a path for robot to follow
import rclpy

import time
import sys
from my_turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator

def main():
    rclpy.init()
    namespace = sys.argv[1]
    navigator = TurtleBot4Navigator(namespace)


    navigator.undock()
    navigator.drive_distance(0.5)
    navigator.rotate_angle_deg(-90)
    navigator.drive_arc(6.28, 0.5)
    navigator.rotate_angle_deg(90)
    #     navigator.drive_distance(0.5)
    #     navigator.rotate_angle_deg(90)
    
    navigator.dock()

    rclpy.shutdown()


if __name__ == '__main__':
    main()