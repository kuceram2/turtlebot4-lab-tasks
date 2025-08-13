#!/usr/bin/env python3


from enum import IntEnum
import math
from threading import Thread
import time

from action_msgs.msg import GoalStatus

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

from irobot_create_msgs.action import Dock, Undock, DriveDistance, RotateAngle, DriveArc
from irobot_create_msgs.msg import DockStatus

#from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from my_tb_nodes.my_robot_navigator import BasicNavigator, TaskResult

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default


class TurtleBot4Directions(IntEnum):
    NORTH = 0
    NORTH_WEST = 45
    WEST = 90
    SOUTH_WEST = 135
    SOUTH = 180
    SOUTH_EAST = 225
    EAST = 270
    NORTH_EAST = 315


class TurtleBot4Navigator(BasicNavigator):
    is_docked = None
    creating_path = False

    def __init__(self, namespace=''):
        super().__init__(namespace=namespace)


        self.create_subscription(DockStatus,
                                 'dock_status',
                                 self._dockCallback,
                                 qos_profile_sensor_data)

        self.create_subscription(PoseWithCovarianceStamped,
                                 'initialpose',
                                 self._poseEstimateCallback,
                                 qos_profile_system_default)

        self.undock_action_client = ActionClient(self, Undock, 'undock')
        self.dock_action_client = ActionClient(self, Dock, 'dock')
        self.drive_distance_action_client = ActionClient(self, DriveDistance, 'drive_distance')
        self.rotate_angle_action_client = ActionClient(self, RotateAngle, 'rotate_angle')
        self.drive_arc_action_client = ActionClient(self, DriveArc, 'drive_arc')

        

        self.undock_result_future = None
        self.dock_result_future = None

    def getPoseStamped(self, position, rotation):
        """
        Fill and return a PoseStamped message.

        :param position: A list consisting of the x and y positions for the Pose. e.g [0.5, 1.2]
        :param rotation: Rotation of the pose about the Z axis in degrees.
        :return: PoseStamped message
        """
        pose = PoseStamped()

        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]

        # Convert Z rotation to quaternion
        pose.pose.orientation.z = math.sin(math.radians(rotation) / 2)
        pose.pose.orientation.w = math.cos(math.radians(rotation) / 2)

        return pose

    def stampPose(self, pose):
        """
        Stamp a Pose message and return a PoseStamped message.

        :param pose: Pose message
        :return: PoseStamped message
        """
        poseStamped = PoseStamped()

        poseStamped.header.frame_id = 'map'
        poseStamped.header.stamp = self.get_clock().now().to_msg()

        poseStamped.pose = pose

        return poseStamped

    def createPath(self):
        """
        Create a path using the '2D Pose Estimate' tool in Rviz.

        :return: List of PoseStamped poses
        """
        poses = []
        self.new_pose = None
        self.creating_path = True

        self.info('Creating a path. Press Enter to finish.')
        self.info('Use the "2D Pose Estimate" tool in Rviz to add a pose to the path.')

        def wait_for_key():
            input()

        input_thread = Thread(target=wait_for_key, daemon=True)
        input_thread.start()

        while self.creating_path:
            while self.new_pose is None:
                if input_thread.is_alive():
                    rclpy.spin_once(self, timeout_sec=0.1)
                else:
                    self.creating_path = False
                    break
            if self.new_pose:
                self.info('Pose added.')
                poses.append(self.stampPose(self.new_pose))
                self.new_pose = None
                self.clearAllCostmaps()
        if len(poses) > 0:
            self.info('Path created.')
            for i, p in enumerate(poses):
                self.info('Pose {0} [x,y]=[{1:.3f},{2:.3f}]'.format(
                    i, p.pose.position.x, p.pose.position.y) +
                    '[x,y,z,w]=[{0:.3f},{1:.3f},{2:.3f},{3:.3f}]'.format(
                    p.pose.orientation.x, p.pose.orientation.y,
                    p.pose.orientation.z, p.pose.orientation.w))
        return poses

    # 2D Pose Estimate callback
    def _poseEstimateCallback(self, msg: PoseWithCovarianceStamped):
        if self.creating_path:
            self.new_pose = msg.pose.pose

    # DockStatus subscription callback
    def _dockCallback(self, msg: DockStatus):
        self.is_docked = msg.is_docked

    def getDockedStatus(self):
        """
        Get current docked status.

        :return: ``True`` if docked, ``False`` otherwise.
        """
        # Spin to get latest dock status
        rclpy.spin_once(self, timeout_sec=0.1)
        # If dock status hasn't been published yet, spin until it is
        while self.is_docked is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        return self.is_docked

    def undock(self):
        """Perform Undock action."""
        self.info('Undocking...')
        self.undock_send_goal()

        while not self.isUndockComplete():
            time.sleep(0.1)

    def undock_send_goal(self):
        goal_msg = Undock.Goal()
        self.undock_action_client.wait_for_server()
        self.info("Waiting for server")
        goal_future = self.undock_action_client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self, goal_future)

        self.undock_goal_handle = goal_future.result()

        if not self.undock_goal_handle.accepted:
            self.error('Undock goal rejected')
            return

        self.undock_result_future = self.undock_goal_handle.get_result_async()

    def isUndockComplete(self):
        """
        Get status of Undock action.

        :return: ``True`` if undocked, ``False`` otherwise.
        """
        if self.undock_result_future is None or not self.undock_result_future:
            return True

        rclpy.spin_until_future_complete(self, self.undock_result_future, timeout_sec=0.1)

        if self.undock_result_future.result():
            self.undock_status = self.undock_result_future.result().status
            if self.undock_status != GoalStatus.STATUS_SUCCEEDED:
                self.info(f'Goal with failed with status code: {self.status}')
                return True
        else:
            return False

        self.info('Undock succeeded')
        return True

    def dock(self):
        """Perform Undock action."""
        self.info('Docking...')
        self.dock_send_goal()

        while not self.isDockComplete():
            time.sleep(0.1)

    def dock_send_goal(self):
        goal_msg = Dock.Goal()
        self.dock_action_client.wait_for_server()
        goal_future = self.dock_action_client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self, goal_future)

        self.dock_goal_handle = goal_future.result()

        if not self.dock_goal_handle.accepted:
            self.error('Dock goal rejected')
            return

        self.dock_result_future = self.dock_goal_handle.get_result_async()

    def isDockComplete(self):
        """
        Get status of Dock action.

        :return: ``True`` if docked, ``False`` otherwise.
        """
        if self.dock_result_future is None or not self.dock_result_future:
            return True

        rclpy.spin_until_future_complete(self, self.dock_result_future, timeout_sec=0.1)

        if self.dock_result_future.result():
            self.dock_status = self.dock_result_future.result().status
            if self.dock_status != GoalStatus.STATUS_SUCCEEDED:
                self.info(f'Goal with failed with status code: {self.status}')
                return True
        else:
            return False

        self.info('Dock succeeded')
        return True

    def startToPose(self, pose: PoseStamped):
        """
        Perform goToPose action and print feedback.

        :param pose: Goal pose.
        """
        i = 0
        self.goToPose(pose)

        while not self.isTaskComplete():
            feedback = self.getFeedback()
            i = i + 1
            if feedback and i % 5 == 0:
                print('Estimated time of arrival: ' + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    + '{0: <20}'.format('seconds.'), end='\r')

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self.cancelTask()

        result = self.getResult()
        if result == TaskResult.SUCCEEDED:
            self.info('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            self.info('Goal was canceled!')
        elif result == TaskResult.FAILED:
            self.info('Goal failed!')
        else:
            self.info('Goal has an invalid return status!')

    def startThroughPoses(self, poses):
        """
        Perform goThroughPoses action and print feedback.

        :param poses: List of goal poses.
        """
        i = 0
        self.goThroughPoses(poses)

        while not self.isTaskComplete():
            feedback = self.getFeedback()
            i = i + 1
            if feedback and i % 5 == 0:
                print('Estimated time of arrival: ' + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    + '{0: <20}'.format(' seconds.'), end='\r')

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self.cancelTask()

        result = self.getResult()
        if result == TaskResult.SUCCEEDED:
            self.info('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            self.info('Goal was canceled!')
        elif result == TaskResult.FAILED:
            self.info('Goal failed!')
        else:
            self.info('Goal has an invalid return status!')

    def startFollowWaypoints(self, poses):
        """
        Perform followWaypoint action and print feedback.

        :param poses: List of goal poses.
        """
        i = 0
        self.followWaypoints(poses)

        while not self.isTaskComplete():
            i = i + 1
            feedback = self.getFeedback()
            if feedback and i % 5 == 0:
                print('Executing current waypoint: {0}/{1: <5}'.format(
                    str(feedback.current_waypoint + 1), str(len(poses))), end='\r')
                
    def drive_distance(self, distance = 0.0):
        """Perform DriveDistance action."""
        self.info('Driving...')
        self.drive_distance_send_goal(distance)

        while not self.isDriveDistanceComplete():
            time.sleep(0.1)

    def drive_distance_send_goal(self, distance = 0):
        goal_msg = DriveDistance.Goal()
        goal_msg.distance = distance
        self.drive_distance_action_client.wait_for_server()
        goal_future = self.drive_distance_action_client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self, goal_future)

        self.drive_distance_goal_handle = goal_future.result()

        if not self.drive_distance_goal_handle.accepted:
            self.error('Drive distance: goal rejected')
            return

        self.drive_distance_result_future = self.drive_distance_goal_handle.get_result_async()

    def isDriveDistanceComplete(self):
        """
        Get status of DriveDistance action.

        :return: ``True`` if docked, ``False`` otherwise.
        """
        if self.drive_distance_result_future is None or not self.drive_distance_result_future:
            return True

        rclpy.spin_until_future_complete(self, self.drive_distance_result_future, timeout_sec=0.1)

        if self.drive_distance_result_future.result():
            self.drive_distance_status = self.drive_distance_result_future.result().status
            if self.drive_distance_status != GoalStatus.STATUS_SUCCEEDED:
                self.info(f'Goal with failed with status code: {self.status}')
                return True
        else:
            return False

        self.info('Drive Distance succeeded')
        return True
    
    def rotate_angle_deg(self, angle_deg):
        angle_rad = math.radians(angle_deg)
        self.rotate_angle(angle_rad)

    def rotate_angle(self, angle = 0.0):
        """Perform RotateAngle action."""
        self.info('Rotating...')
        self.rotate_angle_send_goal(angle)

        while not self.isRotateAngleComplete():
            time.sleep(0.1)

    def rotate_angle_send_goal(self, angle = 0):
        goal_msg = RotateAngle.Goal()
        goal_msg.angle = angle
        self.rotate_angle_action_client.wait_for_server()
        goal_future = self.rotate_angle_action_client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self, goal_future)

        self.rotate_angle_goal_handle = goal_future.result()

        if not self.rotate_angle_goal_handle.accepted:
            self.error('Rotate Angle: goal rejected')
            return

        self.rotate_angle_result_future = self.rotate_angle_goal_handle.get_result_async()

    def isRotateAngleComplete(self):
        """
        Get status of RotateAngle action.

        :return: ``True`` if docked, ``False`` otherwise.
        """
        if self.rotate_angle_result_future is None or not self.rotate_angle_result_future:
            return True

        rclpy.spin_until_future_complete(self, self.rotate_angle_result_future, timeout_sec=0.1)

        if self.rotate_angle_result_future.result():
            self.rotate_angle_status = self.rotate_angle_result_future.result().status
            if self.rotate_angle_status != GoalStatus.STATUS_SUCCEEDED:
                self.info(f'Goal with failed with status code: {self.status}')
                return True
        else:
            return False

        self.info('Rotate Angle succeeded')
        return True
    
    def drive_arc(self, angle = 0.0, radius = 0.0):
        """Perform DriveArc action."""
        self.info('Rotating...')
        self.drive_arc_send_goal(angle, radius)

        while not self.isDriveArcComplete():
            time.sleep(0.1)

    def drive_arc_send_goal(self, angle = 0, radius = 0):
        goal_msg = DriveArc.Goal()
        goal_msg.angle = angle
        goal_msg.radius = radius
        self.drive_arc_action_client.wait_for_server()
        goal_future = self.drive_arc_action_client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self, goal_future)

        self.drive_arc_goal_handle = goal_future.result()

        if not self.drive_arc_goal_handle.accepted:
            self.error('Drive Arc: goal rejected')
            return

        self.drive_arc_result_future = self.drive_arc_goal_handle.get_result_async()

    def isDriveArcComplete(self):
        """
        Get status of DriveArc action.

        :return: ``True`` if docked, ``False`` otherwise.
        """
        if self.drive_arc_result_future is None or not self.drive_arc_result_future:
            return True

        rclpy.spin_until_future_complete(self, self.drive_arc_result_future, timeout_sec=0.1)

        if self.drive_arc_result_future.result():
            self.drive_arc_status = self.drive_arc_result_future.result().status
            if self.drive_arc_status != GoalStatus.STATUS_SUCCEEDED:
                self.info(f'Goal with failed with status code: {self.status}')
                return True
        else:
            return False

        self.info('Drive Arc succeeded')
        return True
    