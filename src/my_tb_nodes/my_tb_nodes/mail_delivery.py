#!/usr/bin/env python3


import sys
import rclpy

from my_turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator


def main():
    rclpy.init()
    namespace = sys.argv[1]
    navigator = TurtleBot4Navigator(namespace)

    # Start on dock
    if not navigator.getDockedStatus():
        navigator.info('Docking before intialising pose')
        navigator.dock()

    # Set initial pose
    # initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
    # navigator.setInitialPose(initial_pose)

    # Wait for Nav2
    navigator.waitUntilNav2Active()

    # Undock
    navigator.undock()

    # Prepare goal pose options
    goal_options = [
        {'name': 'Home',
         'pose': navigator.getPoseStamped([-0.20, 0.0], TurtleBot4Directions.NORTH)},

        {'name': 'Position 1',
         'pose': navigator.getPoseStamped([-1.03, -0.52], TurtleBot4Directions.WEST)},
        {'name': 'Position 2',
                'pose': navigator.getPoseStamped([0.05, 2.40], TurtleBot4Directions.WEST)},
        {'name': 'Position 3',
                'pose': navigator.getPoseStamped([-0.57, 3.40], TurtleBot4Directions.NORTH)},
        {'name': 'Position 4',
                'pose': navigator.getPoseStamped([-0.1, 4.2], TurtleBot4Directions.NORTH)},
        {'name': 'Exit',
         'pose': None}
    ]

    navigator.info('Welcome to the mail delivery service.')

    while True:
        # Create a list of the goals for display
        options_str = 'Please enter the number corresponding to the desired robot goal position:\n'
        for i in range(len(goal_options)):
            options_str += f'    {i}. {goal_options[i]["name"]}\n'

        # Prompt the user for the goal location
        raw_input = input(f'{options_str}Selection: ')

        selected_index = 0

        # Verify that the value input is a number
        try:
            selected_index = int(raw_input)
        except ValueError:
            navigator.error(f'Invalid goal selection: {raw_input}')
            continue

        # Verify that the user input is within a valid range
        if (selected_index < 0) or (selected_index >= len(goal_options)):
            navigator.error(f'Goal selection out of bounds: {selected_index}')

        # Check for exit
        elif goal_options[selected_index]['name'] == 'Exit':
            break

        else:
            # Navigate to requested position
            navigator.startToPose(goal_options[selected_index]['pose'])

    rclpy.shutdown()


if __name__ == '__main__':
    main()