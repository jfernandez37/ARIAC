#!/usr/bin/env python3

import rclpy
import time

from ariac_gazebo.environment_startup import EnvironmentStartup

def main():
    rclpy.init()

    startup_node = EnvironmentStartup()

    # Wait five seconds for gazebo to start up
    time.sleep(2)

    # Parse the trial config file
    startup_node.parse_trial_file()

    # Spawn robots
    startup_node.spawn_robots()

    # Spawn sensors
    startup_node.spawn_sensors()

    # Read conveyor part config
    startup_node.parse_conveyor_config()

    startup_node.environment_ready()

    try:
        rclpy.spin(startup_node)
    except KeyboardInterrupt:
        startup_node.destroy_node()


if __name__ == '__main__':
    main()