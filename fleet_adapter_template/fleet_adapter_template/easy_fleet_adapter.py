# Copyright 2023 Open Source Robotics Foundation, Inc.
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

import sys
import argparse
import yaml
import time
import enum

import rclpy
import rclpy.node
from rclpy.parameter import Parameter

import rmf_adapter as adpt

from functools import partial

from .RobotClientAPI import RobotAPI

# ------------------------------------------------------------------------------
# Helper functions
# ------------------------------------------------------------------------------


# States for RobotCommandHandle's state machine used when guiding robot along
# a new path
class RobotState(enum.IntEnum):
    IDLE = 0
    MOVING = 1


class FleetAdapter:

    def __init__(self, config_path, nav_graph_path, node, use_sim_time):
        # Load config yaml
        with open(config_path, "r") as f:
            config_yaml = yaml.safe_load(f)
        # Initialize robot API for this fleet
        fleet_config = config_yaml['rmf_fleet']
        self.api = RobotAPI(
            fleet_config['fleet_manager']['prefix'],
            fleet_config['fleet_manager']['user'],
            fleet_config['fleet_manager']['password'])

        node.declare_parameter('server_uri', rclpy.Parameter.Type.STRING)
        server_uri = node.get_parameter(
            'server_uri').get_parameter_value().string_value
        if server_uri == "":
            server_uri = None

        # Create EasyFullControl adapter
        self.configuration = adpt.easy_full_control.Configuration.make(
            config_path, nav_graph_path, server_uri)
        self.adapter = self.initialize_fleet(
            self.configuration, config_yaml['robots'], node, use_sim_time)

    def initialize_fleet(self, configuration, robots_yaml, node, use_sim_time):
        # Make the easy full control
        easy_full_control = adpt.EasyFullControl.make(configuration)

        if use_sim_time:
            easy_full_control.node.use_sim_time()

        def _robot_state(robot_name):
            '''Returns a RobotState object that describes the robot's charger waypoint,
            current map, location and battery status. Return None if not available.
            Here you can use RobotAPI and call self.api.position() and self.api.battery()
            to provide the RobotState items.'''
            # ------------------------ #
            # IMPLEMENT YOUR CODE HERE #
            # ------------------------ #
            return None

        def _navigate(robot_name, map_name, goal, execution):
            '''Send a navigation goal to your robot given the map name and goal (x, y, yaw)
            using self.api.navigate(), and call execution.finished() when it has successfully
            reached the target. You can use self.api.navigation_remaining_duration() and
            self.api.navigation_completed() to track the progress of the robot.'''
            # ------------------------ #
            # IMPLEMENT YOUR CODE HERE #
            # ------------------------ #

        def _stop(robot_name):
            '''Send a stop command to your robot using self.api.stop().'''
            # ------------------------ #
            # IMPLEMENT YOUR CODE HERE #
            # ------------------------ #

        def _dock(robot_name, dock_name, execution):
            '''Send a docking goal to your robot given the dock name with self.api.dock(),
            and call execution.finished() when it has successfully docked. You can use
            self.api.process_completed() to track the progress of the robot.'''
            # ------------------------ #
            # IMPLEMENT YOUR CODE HERE #
            # ------------------------ #
        def _action_executor(
                robot_name: str,
                category: str,
                description: dict,
                execution: adpt.robot_update_handle.ActionExecution):
            '''Command your robot to start an action using self.api.start_process() and call
            execution.finished() when it has successfully completed the action. You can use
            self.api.process_completed() to track the process of the robot.'''
            # ------------------------ #
            # IMPLEMENT YOUR CODE HERE #
            # ------------------------ #

        # Add the robots
        missing_robots = robots_yaml
        while len(missing_robots) > 0:
            for robot in list(missing_robots.keys()):
                node.get_logger().info(f'Connecting to robot [{robot}]')
                robot_config = robots_yaml[robot]['rmf_config']
                state = _robot_state(robot)
                if state is None:
                    node.get_logger().info(f'Unable to find robot [{robot}], trying again...')
                    time.sleep(0.2)
                    continue
                # Found robot, add to fleet
                easy_full_control.add_robot(
                    state,
                    partial(_robot_state, robot),
                    partial(_navigate, robot),
                    partial(_stop, robot),
                    partial(_dock, robot),
                    partial(_action_executor, robot))
                node.get_logger().info(f'Successfully added new robot: [{robot}]')
                del missing_robots[robot]

        return easy_full_control


# ------------------------------------------------------------------------------
# Main
# ------------------------------------------------------------------------------
def main(argv=sys.argv):
    # Init rclpy and adapter
    rclpy.init(args=argv)
    adpt.init_rclcpp()
    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    parser = argparse.ArgumentParser(
        prog="fleet_adapter",
        description="Configure and spin up the fleet adapter")
    parser.add_argument("-c", "--config_file", type=str, required=True,
                        help="Path to the config.yaml file")
    parser.add_argument("-n", "--nav_graph", type=str, required=True,
                        help="Path to the nav_graph for this fleet adapter")
    parser.add_argument("-sim", "--use_sim_time", action="store_true",
                        help='Use sim time, default: false')
    args = parser.parse_args(args_without_ros[1:])
    print(f"Starting fleet adapter...")

    config_path = args.config_file
    nav_graph_path = args.nav_graph

    # Load config and nav graph yamls
    with open(config_path, "r") as f:
        config_yaml = yaml.safe_load(f)

    # ROS 2 node for the command handle
    fleet_name = config_yaml['rmf_fleet']['name']
    node = rclpy.node.Node(f'{fleet_name}_command_handle')

    # Enable sim time for testing offline
    if args.use_sim_time:
        param = Parameter("use_sim_time", Parameter.Type.BOOL, True)
        node.set_parameters([param])

    adapter = FleetAdapter(
        config_path,
        nav_graph_path,
        node,
        args.use_sim_time)

    # Create executor for the command handle node
    rclpy_executor = rclpy.executors.SingleThreadedExecutor()
    rclpy_executor.add_node(node)

    # Start the fleet adapter
    rclpy_executor.spin()

    # Shutdown
    node.destroy_node()
    rclpy_executor.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
