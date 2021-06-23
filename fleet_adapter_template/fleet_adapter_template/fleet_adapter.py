# Copyright 2020 Open Source Robotics Foundation, Inc.
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
import nudged

import rclpy
import rclpy.node
from rclpy.parameter import Parameter

import rmf_adapter as adpt
import rmf_adapter.vehicletraits as traits
import rmf_adapter.battery as battery
import rmf_adapter.geometry as geometry
import rmf_adapter.graph as graph
import rmf_adapter.plan as plan

from rmf_task_msgs.msg import TaskProfile, TaskType

from functools import partial

from .RobotCommandHandle import RobotCommandHandle

#------------------------------------------------------------------------------
# Helper functions
#------------------------------------------------------------------------------
def initialize_fleet(config_yaml, nav_graph_path, node):
    # Profile and traits
    profile = traits.Profile(geometry.make_final_convex_circle(
        config_yaml['rmf_fleet']['profile']['footprint']),
        geometry.make_final_convex_circle(config_yaml['rmf_fleet']['profile']['vicinity']))
    vehicle_traits = traits.VehicleTraits(
        linear=traits.Limits(*config_yaml['rmf_fleet']['limits']['linear']),
        angular=traits.Limits(*config_yaml['rmf_fleet']['limits']['angular']),
        profile=profile)
    vehicle_traits.differential.reversible = config_yaml['rmf_fleet']['reversible']

    # Battery system
    voltage = config_yaml['rmf_fleet']['battery_system']['voltage']
    capacity = config_yaml['rmf_fleet']['battery_system']['capacity']
    charging_current = config_yaml['rmf_fleet']['battery_system']['charging_current']
    battery_sys = battery.BatterySystem.make(voltage, capacity, charging_current)

    # Mechanical system
    mass = config_yaml['rmf_fleet']['mechanical_system']['mass']
    moment = config_yaml['rmf_fleet']['mechanical_system']['moment_of_inertia']
    friction = config_yaml['rmf_fleet']['mechanical_system']['friction_coefficient']
    mech_sys = battery.MechanicalSystem.make(mass, moment, friction)

    # Power systems
    ambient_power_sys = battery.PowerSystem.make(
        config_yaml['rmf_fleet']['ambient_system']['power'])
    tool_power_sys = battery.PowerSystem.make(
        config_yaml['rmf_fleet']['tool_system']['power'])

    # Power sinks
    motion_sink = battery.SimpleMotionPowerSink(battery_sys, mech_sys)
    ambient_sink = battery.SimpleDevicePowerSink(battery_sys, ambient_power_sys)
    tool_sink = battery.SimpleDevicePowerSink(battery_sys, tool_power_sys)

    nav_graph = graph.parse_graph(nav_graph_path, vehicle_traits)

    # Adapter
    fleet_name = config_yaml['rmf_fleet']['name']
    adapter = adpt.Adapter.make(f'{fleet_name}_fleet_adapter')

    assert adapter, ("Unable to initialize fleet adapter. Please ensure "
                     "RMF Schedule Node is running")
    fleet_handle = adapter.add_fleet(fleet_name, vehicle_traits, nav_graph)

    if not config_yaml['rmf_fleet']['publish_fleet_state']:
        fleet_handle.fleet_state_publish_period(None)
    # Account for battery drain
    drain_battery = config_yaml['rmf_fleet']['account_for_battery_drain']
    recharge_threshold = config_yaml['rmf_fleet']['recharge_threshold']
    recharge_soc = config_yaml['rmf_fleet']['recharge_soc']
    # Set task planner params
    ok = fleet_handle.set_task_planner_params(
        battery_sys,
        motion_sink,
        ambient_sink,
        tool_sink,
        recharge_threshold,
        recharge_soc,
        drain_battery)
    assert ok, ("Unable to set task planner params")

    # Callable for validating requests that this fleet can accommodate
    def task_request_check(msg: TaskProfile):
        ## ------------------------ ##
        ## IMPLEMENT YOUR CODE HERE ##
        ## Below is a an example for a fleet that can accept delivery and loop
        ## tasks
        ## ------------------------ ##
        if ((msg.description.task_type == TaskType.TYPE_DELIVERY) or
            (msg.description.task_type == TaskType.TYPE_LOOP)):
            return True
        else:
            return False
    fleet_handle.accept_task_requests(task_request_check)

    # Transforms
    rmf_coordinates = config_yaml['reference_coordinates']['rmf']
    robot_coordinates = config_yaml['reference_coordinates']['robot']
    transforms = {
        'rmf_to_robot': nudged.estimate(rmf_coordinates, robot_coordinates),
        'robot_to_rmf': nudged.estimate(robot_coordinates, rmf_coordinates)}
    transforms['orientation_offset'] = transforms['rmf_to_robot'].get_rotation()
    mse = nudged.estimate_error(transforms['rmf_to_robot'],
                                rmf_coordinates,
                                robot_coordinates)
    print(f"Coordinate transformation error: {mse}")
    print("RMF to Robot transform:")
    print(f"    rotation:{transforms['rmf_to_robot'].get_rotation()}")
    print(f"    scale:{transforms['rmf_to_robot'].get_scale()}")
    print(f"    trans:{transforms['rmf_to_robot'].get_translation()}")
    print("Robot to RMF transform:")
    print(f"    rotation:{transforms['robot_to_rmf'].get_rotation()}")
    print(f"    scale:{transforms['robot_to_rmf'].get_scale()}")
    print(f"    trans:{transforms['robot_to_rmf'].get_translation()}")

    def updater_inserter(cmd_handle, update_handle):
        """Insert a RobotUpdateHandle."""
        cmd_handle.update_handle = update_handle

    # Initialize robots for this fleet
    robots = {}
    for robot_name, robot_config in config_yaml['robots'].items():
        print(f"    Initializing robot:{robot_name}")
        rmf_config = robot_config['rmf_config']
        robot_config = robot_config['robot_config']
        initial_waypoint = rmf_config['start']['waypoint']
        initial_orientation = rmf_config['start']['orientation']
        robot = RobotCommandHandle(
            name = robot_name,
            config = robot_config,
            node = node,
            graph = nav_graph,
            vehicle_traits = vehicle_traits,
            transforms = transforms,
            map_name = rmf_config['start']['map_name'],
            initial_waypoint = initial_waypoint,
            initial_orientation = initial_orientation,
            charger_waypoint = rmf_config['charger']['waypoint'],
            update_frequency = rmf_config.get('robot_state_update_frequency', 1),
            adapter = adapter)

        if robot.initialized:
            robots[robot_name] = robot
            # Add robot to fleet
            fleet_handle.add_robot(robot,
                                   robot_name,
                                   profile,
                                   robot.starts,
                                   partial(updater_inserter, robot))
            print(f"    Successfully added new robot:{robot_name}")

        else:
            print(f"    Failed to initialize robot:{robot_name}")

    return adapter, fleet_handle, robots


#------------------------------------------------------------------------------
# Main
#------------------------------------------------------------------------------
def main(argv=sys.argv):
    # Init rclpy and adapter
    rclpy.init(args=argv)
    adpt.init_rclcpp()
    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    parser = argparse.ArgumentParser(
        prog="fleet_adapter",
        description="Configure and spin up the fleet adapter")
    parser.add_argument("-c", "--config_file", type=str, required=True,
                        help="Path to the config.yaml file for this fleet adapter")
    parser.add_argument("-n", "--nav_graph", type=str, required=True,
                    help="Path to the nav_graph for this fleet adapter")
    parser.add_argument("--use_sim_time", action="store_true",
                    help='Use sim time for testing offline, default: false')
    args = parser.parse_args(args_without_ros[1:])
    print(f"Starting fleet adapter...")

    config_path = args.config_file
    nav_graph_path = args.nav_graph

    # Load config and nav graph yamls
    with open(config_path, "r") as f:
        config_yaml = yaml.safe_load(f)

    # ROS 2 node for the command handle
    node = rclpy.node.Node('robot_command_handle')

    adapter,fleet_handle,robots = initialize_fleet(
        config_yaml,
        nav_graph_path,
        node)

    # Enable sim time for testing offline
    if args.use_sim_time:
        param = Parameter("use_sim_time", Parameter.Type.BOOL, True)
        adapter.node.use_sim_time()
        node.set_parameters([param])

    # Create executor for the command handle node
    rclpy_executor = rclpy.executors.SingleThreadedExecutor()
    rclpy_executor.add_node(node)

    # Start the fleet adapter
    adapter.start()
    rclpy_executor.spin()

    # Shutdown
    node.destroy_node()
    rclpy_executor.shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main(sys.argv)
