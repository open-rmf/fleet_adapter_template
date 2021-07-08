# Copyright 2021 Open Source Robotics Foundation, Inc.
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


import rmf_adapter as adpt
import rmf_adapter.plan as plan
import rmf_adapter.schedule as schedule

from rmf_fleet_msgs.msg import DockSummary

import threading
import math
import copy
import enum
import time

from datetime import timedelta

from .RobotClientAPI import RobotAPI


# States for RobotCommandHandle's state machine used when guiding robot along
# a new path
class RobotState(enum.IntEnum):
    IDLE = 0
    WAITING = 1
    MOVING = 2

class RobotCommandHandle(adpt.RobotCommandHandle):
    def __init__(self,
                 name,
                 config,
                 node,
                 graph,
                 vehicle_traits,
                 transforms,
                 map_name,
                 initial_waypoint,
                 initial_orientation,
                 charger_waypoint,
                 update_frequency,
                 adapter):
        adpt.RobotCommandHandle.__init__(self)
        self.name = name
        self.config = config
        self.node = node
        self.graph = graph
        self.vehicle_traits = vehicle_traits
        self.transforms = transforms
        self.map_name = map_name
        self.initial_waypoint = initial_waypoint
        self.initial_orientation = initial_orientation
        # Get the index of the charger waypoint
        waypoint = self.graph.find_waypoint(charger_waypoint)
        assert waypoint, f"Charger waypoint {charger_waypoint} does not exist in the navigation graph"
        self.charger_waypoint_index = waypoint.index
        self.charger_is_set = False
        self.update_frequency = update_frequency
        self.update_handle = None # RobotUpdateHandle
        self.battery_soc = 1.0
        self.api = None
        self.position = [] # (x,y,theta) in RMF coordinates (meters, radians)
        self.initialized = False
        self.state = RobotState.IDLE
        self.dock_name = ""
        self.adapter = adapter

        self.requested_waypoints = [] # RMF Plan waypoints
        self.remaining_waypoints = []
        self.path_finished_callback = None
        self.next_arrival_estimator = None
        self.path_index = 0
        self.docking_finished_callback = None

        # RMF location trackers
        self.last_known_lane_index = None
        self.last_known_waypoint_index = None
        self.on_waypoint = None # if robot is waiting at a waypoint. This is a Graph::Waypoint index
        self.on_lane = None # if robot is travelling on a lane. This is a Graph::Lane index
        self.target_waypoint = None # this is a Plan::Waypoint
        self.dock_waypoint_index = None # The graph index of the waypoint the robot is currently docking into

        # Threading variables
        self._lock = threading.Lock()
        self._follow_path_thread = None
        self._quit_path_event = threading.Event()
        self._dock_thread = None
        self._quit_dock_event = threading.Event()

        # Establish connection with the robot
        self.api = RobotAPI(self.config['base_url'], self.config['user'], self.config['password'])
        assert self.api.connected, "Unable to connect to Robot API server"

        self.position = self.get_position() # RMF coordinates
        assert len(self.position) > 2, "Unable to get current location of the robot for initialization"
        self.node.get_logger().info(f"The robot is starting at: [{self.position[0]:.2f}, {self.position[1]:.2f}, {self.position[2]:.2f}")
        # Obtain StartSet for the robot
        self.starts = []
        time_now = self.adapter.now()
        if (self.initial_waypoint is not None) and\
            (self.initial_orientation is not None):
            self.node.get_logger().info(f"    Using provided initial waypoint [{self.initial_waypoint}] "
            f"and orientation [{self.initial_orientation:.2f}] to initialize "
            f"starts for robot:{self.name}")
            # Get the waypoint index for initial_waypoint
            initial_waypoint_index = self.graph.find_waypoint(self.initial_waypoint)
            self.starts =[plan.Start(time_now,
                                initial_waypoint_index,
                                self.initial_orientation)]
        else:
            self.node.get_logger().info(f"    Running compute_plan_starts for robot:{self.name}")
            self.starts = plan.compute_plan_starts(
                self.graph,
                self.map_name,
                self.position,
                time_now)

        assert self.starts, (f"Unable to determine StartSet for robot:{self.name}")
        start = self.starts[0]

        # Update tracking variables
        if start.lane is not None:  # If the robot is on a lane
            self.last_known_lane_index = start.lane
            self.on_lane = start.lane
            self.last_known_waypoint_index = start.waypoint
        else:  # Otherwise, the robot is on a waypoint
            self.last_known_waypoint_index = start.waypoint
            self.on_waypoint = start.waypoint

        self.state_update_timer = self.node.create_timer(
            1.0 / self.update_frequency,
            self.update)

        self.initialized = True

    def clear(self):
        self.requested_waypoints = []
        self.remaining_waypoints = []
        self.path_finished_callback = None
        self.next_arrival_estimator = None
        self.docking_finished_callback = None
        self.target_waypoint = None
        self.state = RobotState.IDLE

    def stop(self):
        # Stop the robot. Tracking variables should remain unchanged.
        while True:
            self.node.get_logger().info("Requesting robot to stop...")
            if self.api.stop():
                break
            time.sleep(1.0)
        if self._follow_path_thread is not None:
            self._quit_path_event.set()
            self._follow_path_thread.join()
            self._follow_path_thread = None
            self.clear()

    def follow_new_path(
        self,
        waypoints,
        next_arrival_estimator,
        path_finished_callback):

        self.stop()
        self._quit_path_event.clear()

        self.node.get_logger().info("Received new path to follow...")

        self.remaining_waypoints = self.get_remaining_waypoints(waypoints)
        assert next_arrival_estimator is not None
        assert path_finished_callback is not None
        self.next_arrival_estimator = next_arrival_estimator
        self.path_finished_callback = path_finished_callback

        def _follow_path():
            target_pose = []
            while (self.remaining_waypoints or self.state == RobotState.MOVING or self.state == RobotState.WAITING):
                # Check if we need to abort
                if self._quit_path_event.is_set():
                    self.node.get_logger().info("Aborting previously followed path")
                    return
                # State machine
                if self.state == RobotState.IDLE:
                    # Assign the next waypoint
                    self.target_waypoint = self.remaining_waypoints[0][1]
                    self.path_index = self.remaining_waypoints[0][0]
                    # Move robot to next waypoint
                    target_pose =  self.target_waypoint.position
                    [x,y] = self.transforms["rmf_to_robot"].transform(target_pose[:2])
                    theta = math.degrees(target_pose[2] + self.transforms['orientation_offset'])
                    # ------------------------ #
                    # IMPLEMENT YOUR CODE HERE #
                    # Ensure x, y, theta are in units that api.navigate() expects #
                    # ------------------------ #
                    response = self.api.navigate([x, y, theta], self.map_name)

                    if response:
                        self.remaining_waypoints = self.remaining_waypoints[1:]
                        self.state = RobotState.MOVING
                        with self._lock:
                            if self.on_waypoint is not None: # robot starts at a graph waypoint
                                self.last_known_waypoint_index = self.on_waypoint
                                # update the lane the robot will be travelling on
                                if self.target_waypoint.graph_index is not None:
                                    lane = self.graph.lane_from(
                                        self.last_known_waypoint_index,
                                        self.target_waypoint.graph_index)
                                    if lane is not None:
                                        self.on_lane = lane.index
                            self.on_waypoint = None
                    else:
                        self.node.get_logger().info(f"Robot {self.name} failed to navigate to [{x:.0f}, {y:.0f}, {theta:.0f}] coordinates. Retrying...")
                        time.sleep(1.0)

                elif self.state == RobotState.WAITING:
                    time.sleep(1.0)
                    time_now = self.adapter.now()
                    with self._lock:
                        waypoint_wait_time = self.target_waypoint.time
                        if (waypoint_wait_time < time_now):
                            self.state = RobotState.IDLE
                            self.target_waypoint = None
                        else:
                            self.node.get_logger().info(f"Waiting for {(waypoint_wait_time - time_now).seconds}s")
                            self.next_arrival_estimator(self.path_index, timedelta(seconds=0.0))


                elif self.state == RobotState.MOVING:
                    time.sleep(1.0)
                    # Check if we have reached the target
                    with self._lock:
                        if (self.api.navigation_completed()):
                            self.node.get_logger().info(f"Robot [{self.name}] has reached its target waypoint")
                            self.state = RobotState.WAITING
                            if (self.target_waypoint.graph_index is not None):
                                self.on_waypoint = self.target_waypoint.graph_index
                            else:
                                self.on_waypoint = None # we are still on a lane
                        else:
                            # ------------------------ #
                            # IMPLEMENT YOUR CODE HERE #
                            # If your robot does not have an API to report the
                            # remaining travel duration, replace the API call
                            # below with an estimation
                            # ------------------------ #
                            duration = self.api.navigation_remaining_duration()
                            self.next_arrival_estimator(self.path_index, timedelta(seconds=duration))
            self.path_finished_callback()
            self.node.get_logger().info(f"Robot {self.name} has successfully navigated along requested path.")

        self._follow_path_thread = threading.Thread(
            target=_follow_path)
        self._follow_path_thread.start()

    def dock(
        self,
        dock_name,
        docking_finished_callback):

        ''' Docking is very specific to each application. Hence, the user will
            need to customize this function accordingly. In this example, we
            assume the dock_name is the same as the name of the waypoints that
            the robot is trying to dock into. We then call api.start_process()
            to initiate the robot specific process. This could be to start a
            cleaning process or load/unload a cart for delivery.
        '''

        self._quit_dock_event.clear()
        if self._dock_thread is not None:
            self._dock_thread.join()

        self.dock_name = dock_name
        assert docking_finished_callback is not None
        self.docking_finished_callback = docking_finished_callback

        # Get the waypoint that the robot is trying to dock into
        dock_waypoint = self.graph.find_waypoint(self.dock_name)
        assert(dock_waypoint)
        self.dock_waypoint_index = dock_waypoint.index

        def _dock():
            # Request the robot to start the relevant process
            self.node.get_logger().info(f"Requesting robot {self.name} to dock at {self.dock_name}")
            self.api.start_process(self.dock_name, self.map_name)

            self.on_waypoint = None
            self.on_lane = None
            time.sleep(1.0)
            # ------------------------ #
            # IMPLEMENT YOUR CODE HERE #
            # With whatever logic you need for docking #
            # ------------------------ #
            while (not self.api.docking_completed()):
                # Check if we need to abort
                if self._quit_dock_event.is_set():
                    self.node.get_logger().info("Aborting docking")
                    return
                self.node.get_logger().info("Robot is docking...")
                time.sleep(1.0)

            with self._lock:
                self.on_waypoint = self.dock_waypoint_index
                self.dock_waypoint_index = None
                self.docking_finished_callback()

        self._dock_thread = threading.Thread(target=_dock)
        self._dock_thread.start()

    def get_position(self):
        ''' This helper function returns the live position of the robot in the
        RMF coordinate frame'''
        position = self.api.position()
        if position is not None:
            x,y = self.transforms['robot_to_rmf'].transform([position[0],position[1]])
            theta = math.radians(position[2]) - self.transforms['orientation_offset']
            # ------------------------ #
            # IMPLEMENT YOUR CODE HERE #
            # Ensure x, y are in meters and theta in radians #
            # ------------------------ #
            return [x,y,theta]
        else:
            self.node.get_logger().error("Unable to retrieve position from robot. Returning last known position...")
            return self.position

    def get_battery_soc(self):
        battery_soc = self.api.battery_soc()
        if battery_soc is not None:
            return battery_soc
        else:
            self.node.get_logger().error("Unable to retrieve battery data from robot. Returning last known value...")
            return self.battery_soc

    def update(self):
        self.position = self.get_position()
        self.battery_soc = self.get_battery_soc()
        if self.update_handle is not None:
            self.update_state()

    def update_state(self):
        self.update_handle.update_battery_soc(self.battery_soc)
        if not self.charger_is_set:
            if (self.charger_waypoint_index < self.graph.num_waypoints):
                self.update_handle.set_charger_waypoint(self.charger_waypoint_index)
            else:
                self.node.get_logger().warn("Invalid waypoint supplied for charger. Using default nearest charger in the map")
            self.charger_is_set = True
        # Update position
        with self._lock:
            if (self.on_waypoint is not None): # if robot is on a waypoint
                self.update_handle.update_current_waypoint(
                    self.on_waypoint, self.position[2])
            elif (self.on_lane is not None): # if robot is on a lane
                # We only keep track of the forward lane of the robot. However, when
                # calling this update it is recommended to also pass in the reverse
                # lane so that the planner does not assume the robot can only head
                # forwards. This would be helpful when the robot is still rotating on a waypoint.
                forward_lane = self.graph.get_lane(self.on_lane)
                entry_index = forward_lane.entry.waypoint_index
                exit_index = forward_lane.exit.waypoint_index
                reverse_lane = self.graph.lane_from(exit_index, entry_index)
                lane_indices = [self.on_lane]
                if reverse_lane is not None: # Unidirectional graph
                    lane_indices.append(reverse_lane.index)
                self.update_handle.update_current_lanes(
                    self.position, lane_indices)
            elif (self.target_waypoint is not None and self.target_waypoint.graph_index is not None): # if robot is merging into a waypoint
                self.update_handle.update_off_grid_position(
                    self.position, self.target_waypoint.graph_index)
            elif (self.dock_waypoint_index is not None):
                self.update_handle.update_off_grid_position(
                    self.position, self.dock_waypoint_index)
            else: # if robot is lost
                self.update_handle.update_lost_position(
                    self.map_name, self.position)

    def get_remaining_waypoints(self, waypoints:list):
        '''
        The function returns a list where each element is a tuple of the index
        of the waypoint and the waypoint present in waypoints. This function
        may be modified if waypoints in a path need to be filtered.
        '''
        assert(len(waypoints) > 0)
        remaining_waypoints = []

        for i in range(len(waypoints)):
            remaining_waypoints.append((i,waypoints[i]))
        return remaining_waypoints
