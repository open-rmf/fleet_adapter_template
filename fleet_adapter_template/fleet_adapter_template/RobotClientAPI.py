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


"""
The RobotAPI class is a wrapper for API calls to the robot.

Here users are expected to fill up the implementations of functions which
will be used by the RobotCommandHandle. For example, if your robot has a
REST API, you will need to make http request calls to the appropriate
endpoints within these functions.

"""


class RobotAPI:
    # The constructor below accepts parameters typically required to submit
    # http requests. Users should modify the constructor as per the
    # requirements of their robot's API
    def __init__(self, config_yaml):
        self.prefix = config_yaml['prefix']
        self.user = config_yaml['user']
        self.password = config_yaml['password']
        self.timeout = 5.0
        self.debug = False

    def check_connection(self) -> bool:
        """Return True if connection to the robot API server is successful."""
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        return True

    def localize(
        self,
        robot_name: str,
        pose,
        map_name: str,
    ) -> bool:
        """Request the robot to localize on target map."""
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        return False

    def navigate(
        self,
        robot_name: str,
        pose,
        map_name: str,
        speed_limit=0.0
    ) -> bool:
        """Request the robot to navigate to pose:[x,y,theta]."""
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        return False

    def start_activity(
        self,
        robot_name: str,
        activity: str,
        label: str
    ) -> bool:
        """Request the robot to begin a process."""
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        return False

    def stop(self, robot_name: str) -> bool:
        """Request the robot to stop."""
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        return False

    def position(self, robot_name: str) -> tuple[float, float, float]:
        """Return [x, y, theta] expressed in the robot's coordinate frame."""
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        return None

    def battery_soc(self, robot_name: str) -> float:
        """Return state of charge of robot as a value between 0.0 and 1.0."""
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        return None

    def get_map_name(self, robot_name: str) -> str:
        """Return the name of the map that the robot is currently on."""
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        return None

    def is_command_completed(self) -> bool:
        """Return True if the robot has completed its last command."""
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        return False

    def get_data(self, robot_name: str):
        """Return robot update data for the specified robot."""
        map_name = self.get_map_name(robot_name)
        position = self.position(robot_name)
        battery_soc = self.battery_soc(robot_name)
        if not (map_name is None or position is None or battery_soc is None):
            return RobotUpdateData(robot_name, map_name, position, battery_soc)
        return None


class RobotUpdateData:
    """Update data for a single robot."""

    def __init__(self,
                 robot_name: str,
                 map_name: str,
                 position: list[float],
                 battery_soc: float,
                 requires_replan: bool | None = None):
        self.robot_name = robot_name
        self.position = position
        self.map_name = map_name
        self.battery_soc = battery_soc
        self.requires_replan = requires_replan
