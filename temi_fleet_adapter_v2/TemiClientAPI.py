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


'''
    The RobotAPI class is a wrapper for API calls to the robot. Here users
    are expected to fill up the implementations of functions which will be used
    by the RobotCommandHandle. For example, if your robot has a REST API, you
    will need to make http request calls to the appropriate endpoints within
    these functions.
'''

import asyncio
import re
from temi import Temi


class TemiAPI:

    # The constructor below accepts parameters typically required to submit
    # http requests. Users should modify the constructor as per the
    # requirements of their robot's API

    def __init__(self, prefix: str):
        self.prefix = prefix
        self.connected = False
        self.temi = None
        # Test connectivity
        connected = asyncio.get_event_loop().run_until_complete(self.check_connection())
        if connected:
            print("Successfully able to query API server")
            self.connected = True
        else:
            print("Unable to query API server")

    async def check_connection(self):
        """
        Return True if connection to the robot API server is successful
        """

        self.temi = Temi(self.prefix)
        await self.temi.connect()
        await self.temi.speak(sentence='Hello!').run()

        return True

    async def getPosition(self, robot_name: str):
        """
        Return [x, y, theta] expressed in the robot's coordinate frame or
            None if any errors are encountered
        """
        try:
            skidjoy_response = await self.temi.skidJoy(x=float(0.001), y=float(0.001)).run()

            # Example of split_response:
            # ['Position(x', '1.3517', '', 'y', '-5.1696', '', 'yaw', '3.0173', '', 'tiltAngle', '51)']
            split_response = re.split('[= ,]', skidjoy_response.get('position'))

            return [float(split_response[1]), float(split_response[4]), float(split_response[7])]

        except Exception as e:
            print(f"An error has occurred when getting robot position: {e}")
            return None

    async def navigate(self, robot_name: str, pose, map_name: str):
        """
        Request the robot to navigate to pose:[x,y,theta] where x, y and
        theta are in the robot's coordinate convention. This function
        should return True if the robot has accepted the request,
        else False
        """
        try:
            await self.temi.gotoPosition(x=pose[0], y=pose[1], yaw=pose[2], tiltAngle=22).run()
            return True
        except Exception as e:
            print(f"An error has occurred during navigation: {e}")
            return False

    async def stop(self, robot_name: str):
        """
        Command the robot to stop.
        Return True if robot has successfully stopped. Else False
        """
        try:
            await self.temi.stopMovement().run()
            return True
        except Exception as e:
            print(f"An error has occurred when stopping robot movement: {e}")
            return False

    async def docking_completed(self, robot_name: str):
        """
        Check if robot reached home base.
        Return True if robot has successfully docked. Else False
        """
        try:
            return await self.temi.checkIfDockingCompleted().run()
        except Exception as e:
            print(f"An error has occurred when stopping robot movement: {e}")
            return False

    async def navigation_remaining_duration(self, robot_name: str):
        """
        Return the number of seconds remaining for the robot to reach its
        destination
        """
        try:
            duration_response = await self.temi.getRemainingDuration().run()
            return float(duration_response.get('duration'))
        except Exception as e:
            print(f"An error has occurred when retrieving remaining robot duration: {e}")

    async def navigation_completed(self, robot_name: str):
        """
        Return True if the robot has successfully completed its previous
        navigation request. Else False.
        """
        try:
            response = await self.temi.checkIfNavigationCompleted().run()
            return response.get('completedNavigation') == 'True'
        except Exception as e:
            print(f"An error has occurred when checking : {e}")
            return False

    # def start_process(self, robot_name: str, process: str, map_name: str):
    #     """
    #     Request the robot to begin a process. This is specific to the robot
    #     and the use case. For example, load/unload a cart for Deliverybot
    #     or begin cleaning a zone for a cleaning robot.
    #     Return True if the robot has accepted the request, else False
    #     """
    #     # ------------------------ #
    #     # IMPLEMENT YOUR CODE HERE #
    #     # ------------------------ #
    #     return False

    # def process_completed(self, robot_name: str):
    #     ''' Return True if the robot has successfully completed its previous
    #         process request. Else False.'''
    #     # ------------------------ #
    #     # IMPLEMENT YOUR CODE HERE #
    #     # ------------------------ #
    #     return False
    #
    async def battery_soc(self, robot_name: str):
        """
        Return the state of charge of the robot as a value between 0.0
        and 1.0. Else return None if any errors are encountered
        """
        try:
            # Example of battery_data.get('batteryData'):
            # "BatteryData(level=95, isCharging=false)"
            battery_data = await self.temi.getBatteryData().run()

            # Example of response:
            # ['BatteryData(level', '95', '', 'isCharging', 'false)']
            split_response = re.split('[= ,]', battery_data.get('batteryData'))

            battery_level = float(split_response[1])
            return battery_level / 100.0

        except Exception as e:
            print(f"An error has occurred when obtaining the battery level: {e}")
            return None