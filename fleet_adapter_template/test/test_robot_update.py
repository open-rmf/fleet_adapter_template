# Copyright 2026 Jong-Mun Park
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

import asyncio
import importlib.util
from pathlib import Path
import sys
from unittest.mock import MagicMock, Mock, sentinel

from fleet_adapter_template.RobotClientAPI import RobotAPI, RobotUpdateData
import pytest


@pytest.fixture
def adapter_module(monkeypatch):
    """Load the adapter without requiring ROS or optional runtime dependencies."""
    for name in (
        'nudged', 'rclpy', 'rclpy.duration', 'rclpy.node', 'rclpy.parameter',
        'rmf_adapter', 'rmf_adapter.easy_full_control', 'yaml'
    ):
        monkeypatch.setitem(sys.modules, name, MagicMock())

    path = Path(__file__).parents[1] / 'fleet_adapter_template' / 'fleet_adapter.py'
    spec = importlib.util.spec_from_file_location(
        'fleet_adapter_template._test_fleet_adapter', path
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def robot(adapter_module):
    """Create an adapter whose API returns the template's real update data type."""
    api = Mock(spec=RobotAPI)
    api.get_data.return_value = RobotUpdateData(
        'robot_1', 'L1', [1.2, -3.4, 0.5], 0.0
    )
    return adapter_module.RobotAdapter(
        'robot_1', sentinel.configuration, Mock(), api, Mock()
    )


async def _update_robot(adapter_module, robot):
    await adapter_module.update_robot(robot)


def test_register_robot_from_update_data(adapter_module, robot):
    """Register a robot using the map name supplied by RobotUpdateData."""
    asyncio.run(_update_robot(adapter_module, robot))

    robot.api.get_data.assert_called_once_with('robot_1')
    adapter_module.rmf_easy.RobotState.assert_called_once_with(
        'L1', [1.2, -3.4, 0.5], 0.0
    )
    robot.fleet_handle.add_robot.assert_called_once_with(
        'robot_1', adapter_module.rmf_easy.RobotState.return_value,
        sentinel.configuration, adapter_module.rmf_easy.RobotCallbacks.return_value
    )
    assert robot.update_handle is robot.fleet_handle.add_robot.return_value


def test_update_registered_robot_from_update_data(adapter_module, robot):
    """Forward a registered robot's map, position, and battery to its update handle."""
    robot.update_handle = Mock()

    asyncio.run(_update_robot(adapter_module, robot))

    adapter_module.rmf_easy.RobotState.assert_called_once_with(
        'L1', [1.2, -3.4, 0.5], 0.0
    )
    robot.update_handle.update.assert_called_once_with(
        adapter_module.rmf_easy.RobotState.return_value, None
    )
    robot.fleet_handle.add_robot.assert_not_called()


@pytest.mark.parametrize('registered', [False, True])
def test_missing_update_data_is_ignored(adapter_module, robot, registered):
    """Skip registration and state updates when the robot API has no data."""
    robot.api.get_data.return_value = None
    update_handle = Mock()
    if registered:
        robot.update_handle = update_handle

    asyncio.run(_update_robot(adapter_module, robot))

    adapter_module.rmf_easy.RobotState.assert_not_called()
    robot.fleet_handle.add_robot.assert_not_called()
    update_handle.update.assert_not_called()
