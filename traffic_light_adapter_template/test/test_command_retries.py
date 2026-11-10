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

from collections import Counter
from concurrent.futures import ThreadPoolExecutor
from enum import Enum
import importlib.util
from pathlib import Path
import sys
import threading
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock

import pytest
from traffic_light_adapter_template.RobotClientAPI import RobotAPI, RobotUpdateData


@pytest.fixture
def adapter_module(monkeypatch):
    """Load the real command handler with only the RMF boundary replaced."""
    etl = ModuleType('rmf_adapter.easy_traffic_light')
    etl.MovingInstruction = Enum('MovingInstruction', [
        'ContinueAtNextCheckpoint', 'WaitAtNextCheckpoint',
        'PauseImmediately', 'MovingError',
    ])
    etl.WaitingInstruction = Enum('WaitingInstruction', [
        'Resume', 'Wait', 'WaitingError',
    ])
    rmf = ModuleType('rmf_adapter')
    rmf.easy_traffic_light = etl
    rmf.Waypoint = lambda map_name, position, delay, can_yield: SimpleNamespace(
        map_name=map_name, position=position)
    monkeypatch.setitem(sys.modules, 'rmf_adapter', rmf)
    monkeypatch.setitem(sys.modules, 'rmf_adapter.easy_traffic_light', etl)
    path = (Path(__file__).parents[1] / 'traffic_light_adapter_template'
            / 'traffic_light_command_handle.py')
    spec = importlib.util.spec_from_file_location(
        'traffic_light_adapter_template._test_command_handle', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def handle(adapter_module):
    """Create a robot whose commands succeed unless a test injects failures."""
    api = Mock(spec=RobotAPI)
    api.pause.return_value = True
    api.resume.return_value = True
    api.pause_at_checkpoint.return_value = True
    result = adapter_module.TrafficLightCommandHandle('fleet', 'robot', api, Mock())
    traffic_light = Mock()
    traffic_light.moving_from.return_value = (
        adapter_module.etl.MovingInstruction.ContinueAtNextCheckpoint)
    traffic_light.waiting_at.return_value = adapter_module.etl.WaitingInstruction.Resume
    traffic_light.waiting_after.return_value = adapter_module.etl.WaitingInstruction.Resume
    result.traffic_light_cb(traffic_light)
    return result


def snapshot(moving=True, checkpoint=0):
    """Return a fresh robot snapshot because the handler normalizes its path."""
    return RobotUpdateData({
        'position': [0.0, 0.0, 0.0], 'map_name': 'L1',
        'current_path': [{'map_name': 'L1', 'x': float(x), 'y': 0.0} for x in range(3)],
        'last_completed_checkpoint': checkpoint, 'is_moving': moving,
    })


@pytest.fixture
def active_handle(handle):
    """Register a path and pass the initial stop/resume gate."""
    handle.update_state(snapshot(moving=False))
    handle.update_state(snapshot(moving=False))
    handle.robot_api.reset_mock()
    return handle


@pytest.mark.parametrize('failures', [1, 3])
def test_failed_pause_is_retried(adapter_module, active_handle, failures):
    """Retry rejected stops, then suppress duplicates after an accepted stop."""
    handle = active_handle
    handle._handler.moving_from.return_value = (
        adapter_module.etl.MovingInstruction.PauseImmediately)
    handle.robot_api.pause.side_effect = [False] * failures + [True]

    for attempt in range(failures):
        handle.update_state(snapshot())
        assert handle.robot_api.pause.call_count == attempt + 1
        assert not handle._paused_by_rmf
    handle.update_state(snapshot())
    assert handle._paused_by_rmf
    handle.update_state(snapshot(moving=False))
    assert handle.robot_api.pause.call_count == failures + 1


@pytest.mark.parametrize('failures', [1, 3])
def test_failed_checkpoint_stop_is_retried(adapter_module, active_handle, failures):
    """Keep retrying a rejected checkpoint gate until the robot accepts it."""
    handle = active_handle
    handle._handler.moving_from.return_value = (
        adapter_module.etl.MovingInstruction.WaitAtNextCheckpoint)
    handle.robot_api.pause_at_checkpoint.side_effect = [False] * failures + [True]

    for attempt in range(failures):
        handle.update_state(snapshot())
        assert handle.robot_api.pause_at_checkpoint.call_count == attempt + 1
        assert handle._gated_at_cp is None
    handle.update_state(snapshot())
    assert handle._gated_at_cp == 1
    handle.robot_api.pause_at_checkpoint.assert_called_with('robot', 1)
    handle.update_state(snapshot())
    assert handle.robot_api.pause_at_checkpoint.call_count == failures + 1


@pytest.mark.parametrize('failures', [1, 3])
def test_failed_moving_resume_preserves_pause(active_handle, failures):
    """Release an RMF pause only after the robot accepts the resume command."""
    handle = active_handle
    handle._paused_by_rmf = True
    handle.robot_api.resume.side_effect = [False] * failures + [True]

    for attempt in range(failures):
        handle.update_state(snapshot(moving=False))
        assert handle.robot_api.resume.call_count == attempt + 1
        assert handle._paused_by_rmf
    handle.update_state(snapshot(moving=False))
    assert handle.robot_api.resume.call_count == failures + 1
    assert not handle._paused_by_rmf


@pytest.mark.parametrize('failures', [1, 3])
def test_failed_start_resume_does_not_authorize_path(adapter_module, handle, failures):
    """Keep the start gate closed while the resume command is rejected."""
    handle.update_state(snapshot(moving=False))
    handle.robot_api.resume.side_effect = [False] * failures + [True]

    for attempt in range(failures):
        handle.update_state(snapshot(moving=False))
        assert handle.robot_api.resume.call_count == attempt + 1
        assert not handle._authorized_past_start
        assert handle._last_waiting_at_result != adapter_module.etl.WaitingInstruction.Resume
    handle.update_state(snapshot(moving=False))
    assert handle._authorized_past_start


@pytest.mark.parametrize('failures', [1, 3])
def test_failed_vendor_stop_resume_is_retried(adapter_module, active_handle, failures):
    """Retry a rejected resume while the robot remains stopped between waypoints."""
    handle = active_handle
    handle.robot_api.resume.side_effect = [False] * failures + [True]

    for attempt in range(failures):
        handle.update_state(snapshot(moving=False))
        assert handle.robot_api.resume.call_count == attempt + 1
        assert handle._last_waiting_after_result != adapter_module.etl.WaitingInstruction.Resume
    handle.update_state(snapshot(moving=False))
    assert handle.robot_api.resume.call_count == failures + 1
    handle.update_state(snapshot())
    assert handle.robot_api.resume.call_count == failures + 1


def test_failed_resume_keeps_checkpoint_gate(adapter_module, active_handle):
    """Do not clear a checkpoint gate before its resume command is accepted."""
    handle = active_handle
    handle._gated_at_cp = 1
    handle.robot_api.resume.side_effect = [False, True]

    handle.update_state(snapshot(moving=False, checkpoint=1))
    assert handle._gated_at_cp == 1
    handle.update_state(snapshot(moving=False, checkpoint=1))
    assert handle._gated_at_cp is None
    assert handle.robot_api.resume.call_count == 2


@pytest.mark.parametrize('command,paused', [('pause', False), ('resume', True)])
def test_failed_callback_preserves_pause_state(active_handle, command, paused):
    """Keep the previous confirmed state when a callback command is rejected."""
    handle = active_handle
    handle._paused_by_rmf = paused
    handle._gated_at_cp = 1
    method = getattr(handle.robot_api, command)
    method.side_effect = [False, True]

    callback = getattr(handle, command + '_cb')
    callback()
    assert handle._paused_by_rmf is paused
    assert handle._gated_at_cp == 1
    callback()
    assert handle._paused_by_rmf is not paused
    assert method.call_count == 2


@pytest.mark.parametrize('command', ['pause', 'resume'])
def test_shared_api_keeps_robot_command_states_independent(adapter_module, command):
    """Keep concurrent robots independent when one shared-API command is rejected."""
    api = Mock(spec=RobotAPI)
    api.pause.return_value = True
    api.resume.return_value = True
    robots = []
    for name in ('alpha', 'beta'):
        handle = adapter_module.TrafficLightCommandHandle('fleet', name, api, Mock())
        traffic_light = Mock()
        traffic_light.moving_from.return_value = (
            adapter_module.etl.MovingInstruction.ContinueAtNextCheckpoint)
        traffic_light.waiting_at.return_value = adapter_module.etl.WaitingInstruction.Resume
        traffic_light.waiting_after.return_value = adapter_module.etl.WaitingInstruction.Resume
        handle.traffic_light_cb(traffic_light)
        handle.update_state(snapshot(moving=False))
        handle.update_state(snapshot(moving=False))
        if command == 'pause':
            traffic_light.moving_from.return_value = (
                adapter_module.etl.MovingInstruction.PauseImmediately)
        else:
            handle.pause_cb()
        robots.append(handle)
    api.reset_mock()
    barrier = threading.Barrier(2)

    def first_attempt(robot_name):
        barrier.wait(timeout=2)
        return robot_name != 'alpha'

    method = getattr(api, command)
    method.side_effect = first_attempt
    with ThreadPoolExecutor(max_workers=2) as pool:
        jobs = [pool.submit(handle.update_state, snapshot(moving=command == 'pause'))
                for handle in robots]
        for job in jobs:
            job.result(timeout=5)

    assert robots[0]._paused_by_rmf is (command == 'resume')
    assert robots[1]._paused_by_rmf is (command == 'pause')
    method.side_effect = lambda robot_name: True
    for handle in robots:
        handle.update_state(snapshot(moving=command == 'pause' or handle.robot_name == 'beta'))
    assert Counter(call.args[0] for call in method.call_args_list) == {'alpha': 2, 'beta': 1}
    assert all(handle._paused_by_rmf is (command == 'pause') for handle in robots)
