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

"""
Check command retries with an already running TurtleBot3 Gazebo/Nav2 stack.

Start the standard ROS 2 Jazzy simulation in one terminal:
    ros2 launch nav2_bringup tb3_simulation_launch.py headless:=True use_rviz:=False

In another terminal with ROS, Nav2 and RMF Python bindings sourced, run:
    python3 tools/validate_nav2_retries.py \
        --adapter-root traffic_light_adapter_template \
        --output /tmp/nav2-retries --expect fixed

Use an unmodified source checkout and --expect buggy to reproduce the failures.
The simulation must use the default spawn pose and map. It is driven back to
(-2, -0.5) before each case, and the navigation goal is cancelled on exit.
RMF instructions are deterministic inputs; live traffic negotiation is not tested.
Rejected API calls are dropped before a Nav2 action is sent. Results include API
attempts, actual Gazebo odometry, and JSON/CSV evidence. No physical robot is used.
"""
import argparse
import copy
import csv
import json
import math
from pathlib import Path
import sys
import time

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
import rmf_adapter.easy_traffic_light as etl


def main():
    """Run the requested failure scenarios and save the observed motion."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--adapter-root', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--expect', choices=['buggy', 'fixed'], required=True)
    parser.add_argument('--cases', default='pause_once,resume_once,pause_always')
    args = parser.parse_args()
    sys.path.insert(0, args.adapter_root)
    from traffic_light_adapter_template.RobotClientAPI import RobotAPI, RobotUpdateData
    from traffic_light_adapter_template.traffic_light_command_handle import (
        TrafficLightCommandHandle,
    )

    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    nav = BasicNavigator(node_name='fleet_adapter_fault_trial')
    nav.set_parameters([Parameter('use_sim_time', value=True)])
    latest = {}
    map_pose = [-2.0, -0.5, 0.0]
    trace = []
    events = []
    context = {'case': 'startup', 'phase': 'setup', 'handle': None}
    epoch = time.monotonic()

    def yaw(q):
        return math.atan2(2 * (q.w * q.z + q.x * q.y),
                          1 - 2 * (q.y * q.y + q.z * q.z))

    def odometry(message):
        p = message.pose.pose.position
        twist = message.twist.twist
        latest.update(x=p.x, y=p.y, yaw=yaw(message.pose.pose.orientation),
                      speed=math.hypot(twist.linear.x, twist.linear.y),
                      angular_speed=abs(twist.angular.z))
        handle = context['handle']
        trace.append(dict(time_s=time.monotonic() - epoch, case=context['case'],
                          phase=context['phase'], **latest,
                          paused_by_rmf=None if handle is None else handle._paused_by_rmf,
                          last_waiting_after=None if handle is None else str(
                              handle._last_waiting_after_result)))

    def localization(message):
        p = message.pose.pose
        map_pose[:] = [p.position.x, p.position.y, yaw(p.orientation)]

    nav.create_subscription(Odometry, '/odom', odometry, qos_profile_sensor_data)
    nav.create_subscription(
        PoseWithCovarianceStamped, '/amcl_pose', localization,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                   reliability=ReliabilityPolicy.RELIABLE))

    def pump(duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(nav, timeout_sec=0.02)

    def pose(x, y, heading=math.pi / 2):
        result = PoseStamped()
        result.header.frame_id = 'map'
        result.header.stamp = nav.get_clock().now().to_msg()
        result.pose.position.x = x
        result.pose.position.y = y
        result.pose.orientation.z = math.sin(heading / 2)
        result.pose.orientation.w = math.cos(heading / 2)
        return result

    def wait_for_stop():
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            pump(0.1)
            if latest.get('speed', 1.0) < 0.005 and latest.get('angular_speed', 1.0) < 0.02:
                pump(0.25)
                return
        raise RuntimeError('Gazebo odometry did not settle after goal cancellation')

    class Instructions:
        moving = etl.MovingInstruction.ContinueAtNextCheckpoint
        waiting = etl.WaitingInstruction.Resume

        def follow_new_path(self, waypoints):
            assert len(waypoints) >= 2

        def moving_from(self, checkpoint, position):
            return self.moving

        def waiting_at(self, checkpoint):
            return self.waiting

        def waiting_after(self, checkpoint, position):
            return self.waiting

        def update_idle_location(self, map_name, position):
            pass

    class Nav2API(RobotAPI):
        def __init__(self):
            super().__init__({})
            self.goal = pose(-2.0, 0.8)
            self.path = [
                {'map_name': 'map', 'x': -2.0, 'y': -0.5, 'yaw': math.pi / 2},
                {'map_name': 'map', 'x': -2.0, 'y': 0.8, 'yaw': math.pi / 2},
            ]
            self.failures = {}
            self.calls = {'pause': 0, 'resume': 0}

        def allow(self, command):
            self.calls[command] += 1
            remaining = self.failures.get(command, 0)
            allowed = remaining == 0
            if remaining > 0:
                self.failures[command] -= 1
            events.append({
                'time_s': time.monotonic() - epoch, 'case': context['case'],
                'phase': context['phase'], 'command': command,
                'injected_failure': not allowed,
            })
            return allowed

        def pause(self, robot_name):
            if not self.allow('pause'):
                return False
            if nav.result_future and not nav.result_future.done():
                future = nav.goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(nav, future, timeout_sec=5)
                return bool(future.done() and future.result().goals_canceling)
            return True

        def resume(self, robot_name):
            if not self.allow('resume'):
                return False
            self.goal.header.stamp = nav.get_clock().now().to_msg()
            return bool(nav.goToPose(self.goal))

        def get_data(self, robot_name):
            return RobotUpdateData({
                'position': list(map_pose), 'map_name': 'map',
                'current_path': copy.deepcopy(self.path), 'last_completed_checkpoint': 0,
                'is_moving': (latest.get('speed', 0) > 0.01
                              or latest.get('angular_speed', 0) > 0.03),
                'battery_soc': 1.0,
            }, robot_name)

    def reset_robot():
        print(f'Resetting Gazebo robot for {context["case"]}', flush=True)
        nav.cancelTask()
        wait_for_stop()
        assert nav.goToPose(pose(-2.0, -0.5)), 'Reset goal rejected'
        deadline = time.monotonic() + 45
        while not nav.isTaskComplete():
            if time.monotonic() > deadline:
                nav.cancelTask()
                raise RuntimeError('Reset navigation timed out')
            pump(0.05)
        assert nav.getResult() == TaskResult.SUCCEEDED, 'Reset navigation failed'
        wait_for_stop()

    def prepare():
        reset_robot()
        api = Nav2API()
        instructions = Instructions()
        handle = TrafficLightCommandHandle('tb3_test', 'turtlebot3_waffle', api, nav)
        handle.traffic_light_cb(instructions)
        context['handle'] = handle
        handle.update_state(api.get_data('turtlebot3_waffle'))
        handle.update_state(api.get_data('turtlebot3_waffle'))
        deadline = time.monotonic() + 10
        while latest.get('speed', 0.0) < 0.06:
            if time.monotonic() > deadline:
                raise RuntimeError('Nav2 failed to start translational motion')
            pump(0.1)
        return handle, api, instructions

    def window(handle, api, duration=2.5):
        start = dict(latest)
        deadline = time.monotonic() + duration
        polls = 0
        while time.monotonic() < deadline:
            handle.update_state(api.get_data('turtlebot3_waffle'))
            polls += 1
            pump(0.4)
        return {
            'polls': polls,
            'displacement_m': math.dist((start['x'], start['y']), (latest['x'], latest['y'])),
            'final_speed_m_s': latest['speed'],
        }

    results = []
    try:
        pump(1)
        assert latest, 'No Gazebo odometry received'
        if not nav.initial_pose_received:
            nav.setInitialPose(pose(-2.0, -0.5, 0.0))
        nav.waitUntilNav2Active()
        for case in args.cases.split(','):
            context.update(case=case, phase='setup', handle=None)
            handle, api, instructions = prepare()
            if case.startswith('resume'):
                nav.cancelTask()
                wait_for_stop()
                api.failures['resume'] = -1 if case == 'resume_always' else 1
            else:
                api.failures['pause'] = -1 if case == 'pause_always' else 1
                instructions.moving = etl.MovingInstruction.PauseImmediately
            before = dict(api.calls)
            context['phase'] = 'fault'
            result = dict(case=case, **window(handle, api))
            result['api_calls'] = {key: api.calls[key] - before[key] for key in api.calls}
            result['paused_by_rmf'] = handle._paused_by_rmf
            if case == 'pause_once':
                result['passed'] = (result['api_calls']['pause'] >= 2
                                    and result['final_speed_m_s'] < 0.015
                                    and handle._paused_by_rmf)
            elif case == 'resume_once':
                result['passed'] = (result['api_calls']['resume'] >= 2
                                    and result['displacement_m'] > 0.03)
            elif case == 'pause_always':
                result['passed'] = (result['api_calls']['pause'] >= 3
                                    and not handle._paused_by_rmf)
            elif case == 'resume_always':
                result['passed'] = (result['api_calls']['resume'] >= 3
                                    and handle._last_waiting_after_result
                                    != etl.WaitingInstruction.Resume)
            else:
                raise ValueError(case)
            result['passed'] = bool(result['passed'])
            results.append(result)
            print(json.dumps(result), flush=True)
            context['phase'] = 'cleanup'
            nav.cancelTask()
            wait_for_stop()
    finally:
        (output / 'results.json').write_text(json.dumps(results, indent=2))
        (output / 'events.json').write_text(json.dumps(events, indent=2))
        if trace:
            with (output / 'odometry.csv').open('w', newline='') as stream:
                writer = csv.DictWriter(stream, fieldnames=list(trace[0]))
                writer.writeheader()
                writer.writerows(trace)
        nav.cancelTask()
        pump(0.3)
        nav.destroy_node()
        rclpy.shutdown()

    assert results, 'No cases ran'
    if args.expect == 'buggy':
        assert all(not result['passed'] for result in results), results
    else:
        assert all(result['passed'] for result in results), results


if __name__ == '__main__':
    main()
