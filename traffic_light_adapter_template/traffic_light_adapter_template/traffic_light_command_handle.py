# Copyright 2026 Open Source Robotics Foundation, Inc.
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

import threading

from rmf_adapter import Waypoint
import rmf_adapter.easy_traffic_light as etl

from .RobotClientAPI import RobotAPI


class TrafficLightCommandHandle:
    """
    One instance per robot. Lifetime managed by `fleet_adapter.py`.

    Three callbacks are passed to `Adapter.add_easy_traffic_light(...)`:
      - traffic_light_cb : fires once with the EasyTrafficLight object.
      - pause_cb         : fires on imminent traffic conflict.
      - resume_cb        : fires when the conflict has cleared.
    """

    def __init__(
        self,
        fleet_name: str,
        robot_name: str,
        robot_api: RobotAPI,
        node,
        transforms: dict = None,
    ):
        self.fleet_name = fleet_name
        self.robot_name = robot_name
        self.robot_api = robot_api
        self.node = node
        self.logger = node.get_logger()
        self.transforms = transforms or {}

        self._handler = None

        self._cached_path = None   # last-seen deduped path (what RMF sees)
        self._cached_last_cp = -1

        self._deduped_to_raw = {}  # maps RMF indices to robot indices

        self._last_error_logged = None

        # True while we've issued an emergency pause (PAUSE_IMMEDIATELY)
        # in response to PauseImmediately or pause_cb. Cleared on resume.
        self._paused_by_rmf = False

        # When set, the cp index we've told the robot to stop at via
        # pause_at_checkpoint() in response to WaitAtNextCheckpoint.
        self._gated_at_cp = None

        # Flips True once waiting_at(0) returns Resume on this path. Until
        # then we keep calling waiting_at(0).
        self._authorized_past_start = False

        self._last_moving_from_cp = None
        self._last_moving_error_cp = None

        self._last_waiting_at_result = None
        self._last_waiting_at_trigger_cp = None

        self._last_waiting_after_result = None
        self._last_waiting_after_trigger_cp = None

        self._lock = threading.Lock()

    # ==================================================================
    # Callbacks passed to Adapter.add_easy_traffic_light
    # ==================================================================
    def traffic_light_cb(self, ez_traffic_light):
        """Receive the traffic light handle once RMF is ready."""
        self.logger.info(
            f'[{self.robot_name}] traffic light handle ready')
        self._handler = ez_traffic_light

    def pause_cb(self):
        """Emergency pause from RMF (imminent conflict)."""
        with self._lock:
            self.logger.info(
                f'[{self.robot_name}] RMF requested PAUSE')
            if not self.robot_api.pause(self.robot_name):
                self.logger.warn(
                    f'[{self.robot_name}] pause command failed')
                return
            self._paused_by_rmf = True
            self._gated_at_cp = None

    def resume_cb(self):
        """Resume the robot."""
        with self._lock:
            self.logger.info(
                f'[{self.robot_name}] RMF requested RESUME')
            if not self.robot_api.resume(self.robot_name):
                self.logger.warn(
                    f'[{self.robot_name}] resume command failed')
                return
            self._paused_by_rmf = False

    # ==================================================================
    # Polling-loop interface (called per cycle from fleet_adapter's update_loop)
    # ==================================================================

    def fetch_state(self):
        """Fetch the current state snapshot."""
        return self.robot_api.get_data(self.robot_name)

    def update_state(self, data):
        if self._handler is None:
            return

        with self._lock:
            self._update_state_locked(data)

    def _update_state_locked(self, data):
        # Dedupe up front so current_path and last_completed_checkpoint share
        # one index space for the rest of this method. RMF only ever sees the
        # deduped path (follow_new_path), so the checkpoint must be translated
        # from the incoming path's raw index into the deduped index.
        data.current_path, raw_to_deduped, self._deduped_to_raw = (
            self._dedupe_path(data.current_path))
        if 0 <= data.last_completed_checkpoint < len(raw_to_deduped):
            data.last_completed_checkpoint = (
                raw_to_deduped[data.last_completed_checkpoint])

        rmf_pose = self._robot_to_rmf(data.map_name, data.position)

        # Cache the checkpoint up front, in case the robot clears
        # current_path on the same cycle.
        if data.last_completed_checkpoint >= 0:
            self._cached_last_cp = data.last_completed_checkpoint

        # Robot is in a fault. Hold the trajectory with waiting_after on
        # the cached path. If there's no cached path, just report idle.
        if data.error:
            if data.error != self._last_error_logged:
                self.logger.error(
                    f'[{self.robot_name}] Robot reports error: '
                    f'{data.error}; holding RMF trajectory')
                self._last_error_logged = data.error
            if (self._cached_path is not None
                    and self._cached_last_cp >= 0
                    and self._cached_last_cp < len(self._cached_path) - 1):
                self._handler.waiting_after(
                    self._cached_last_cp, rmf_pose)
            else:
                self._handler.update_idle_location(
                    data.map_name, rmf_pose)
            return
        # There is no error for current cycle
        if self._last_error_logged is not None:
            self.logger.info(
                f'[{self.robot_name}] Robot error cleared, back to normal')
            self._last_error_logged = None

        # The robot may have finished its path. We detect this in two
        # ways depending on how the fleet manager reports state.
        # In either case we call waiting_at(last_cp) to release RMF's
        # trajectory and mark the robot idle.

        # Finish detection 1/2: fleet manager dropped the path entirely.
        # If we had a cached path, the robot just finished it.
        if not data.current_path:
            if self._cached_path is not None:
                end_wp = len(self._cached_path) - 1
                self.logger.info(
                    f'[{self.robot_name}] path finished at '
                    f'{self._cp_info(end_wp)}, going idle')
                self._handler.waiting_at(end_wp)
            self._handler.update_idle_location(data.map_name, rmf_pose)
            self._reset_path_state()
            return

        # Finish detection 2/2: fleet manager still reports the full path,
        # but the robot has reached the last waypoint and stopped.
        last_cp = data.last_completed_checkpoint
        path_len = len(data.current_path)
        if last_cp >= path_len - 1 and not data.is_moving:
            self.logger.info(
                f'[{self.robot_name}] path finished at '
                f'{self._cp_info(last_cp)}, going idle')
            self._handler.waiting_at(last_cp)
            self._handler.update_idle_location(data.map_name, rmf_pose)
            self._reset_path_state()
            return

        # There is an active path.
        # Did the robot hand us a new path?
        if not _waypoints_equal(self._cached_path, data.current_path):
            self._cached_path = data.current_path
            self._register_new_path(self._cached_path)
            # A new path starts at last_completed = -1 in RMF's view.
            # The snapshot at the top of update_state catches it up.
            self._cached_last_cp = -1
            # we pause robot immediately on a new path start
            if not self.robot_api.pause(self.robot_name):
                self.logger.warn(
                    f'[{self.robot_name}] pause command failed '
                    f'(start-of-path gate)')
            self._paused_by_rmf = False
            self._gated_at_cp = None
            self._authorized_past_start = False
            self._reset_log_trackers()
            return

        # Path just started. Keep calling waiting_at(0) until the blockade
        # gives us Resume.
        if not self._authorized_past_start:
            self._handle_waiting_at(0)
            if (self._last_waiting_at_result
                    == etl.WaitingInstruction.Resume):
                self._authorized_past_start = True
            return

        # Robot is gated and currently stopped. Two sub-cases:
        #   a) stopped at (or arriving at) the gated cp as expected -> ask
        #      RMF via waiting_at whether we can resume.
        #   b) stopped somewhere else (e.g. obstacle ahead) -> treat as a
        #      mid-segment stop and report our actual pose via waiting_after.
        if self._gated_at_cp is not None and not data.is_moving:
            next_target = last_cp + 1
            at_gate = (
                next_target == self._gated_at_cp
                or last_cp == self._gated_at_cp
            )
            if at_gate:
                self._handle_waiting_at(self._gated_at_cp)
                if (self._last_waiting_at_result
                        == etl.WaitingInstruction.Resume):
                    self._gated_at_cp = None
            else:
                self._handle_waiting_after(last_cp, rmf_pose)
                if (self._last_waiting_after_result
                        == etl.WaitingInstruction.Resume):
                    self._gated_at_cp = None
            return

        # Vendor-side mid-segment stop (obstacle, safety pause, etc.):
        # robot reports stopped and we didn't pause/gate it for traffic.
        # waiting_after() reports our pose and asks RMF whether to yield.
        if (not data.is_moving
                and not self._paused_by_rmf
                and self._gated_at_cp is None):
            self._handle_waiting_after(last_cp, rmf_pose)
            return

        # moving_from with live pose. Handles translation, in-place
        # rotation at last_cp, and RMF-paused polling.
        self._handle_moving(last_cp, rmf_pose)

    # ==================================================================
    # RMF call wrappers
    # ==================================================================
    def _dedupe_path(self, path):
        """
        Drop consecutive coincident robot waypoints.

        Returns the deduped path and a map from robot-reported waypoint
        indices to deduped indices and vice versa.
        """
        deduped = []
        raw_to_deduped = []
        deduped_to_raw = {}
        last_pose = None
        for i, wp in enumerate(path):
            pose = (wp['x'], wp['y'], wp.get('yaw', 0.0))
            if last_pose is None or not _same_pose(pose, last_pose):
                deduped.append(wp)
                last_pose = pose
            raw_to_deduped.append(len(deduped) - 1)
            deduped_to_raw[len(deduped) - 1] = i
        return deduped, raw_to_deduped, deduped_to_raw

    def _register_new_path(self, path):
        """Translate a robot-frame path into RMF Waypoints and submit."""
        rmf_waypoints = []
        for wp in path:
            rmf_pos = self._robot_to_rmf(
                wp['map_name'],
                [wp['x'], wp['y'], wp.get('yaw', 0.0)])
            rmf_waypoints.append(Waypoint(
                wp['map_name'],
                rmf_pos,
                wp.get('mandatory_delay', 0.0),   # expected dwell at this wp (sec)
                wp.get('yield', True),            # robot may stop here if RMF asks
            ))
        if len(rmf_waypoints) < 2:
            self.logger.info(
                f'[{self.robot_name}] skipping follow_new_path: '
                f'only {len(rmf_waypoints)} waypoint(s) after dedup.')
            return
        self.logger.info(
            f'[{self.robot_name}] triggers follow_new_path with '
            f'{len(rmf_waypoints)} waypoints:')
        for i, w in enumerate(rmf_waypoints):
            p = w.position
            self.logger.info(
                f'  [{i}] map={w.map_name}, '
                f'x={p[0]:.3f}, y={p[1]:.3f}, yaw={p[2]:.3f}')
        self._handler.follow_new_path(rmf_waypoints)

    def _handle_moving(self, last_cp, rmf_pose):
        """
        Ask RMF about the slot ahead via moving_from().

        Called both when the robot is moving and when RMF has paused us
        mid-segment. moving_from() is the only call that re-checks the
        slot ahead.
        """
        # Skip moving_from if last_cp > the path length
        if (self._cached_path is not None
                and last_cp + 1 >= len(self._cached_path)):
            return

        if last_cp != self._last_moving_from_cp:
            self.logger.info(
                f'[{self.robot_name}] entered {self._cp_info(last_cp)}, '
                f'moving_from polling will run every cycle from here.')
            self._last_moving_from_cp = last_cp
        result = self._handler.moving_from(last_cp, rmf_pose)

        if result == etl.MovingInstruction.ContinueAtNextCheckpoint:
            # Slot ahead approved. Release any active pause or gate.
            if self._paused_by_rmf or self._gated_at_cp is not None:
                self.logger.info(
                    f'[{self.robot_name}] RMF: ContinueAtNextCheckpoint, '
                    f'resuming')
                if not self.robot_api.resume(self.robot_name):
                    self.logger.warn(
                        f'[{self.robot_name}] resume command failed')
                    return
                self._paused_by_rmf = False
                self._gated_at_cp = None
            return

        if result == etl.MovingInstruction.WaitAtNextCheckpoint:
            next_cp = last_cp + 1
            # Don't gate at the path's last waypoint. RMF always returns
            # WaitAtNextCheckpoint there.
            if (self._cached_path is not None
                    and next_cp >= len(self._cached_path) - 1):
                return

            # Tell the robot to stop at the next_cp.
            if self._gated_at_cp != next_cp:
                robot_cp = self._deduped_to_raw.get(next_cp, next_cp)
                self.logger.info(
                    f'[{self.robot_name}] RMF: WaitAtNextCheckpoint, '
                    f'gating at {self._cp_info(next_cp)} (robot cp {robot_cp})')
                if not self.robot_api.pause_at_checkpoint(
                        self.robot_name, robot_cp):
                    self.logger.warn(
                        f'[{self.robot_name}] pause_at_checkpoint '
                        f'command failed')
                    return
                self._gated_at_cp = next_cp
            return

        if result == etl.MovingInstruction.PauseImmediately:
            if not self._paused_by_rmf:
                self.logger.warn(
                    f'[{self.robot_name}] RMF: PauseImmediately, pausing')
                if not self.robot_api.pause(self.robot_name):
                    self.logger.warn(
                        f'[{self.robot_name}] pause command failed')
                    return
                self._paused_by_rmf = True
                self._gated_at_cp = None
            return

        if result == etl.MovingInstruction.MovingError:
            if last_cp != self._last_moving_error_cp:
                self.logger.error(
                    f'[{self.robot_name}] RMF: MovingError, will retry on next poll')
                self._last_moving_error_cp = last_cp

    def _handle_waiting_at(self, last_cp):
        if last_cp != self._last_waiting_at_trigger_cp:
            self.logger.info(
                f'[{self.robot_name}] triggers handle.waiting_at('
                f'{self._cp_info(last_cp)})')
            self._last_waiting_at_trigger_cp = last_cp
            self._last_waiting_at_result = None
        result = self._handler.waiting_at(last_cp)

        if result == etl.WaitingInstruction.Resume:
            if self._last_waiting_at_result != etl.WaitingInstruction.Resume:
                self.logger.info(
                    f'[{self.robot_name}] RMF: Resume, clear to resume from '
                    f'{self._cp_info(last_cp)}')
                if not self.robot_api.resume(self.robot_name):
                    self.logger.warn(
                        f'[{self.robot_name}] resume command failed')
                    return
            self._last_waiting_at_result = result
            return

        if result == etl.WaitingInstruction.Wait:
            if self._last_waiting_at_result != etl.WaitingInstruction.Wait:
                self.logger.info(
                    f'[{self.robot_name}] RMF: Wait at '
                    f'{self._cp_info(last_cp)}')
            self._last_waiting_at_result = result
            return  # Wait here.

        if result == etl.WaitingInstruction.WaitingError:
            self.logger.error(
                f'[{self.robot_name}] RMF: WaitingError at '
                f'{self._cp_info(last_cp)}')
            self._last_waiting_at_result = result

    def _handle_waiting_after(self, last_cp, rmf_pose):
        """Handle a vendor-side mid-segment stop (obstacle, safety, manual pause)."""
        if last_cp != self._last_waiting_after_trigger_cp:
            self.logger.info(
                f'[{self.robot_name}] triggers handle.waiting_after('
                f'{self._cp_info(last_cp)}, '
                f'pose=({rmf_pose[0]:.3f}, {rmf_pose[1]:.3f}, '
                f'{rmf_pose[2]:.3f}))')
            self._last_waiting_after_trigger_cp = last_cp
            self._last_waiting_after_result = None
        result = self._handler.waiting_after(last_cp, rmf_pose)

        if result == etl.WaitingInstruction.Resume:
            if self._last_waiting_after_result != etl.WaitingInstruction.Resume:
                self.logger.info(
                    f'[{self.robot_name}] RMF: Resume, clear to resume '
                    f'mid-segment (after {self._cp_info(last_cp)})')
                if not self.robot_api.resume(self.robot_name):
                    self.logger.warn(
                        f'[{self.robot_name}] resume command failed')
                    return
            self._last_waiting_after_result = result
            return

        if result == etl.WaitingInstruction.Wait:
            if self._last_waiting_after_result != etl.WaitingInstruction.Wait:
                self.logger.info(
                    f'[{self.robot_name}] RMF: Wait mid-segment '
                    f'(after {self._cp_info(last_cp)})')
            self._last_waiting_after_result = result
            return  # Wait here.

        if result == etl.WaitingInstruction.WaitingError:
            self.logger.error(
                f'[{self.robot_name}] RMF: WaitingError after '
                f'{self._cp_info(last_cp)}')
            self._last_waiting_after_result = result

    def _reset_path_state(self):
        self._cached_path = None
        self._cached_last_cp = -1
        self._paused_by_rmf = False
        self._gated_at_cp = None
        self._authorized_past_start = False
        self._reset_log_trackers()

    def _reset_log_trackers(self):
        self._last_moving_from_cp = None
        self._last_moving_error_cp = None
        self._last_waiting_at_result = None
        self._last_waiting_at_trigger_cp = None
        self._last_waiting_after_result = None
        self._last_waiting_after_trigger_cp = None

    def _cp_info(self, cp):
        info = f'cp {cp}'
        if (self._cached_path is not None
                and 0 <= cp < len(self._cached_path)):
            wp = self._cached_path[cp]
            info += (
                f' (x={wp["x"]:.3f}, y={wp["y"]:.3f}, '
                f'yaw={wp.get("yaw", 0.0):.3f})')
        return info

    def _robot_to_rmf(self, map_name: str, pos: list) -> list:
        """Convert [x, y, yaw] from robot frame to RMF frame."""
        tf = self.transforms.get(map_name)
        if tf is None:
            return list(pos)
        x, y = tf.transform([pos[0], pos[1]])
        return [x, y, pos[2] + tf.get_rotation()]


# =================================================================────
# Module-level helpers
# =================================================================────
def _close(a, b, tol=1e-3):
    if a is None or b is None:
        return a is b
    return abs(a - b) < tol


def _same_pose(a, b, tol=1e-3):
    """
    Return True if poses a and b coincide in x/y/yaw within tol.

    a and b are [x, y, yaw] sequences. yaw uses the same tol as x/y and
    is compared without angle wraparound, matching the rest of this module.
    """
    return (_close(a[0], b[0], tol)
            and _close(a[1], b[1], tol)
            and _close(a[2], b[2], tol))


def _waypoints_equal(a, b):
    """Return True if two waypoint lists describe the same path."""
    if a is None or b is None:
        return a is b
    if len(a) != len(b):
        return False
    for wa, wb in zip(a, b):
        if wa.get('map_name') != wb.get('map_name'):
            return False
        if not _same_pose((wa.get('x'), wa.get('y'), wa.get('yaw')),
                          (wb.get('x'), wb.get('y'), wb.get('yaw'))):
            return False
    return True
