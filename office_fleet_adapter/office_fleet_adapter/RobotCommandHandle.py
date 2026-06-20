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

import rclpy
import rclpy.node
from rclpy.duration import Duration

from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSReliabilityPolicy as Reliability
from rclpy.qos import qos_profile_system_default

import rmf_adapter as adpt
import rmf_adapter.plan as plan
import rmf_adapter.schedule as schedule

from rmf_fleet_msgs.msg import DockSummary, LaneRequest, ModeRequest

import numpy as np

import threading
import math
import enum
import time
import os

from datetime import timedelta


# States for RobotCommandHandle's state machine used when guiding robot along
# a new path
class RobotState(enum.IntEnum):
    IDLE = 0
    MOVING = 1


# Custom wrapper for Plan::Waypoint. We use this to modify position of
# waypoints to prevent backtracking
class PlanWaypoint:
    def __init__(self, index, wp: plan.Waypoint):
        # the index of the Plan::Waypoint in the waypoints in follow_new_path
        self.index = index
        self.position = wp.position
        self.time = wp.time
        self.graph_index = wp.graph_index
        self.approach_lanes = wp.approach_lanes


class RobotCommandHandle(adpt.RobotCommandHandle):
    def __init__(self,
                 name,
                 fleet_name,
                 config,
                 node,
                 graph,
                 vehicle_traits,
                 map_name,
                 start,
                 position,
                 charger_waypoint,
                 update_frequency,
                 lane_merge_distance,
                 adapter,
                 api):
        adpt.RobotCommandHandle.__init__(self)
        self.debug = False
        self.name = name
        self.fleet_name = fleet_name
        self.config = config
        self.node = node
        self.graph = graph
        self.vehicle_traits = vehicle_traits
        self.map_name = map_name
        # Get the index of the charger waypoint
        waypoint = self.graph.find_waypoint(charger_waypoint)
        assert waypoint, f"Charger waypoint {charger_waypoint} \
          does not exist in the navigation graph"
        self.charger_waypoint_index = waypoint.index
        self.update_frequency = update_frequency
        self.lane_merge_distance = lane_merge_distance
        self.update_handle = None  # RobotUpdateHandle
        self.battery_soc = 1.0
        self.api = api
        self.position = position  # (x,y,theta) in RMF crs (meters,radians)
        self.initialized = False
        self.state = RobotState.IDLE
        self.dock_name = ""
        # TODO(YV): Remove self.adapter. This is only being used for time point
        # comparison with Plan::Waypoint::time
        self.adapter = adapter
        self.action_execution = None

        self.requested_waypoints = []  # RMF Plan waypoints
        self.remaining_waypoints = []
        self.docks = {}

        # RMF location trackers
        self.last_known_lane_index = None
        self.last_known_waypoint_index = None
        self.last_replan_time = None
        # if robot is waiting at a waypoint. This is a Graph::Waypoint index
        self.on_waypoint = None
        # if robot is travelling on a lane. This is a Graph::Lane index
        self.on_lane = None
        self.target_waypoint = None  # this is a Plan::Waypoint
        # The graph index of the waypoint the robot is currently docking into
        self.dock_waypoint_index = None
        # The graph index of the waypoint the robot starts or ends an action
        self.action_waypoint_index = None
        self.current_cmd_id = 0
        self.started_action = False

        # Auto lane closure for obstacle avoidance
        self._auto_closed_lanes = {}  # lane_index -> time.time() when closed
        self._lane_request_pub = self.node.create_publisher(
            LaneRequest, 'lane_closure_requests',
            qos_profile=qos_profile_system_default)
        # Track whether we are waiting for RMF to respond with a new path
        # after triggering a replan. Prevents repeated replan calls.
        self._awaiting_rmf_response = False
        self._awaiting_rmf_since = None  # timestamp when awaiting started

        # File logger for obstacle avoidance diagnostics
        _log_dir = os.path.join(os.path.expanduser('~'), 'ros_ws',
                                'experiment_results')
        os.makedirs(_log_dir, exist_ok=True)
        self._avoid_log_path = os.path.join(_log_dir, 'avoid_debug.log')
        self._avoid_log_file = open(self._avoid_log_path, 'a')
        self._log("[AVOID] ===== RobotCommandHandle initialized =====")

        # Threading variables
        self._lock = threading.Lock()
        self._follow_path_thread = None
        self._quit_path_event = threading.Event()
        self._dock_thread = None
        self._quit_dock_event = threading.Event()
        self._stopping_thread = None
        self._quit_stopping_event = threading.Event()

        self.node.get_logger().info(
            f"The robot is starting at: [{self.position[0]:.2f}, "
            f"{self.position[1]:.2f}, {self.position[2]:.2f}]")

        # Update tracking variables
        if start.lane is not None:  # If the robot is on a lane
            self.last_known_lane_index = start.lane
            self.on_lane = start.lane
            self.last_known_waypoint_index = start.waypoint
        else:  # Otherwise, the robot is on a waypoint
            self.last_known_waypoint_index = start.waypoint
            self.on_waypoint = start.waypoint

        transient_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=1,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL)

        self.node.create_subscription(
            DockSummary,
            'dock_summary',
            self.dock_summary_cb,
            qos_profile=transient_qos)

        self.node.create_subscription(
            ModeRequest,
            'action_execution_notice',
            self.mode_request_cb,
            qos_profile=qos_profile_system_default)

        self.update_thread = threading.Thread(target=self.update)
        self.update_thread.start()

        self.initialized = True

    def next_cmd_id(self):
        self.current_cmd_id = self.current_cmd_id + 1
        if self.debug:
            print(f'Issuing cmd_id for {self.name}: {self.current_cmd_id}')
        return self.current_cmd_id

    def sleep_for(self, seconds):
        goal_time =\
          self.node.get_clock().now() + Duration(nanoseconds=1e9*seconds)
        while (self.node.get_clock().now() <= goal_time):
            time.sleep(0.001)

    def wait_on(self, event: threading.Event, seconds):
        goal_time = (
            self.node.get_clock().now() + Duration(nanoseconds=1e9*seconds)
        )
        while self.node.get_clock().now() <= goal_time:
            if event.wait(0.001):
                return True
        return False

    def clear(self):
        self.requested_waypoints = []
        self.remaining_waypoints = []
        self.state = RobotState.IDLE

    def interrupt(self):
        if self.debug:
            print(
                f'Interrupting {self.name} '
                f'(latest cmd_id is {self.current_cmd_id})'
            )
        self._quit_dock_event.set()
        self._quit_path_event.set()
        self._quit_stopping_event.set()

        if self._follow_path_thread is not None:
            if self._follow_path_thread.is_alive():
                self._follow_path_thread.join()

        if self._dock_thread is not None:
            if self._dock_thread.is_alive():
                self._dock_thread.join()

        if self._stopping_thread is not None:
            if self._stopping_thread.is_alive():
                self._stopping_thread.join()

    def stop(self):
        if self.debug:
            plan_id = self.update_handle.unstable_current_plan_id()
            print(f'stop for {self.name} with PlanId {plan_id}')

        self.interrupt()
        # Stop the robot. Tracking variables should remain unchanged.
        with self._lock:
            self._quit_stopping_event.clear()

            def _stop():
                while not self._quit_stopping_event.is_set():
                    self.node.get_logger().info(
                        f"Requesting {self.name} to stop..."
                    )
                    if self.api.stop(self.name, self.next_cmd_id()):
                        break
                    self._quit_stopping_event.wait(0.1)

            self._stopping_thread = threading.Thread(target=_stop)
            self._stopping_thread.start()

    def replan(self, close_lane=False):
        self._log(
            f"[AVOID] {self.name} replan() called, close_lane={close_lane}, "
            f"state={self.state}, on_waypoint={self.on_waypoint}, "
            f"on_lane={self.on_lane}")
        # Lane closure must happen IMMEDIATELY regardless of cooldown.
        if close_lane:
            self._close_blocked_lane()

        if self.update_handle is not None:
            now = self.adapter.now()
            # 15-second cooldown for normal replans
            if self.last_replan_time is not None and not close_lane:
                elapsed = (now - self.last_replan_time).total_seconds()
                if elapsed < 15.0:
                    self._log(
                        f"[AVOID] {self.name} RMF replan BLOCKED by "
                        f"cooldown ({elapsed:.1f}s < 15s)")
                    return
            self.last_replan_time = now
            self._awaiting_rmf_since = now
            self.update_handle.replan()
            self._awaiting_rmf_response = True
            self._log(
                f'[AVOID] {self.name} RMF replan() called successfully, '
                f'awaiting response')
        else:
            self._log(
                f"[AVOID] {self.name} update_handle is None, cannot replan")

    def _close_blocked_lane(self):
        """Close the lane that is blocked by an obstacle so RMF can find
        an alternative route around it."""
        current_lane = self.get_current_lane()
        lanes_to_close = []

        self._log(
            f"[AVOID] {self.name} _close_blocked_lane(): "
            f"current_lane={current_lane}, "
            f"target_waypoint={self.target_waypoint}, "
            f"remaining_wps={len(self.remaining_waypoints)}, "
            f"tracked_on_lane={self.on_lane}")

        if current_lane is not None:
            lanes_to_close = [current_lane]
            self._log(
                f"[AVOID] {self.name} Using current_lane={current_lane}")
        elif (self.target_waypoint is not None
              and self.target_waypoint.approach_lanes):
            # Robot is on a waypoint, close approach lanes to target
            lanes_to_close = list(self.target_waypoint.approach_lanes)
            self._log(
                f"[AVOID] {self.name} Using approach_lanes="
                f"{lanes_to_close} (on waypoint)")
        elif self.on_lane is not None:
            # Fallback: use the lane tracked by the state machine.
            # This handles cases where get_current_lane() projection fails
            # (robot at a waypoint) and the target PlanWaypoint has no
            # approach_lanes populated.
            lanes_to_close = [self.on_lane]
            self._log(
                f"[AVOID] {self.name} Using tracked on_lane="
                f"{self.on_lane} (fallback)")
        else:
            self._log(
                f"[AVOID] {self.name} No lane info available anywhere")
            return

        if not lanes_to_close:
            self._log(
                f"[AVOID] {self.name} Cannot determine blocked lane")
            return

        # Only close lanes that aren't already auto-closed
        new_closures = []
        for lane_idx in lanes_to_close:
            if lane_idx not in self._auto_closed_lanes:
                new_closures.append(lane_idx)
                self._auto_closed_lanes[lane_idx] = time.time()

        if new_closures:
            msg = LaneRequest()
            msg.fleet_name = self.fleet_name
            msg.close_lanes = new_closures
            msg.open_lanes = []
            self._lane_request_pub.publish(msg)
            self._log(
                f"[AVOID] Auto-closing lanes {new_closures} for {self.name}"
            )
        else:
            self._log(
                f"[AVOID] {self.name} Lanes {lanes_to_close} already closed")

    def _reopen_auto_closed_lanes(self):
        """Reopen lanes that were auto-closed due to obstacles.

        During obstacle avoidance, lanes must stay closed until the robot
        has actually moved away from the obstacle. Reopening them in
        follow_new_path would defeat the purpose of closing them.
        """
        if not self._auto_closed_lanes:
            return

        # Only reopen if the robot is NOT currently stuck at an obstacle.
        # If requires_replan returns True, the robot is still in
        # MODE_WAITING and we need to keep lanes closed.
        try:
            still_blocked = self.api.requires_replan(self.name)
        except Exception:
            still_blocked = False

        if still_blocked:
            self._log(
                f"[AVOID] {self.name} Keeping lanes "
                f"{list(self._auto_closed_lanes.keys())} CLOSED "
                f"(robot still blocked by obstacle)")
            return

        lanes_to_open = list(self._auto_closed_lanes.keys())
        self._auto_closed_lanes.clear()

        msg = LaneRequest()
        msg.fleet_name = self.fleet_name
        msg.close_lanes = []
        msg.open_lanes = lanes_to_open
        self._lane_request_pub.publish(msg)
        self._log(
            f"[AVOID] Reopening auto-closed lanes {lanes_to_open} "
            f"for {self.name} (obstacle cleared)")

    def _check_auto_reopen_lanes(self):
        """Safety timeout: reopen auto-closed lanes after 120 seconds
        to avoid permanently blocking routes for other robots."""
        now = time.time()
        lanes_to_reopen = []
        for lane_idx, close_time in list(self._auto_closed_lanes.items()):
            if now - close_time > 120.0:
                lanes_to_reopen.append(lane_idx)

        if lanes_to_reopen:
            for lane_idx in lanes_to_reopen:
                del self._auto_closed_lanes[lane_idx]
            msg = LaneRequest()
            msg.fleet_name = self.fleet_name
            msg.close_lanes = []
            msg.open_lanes = lanes_to_reopen
            self._lane_request_pub.publish(msg)
            self.node.get_logger().info(
                f"Safety timeout: reopening lanes {lanes_to_reopen} "
                f"for {self.name}")

    def _log(self, msg):
        """Write a timestamped message to the avoid debug log file
        AND output it via ROS logger."""
        ts = time.strftime('%H:%M:%S', time.localtime())
        line = f"[{ts}] {msg}\n"
        self._avoid_log_file.write(line)
        self._avoid_log_file.flush()
        if msg.startswith('[AVOID]'):
            self.node.get_logger().info(msg)
        elif 'Cannot' in msg or 'warn' in msg.lower():
            self.node.get_logger().warn(msg)
        else:
            self.node.get_logger().info(msg)

    def follow_new_path(
            self,
            waypoints,
            next_arrival_estimator,
            path_finished_callback):
        if self.debug:
            plan_id = self.update_handle.unstable_current_plan_id()
            print(f'follow_new_path for {self.name} with PlanId {plan_id}')

        # CRITICAL FIX: During obstacle avoidance, do NOT call interrupt().
        # interrupt() kills the _follow_path thread, which means the robot
        # never gets to execute any path. RMF sends new paths async after
        # each replan(), creating an infinite cycle:
        #   replan → RMF returns path → follow_new_path → interrupt → kill
        #   → requires_replan still True → replan → repeat
        #
        # Instead, during obstacle avoidance we do a "soft update":
        # update waypoints in-place and let the running _follow_path loop
        # pick up changes on its next iteration.
        in_avoidance = bool(self._auto_closed_lanes)

        with self._lock:
            if in_avoidance and self._follow_path_thread is not None \
                    and self._follow_path_thread.is_alive():
                # Soft update: update waypoints in-place, reset navigation
                # state so the running _follow_path picks up new targets.
                # Clear awaiting flag since we received a new path.
                self._awaiting_rmf_response = False
                self._awaiting_rmf_since = None
                self._log(
                    f"[AVOID] {self.name} follow_new_path: soft update "
                    f"(avoidance active, skipping interrupt+restart)")
                self.remaining_waypoints = self.filter_waypoints(waypoints)
                self.state = RobotState.IDLE
                wp_count = len(waypoints) if waypoints else 0
                self.node.get_logger().info(
                    f"Received new path for {self.name} "
                    f"(raw_wps={wp_count}, soft-update)")
                return

            # Normal path: full interrupt + restart
            self.interrupt()
            self._follow_path_thread = None
            self._quit_path_event.clear()
            self.clear()

            # NOTE: Do NOT reopen auto-closed lanes here.
            # During obstacle avoidance, lanes must stay closed until the
            # robot has actually completed its path or the safety timeout
            # expires. Reopening here creates a race condition where RMF's
            # new plan arrives while the robot is still blocked.

            wp_count = len(waypoints) if waypoints else 0
            self.node.get_logger().info(
                f"Received new path for {self.name} "
                f"(raw_wps={wp_count})")

            self.remaining_waypoints = self.filter_waypoints(waypoints)
            assert next_arrival_estimator is not None
            assert path_finished_callback is not None

            def _follow_path():
                target_pose = None
                path_index = 0
                while self.remaining_waypoints \
                        or self.state == RobotState.MOVING:
                    # Save the current_cmd_id before checking if we need to
                    # abort. We should always be told to abort before the
                    # current_cmd_id gets modified, so whatever the value of
                    # current_cmd_id is before being told to abort will be the
                    # value that we want. If we are saving the wrong value
                    # here, then the next thing we will be told to do is abort.
                    cmd_id = self.current_cmd_id
                    # Check if we need to abort
                    if self._quit_path_event.is_set():
                        self.node.get_logger().info(
                            f"[{self.name}] aborting path request"
                        )
                        return
                    # State machine
                    if self.state == RobotState.IDLE or target_pose is None:
                        # Assign the next waypoint
                        self.target_waypoint = self.remaining_waypoints[0]
                        path_index = self.remaining_waypoints[0].index
                        # Move robot to next waypoint
                        target_pose = self.target_waypoint.position
                        [x, y] = target_pose[:2]
                        theta = target_pose[2]
                        speed_limit = \
                            self.get_speed_limit(self.target_waypoint)
                        response = self.api.navigate(
                            self.name,
                            self.next_cmd_id(),
                            [x, y, theta],
                            self.map_name,
                            speed_limit
                        )

                        if response:
                            self.remaining_waypoints = \
                                self.remaining_waypoints[1:]
                            self.state = RobotState.MOVING
                        else:
                            self.node.get_logger().info(
                                f"Robot {self.name} failed to request "
                                f"navigation to "
                                f"[{x:.0f}, {y:.0f}, {theta:.0f}]."
                                f"Retrying...")
                            self._quit_path_event.wait(0.1)

                    elif self.state == RobotState.MOVING:
                        _replan_needed = self.api.requires_replan(self.name)
                        if _replan_needed:
                            # Check if RMF timeout expired (no response after 15s)
                            if self._awaiting_rmf_response:
                                if self._awaiting_rmf_since is not None:
                                    elapsed = (
                                        self.adapter.now() -
                                        self._awaiting_rmf_since
                                    ).total_seconds()
                                    if elapsed >= 15.0:
                                        self._log(
                                            f"[AVOID] {self.name} RMF timeout "
                                            f"({elapsed:.1f}s), retrying replan")
                                        self._awaiting_rmf_response = False
                                    else:
                                        # Still waiting for RMF, skip replan
                                        self._log(
                                            f"[AVOID] {self.name} waiting for "
                                            f"RMF response ({elapsed:.1f}s "
                                            f"< 15s timeout)")
                            if not self._awaiting_rmf_response:
                                self._log(
                                    f"[AVOID] {self.name} MOVING: "
                                    f"requires_replan=True, calling replan")
                                self.replan(close_lane=True)

                        if self._quit_path_event.wait(0.1):
                            return

                        # Check if we have reached the target
                        with self._lock:
                            if self.api.navigation_completed(
                                    self.name, cmd_id):
                                self.node.get_logger().info(
                                    f"Robot [{self.name}] has reached the "
                                    f"destination for cmd_id {cmd_id}"
                                )
                                self.state = RobotState.IDLE
                                graph_index = self.target_waypoint.graph_index
                                if graph_index is not None:
                                    self.on_waypoint = graph_index
                                    self.last_known_waypoint_index = \
                                        graph_index
                                else:
                                    self.on_waypoint = None  # still on a lane
                            else:
                                # Update the lane the robot is on
                                lane = self.get_current_lane()
                                if lane is not None:
                                    self.on_waypoint = None
                                    self.on_lane = lane
                                else:
                                    # The robot may either be on the previous
                                    # waypoint or the target one
                                    if self.target_waypoint.graph_index is \
                                        not None \
                                        and self.dist(
                                            self.position, target_pose) < 0.5:
                                        self.on_waypoint =\
                                            self.target_waypoint.graph_index
                                    elif self.last_known_waypoint_index is \
                                            not None and self.dist(
                                            self.position,
                                            self.graph.get_waypoint(
                                                self.last_known_waypoint_index
                                            ).location) < 0.5:
                                        self.on_waypoint =\
                                            self.last_known_waypoint_index
                                    else:
                                        # update_off_grid()
                                        self.on_lane = None
                                        self.on_waypoint = None
                            duration = self.api.navigation_remaining_duration(
                                self.name, cmd_id
                            )

                            if path_index is not None and duration is not None:
                                next_arrival_estimator(
                                    path_index,
                                    timedelta(seconds=duration)
                                )

                if (not self.remaining_waypoints) \
                        and self.state == RobotState.IDLE:
                    path_finished_callback()
                    self._reopen_auto_closed_lanes()
                    self.node.get_logger().info(
                        f"Robot {self.name} has successfully navigated along "
                        f"requested path."
                    )

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
        self.interrupt()
        with self._lock:
            self._quit_dock_event.clear()
            self.dock_name = dock_name
            assert docking_finished_callback is not None

            # Get the waypoint that the robot is trying to dock into
            dock_waypoint = self.graph.find_waypoint(self.dock_name)
            assert(dock_waypoint)
            self.dock_waypoint_index = dock_waypoint.index

            def _dock():
                # Request the robot to start the relevant process
                cmd_id = self.next_cmd_id()
                while not self.api.start_process(
                    self.name, cmd_id, self.dock_name, self.map_name
                ):
                    self.node.get_logger().info(
                        f"Requesting robot {self.name} to dock at "
                        f"{self.dock_name}"
                    )
                    if self._quit_dock_event.wait(1.0):
                        break

                with self._lock:
                    self.on_waypoint = None
                    self.on_lane = None

                if self.dock_name not in self.docks:
                    self.node.get_logger().info(
                        f"Requested dock {self.dock_name} not found, "
                        "ignoring docking request"
                    )
                    # TODO(MXG): This should open an issue ticket for the robot
                    # to tell the operator that the robot cannot proceed
                    return

                positions = []
                for wp in self.docks[self.dock_name]:
                    positions.append([wp.x, wp.y, wp.yaw])
                self.node.get_logger().info(
                    f"Robot {self.name} is docking at {self.dock_name}..."
                )

                while not self.api.process_completed(self.name, cmd_id):
                    if len(positions) < 1:
                        break

                    traj = schedule.make_trajectory(
                        self.vehicle_traits,
                        self.adapter.now(),
                        positions
                    )
                    itinerary = schedule.Route(self.map_name, traj)
                    if self.update_handle is not None:
                        participant = \
                            self.update_handle.get_unstable_participant()
                        participant.set_itinerary([itinerary])

                    # Check if we need to abort
                    if self._quit_dock_event.wait(0.1):
                        self.node.get_logger().info("Aborting docking")
                        return

                with self._lock:
                    self.on_waypoint = self.dock_waypoint_index
                    self.dock_waypoint_index = None
                    docking_finished_callback()
                    self.node.get_logger().info(
                        f"Robot {self.name} has completed docking"
                    )

            self._dock_thread = threading.Thread(target=_dock)
            self._dock_thread.start()

    def get_position(self):
        ''' This helper function returns the live position of the robot in the
        RMF coordinate frame'''
        position = self.api.position(self.name)
        if position is not None:
            x, y = [position[0], position[1]]
            theta = position[2]
            # Wrap theta between [-pi, pi]. Else arrival estimate will
            # assume robot has to do full rotations and delay the schedule
            if theta > np.pi:
                theta = theta - (2 * np.pi)
            if theta < -np.pi:
                theta = (2 * np.pi) + theta
            return [x, y, theta]
        else:
            self.node.get_logger().error(
                "Unable to retrieve position from robot.")
            return self.position

    def get_battery_soc(self):
        battery_soc = self.api.battery_soc(self.name)
        if battery_soc is not None:
            return battery_soc
        else:
            self.node.get_logger().error(
                "Unable to retrieve battery data from robot.")
            return self.battery_soc

    def update(self):
        while rclpy.ok():
            self.position = self.get_position()
            self.battery_soc = self.get_battery_soc()
            if self.update_handle is not None:
                self.update_state()
                self._check_auto_reopen_lanes()
            sleep_duration = float(1.0/self.update_frequency)
            self.sleep_for(sleep_duration)

    def update_state(self):
        self.update_handle.update_battery_soc(self.battery_soc)
        # Update position
        with self._lock:
            if (self.on_waypoint is not None):  # if robot is on a waypoint
                self.update_handle.update_current_waypoint(
                    self.on_waypoint, self.position[2])
            elif (self.on_lane is not None):  # if robot is on a lane
                # We only keep track of the forward lane of the robot.
                # However, when calling this update it is recommended to also
                # pass in the reverse lane so that the planner does not assume
                # the robot can only head forwards. This would be helpful when
                # the robot is still rotating on a waypoint.
                forward_lane = self.graph.get_lane(self.on_lane)
                entry_index = forward_lane.entry.waypoint_index
                exit_index = forward_lane.exit.waypoint_index
                reverse_lane = self.graph.lane_from(exit_index, entry_index)
                lane_indices = [self.on_lane]
                if reverse_lane is not None:  # Unidirectional graph
                    lane_indices.append(reverse_lane.index)
                self.update_handle.update_current_lanes(
                    self.position, lane_indices)
            elif (self.dock_waypoint_index is not None):
                self.update_handle.update_off_grid_position(
                    self.position, self.dock_waypoint_index)
            # if robot is performing an action
            elif (self.action_execution is not None):
                if not self.started_action:
                    self.started_action = True
                    self.api.toggle_action(self.name, self.started_action)
                self.update_handle.update_off_grid_position(
                    self.position, self.action_waypoint_index)
            # if robot is merging into a waypoint
            elif (self.target_waypoint is not None and
                    self.target_waypoint.graph_index is not None):
                self.update_handle.update_off_grid_position(
                    self.position, self.target_waypoint.graph_index)
            else:  # if robot is lost
                self.update_handle.update_lost_position(
                    self.map_name,
                    self.position,
                    max_merge_lane_distance=self.lane_merge_distance
                )

    def get_current_lane(self):
        def projection(current_position,
                       target_position,
                       lane_entry,
                       lane_exit):
            px, py, _ = current_position
            p = np.array([px, py])
            t = np.array(target_position)
            entry = np.array(lane_entry)
            exit = np.array(lane_exit)
            return np.dot(p - t, exit - entry)

        if self.target_waypoint is None:
            return None
        approach_lanes = self.target_waypoint.approach_lanes
        # Spin on the spot
        if approach_lanes is None or len(approach_lanes) == 0:
            return None
        # Determine which lane the robot is currently on
        for lane_index in approach_lanes:
            lane = self.graph.get_lane(lane_index)
            p0 = self.graph.get_waypoint(lane.entry.waypoint_index).location
            p1 = self.graph.get_waypoint(lane.exit.waypoint_index).location
            p = self.position
            before_lane = projection(p, p0, p0, p1) < 0.0
            after_lane = projection(p, p1, p0, p1) >= 0.0
            if not before_lane and not after_lane:  # The robot is on this lane
                return lane_index
        return None

    def dist(self, A, B):
        ''' Euclidian distance between A(x,y) and B(x,y)'''
        assert(len(A) > 1)
        assert(len(B) > 1)
        return math.sqrt((A[0] - B[0])**2 + (A[1] - B[1])**2)

    def get_speed_limit(self, target_waypoint):
        approach_lane_limit = np.inf
        approach_lanes = target_waypoint.approach_lanes
        for lane_index in approach_lanes:
            lane = self.graph.get_lane(lane_index)
            lane_limit = lane.properties.speed_limit
            if lane_limit is not None:
                if lane_limit < approach_lane_limit:
                    approach_lane_limit = lane_limit
        return approach_lane_limit if approach_lane_limit != np.inf else 0.0

    def filter_waypoints(self, wps: list):
        ''' Return filtered PlanWaypoints'''

        assert(len(wps) > 0)
        p = np.array([self.position[0], self.position[1]])

        waypoints = []
        for i in range(len(wps)):
            waypoints.append(PlanWaypoint(i, wps[i]))

        # If the robot is already in the middle of two waypoints, then we can
        # truncate all the waypoints that come before it.
        begin_at_index = 0
        for i in reversed(range(len(waypoints)-1)):
            i0 = i
            i1 = i+1
            p0 = waypoints[i0].position
            p0 = np.array([p0[0], p0[1]])
            p1 = waypoints[i1].position
            p1 = np.array([p1[0], p1[1]])
            dp_lane = p1 - p0
            lane_length = np.linalg.norm(dp_lane)
            if lane_length < 1e-3:
                continue
            n_lane = dp_lane/lane_length
            p_l = p - p0
            p_l_proj = np.dot(p_l, n_lane)
            if lane_length < p_l_proj:
                # Check if the robot's position is close enough to the lane
                # endpoint to merge it
                if np.linalg.norm(p - p1) <= self.lane_merge_distance:
                    begin_at_index = i1
                    break
                # Otherwise, continue to the next lane because the robot is not
                # between the lane endpoints
                continue
            if p_l_proj < 0.0:
                # Check if the robot's position is close enough to the lane
                # start point to merge it
                if np.linalg.norm(p - p0) <= self.lane_merge_distance:
                    begin_at_index = i0
                    break
                # Otherwise, continue to the next lane because the robot is not
                # between the lane endpoints
                continue

            lane_dist = np.linalg.norm(p_l - p_l_proj*n_lane)
            if lane_dist <= self.lane_merge_distance:
                begin_at_index = i1
                break

        if begin_at_index > 0:
            del waypoints[:begin_at_index]

        return waypoints

    def complete_robot_action(self):
        with self._lock:
            if self.action_execution is None:
                return
            self.action_execution.finished()
            self.action_execution = None
            self.started_action = False
            self.api.toggle_action(self.name, self.started_action)
            self.node.get_logger().info(f"Robot {self.name} has completed the"
                                        f" action it was performing")

    def newly_closed_lanes(self, closed_lanes):
        need_to_replan = False
        current_lane = self.get_current_lane()

        if self.target_waypoint is not None and \
                self.target_waypoint.approach_lanes is not None:
            for lane_idx in self.target_waypoint.approach_lanes:
                if lane_idx in closed_lanes:
                    need_to_replan = True
                    # The robot is currently on a lane that has been closed.
                    # We take this to mean that the robot needs to reverse.
                    if lane_idx == current_lane:
                        lane = self.graph.get_lane(current_lane)

                        return_waypoint = lane.entry.waypoint_index
                        reverse_lane = \
                            self.graph.lane_from(lane.exit.waypoint_index,
                                                 lane.entry.waypoint_index)

                        with self._lock:
                            if reverse_lane:
                                # Update current lane to reverse back to
                                # start of the lane
                                self.on_lane = reverse_lane.index
                            else:
                                # Update current position and waypoint index
                                # to return to
                                self.target_waypoint = return_waypoint

        if not need_to_replan and self.target_waypoint is not None:
            # Check if the remainder of the current plan has been invalidated
            # by the lane closure
            for wp in self.remaining_waypoints:
                for lane in wp.approach_lanes:
                    if lane in closed_lanes:
                        need_to_replan = True
                        break
                if need_to_replan:
                    break

        if need_to_replan:
            self.update_handle.replan()

    def dock_summary_cb(self, msg):
        for fleet in msg.docks:
            if(fleet.fleet_name == self.fleet_name):
                for dock in fleet.params:
                    self.docks[dock.start] = dock.path

    def mode_request_cb(self, msg):
        if not msg.fleet_name or msg.fleet_name != self.fleet_name or\
                not msg.robot_name or msg.robot_name != self.name:
            return
        if msg.mode.mode == RobotState.IDLE:
            self.complete_robot_action()

