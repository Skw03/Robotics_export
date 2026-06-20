#!/usr/bin/env python3

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
"""Dispatch a cleaning."""

import argparse
import asyncio
import json
import math
import os
import sys
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy as Reliability

from rmf_task_msgs.msg import ApiRequest
from rmf_task_msgs.msg import ApiResponse
import requests
try:
    import yaml
except ImportError:
    yaml = None

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None

###############################################################################


class TaskRequester(Node):
    """Task requester."""

    def __init__(self, argv=sys.argv):
        """Initialize a task requester."""
        super().__init__('task_requester')
        parser = argparse.ArgumentParser()
        parser.add_argument(
            '-cs',
            '--clean_start',
            required=True,
            type=str,
            help='Cleaning zone',
        )
        parser.add_argument(
            '-F',
            '--fleet',
            type=str,
            help='Fleet name, should define together with robot',
        )
        parser.add_argument(
            '-R',
            '--robot',
            type=str,
            help='Robot name, should define together with fleet',
        )
        parser.add_argument(
            '-st',
            '--start_time',
            help='Start time from now in secs, default: 0',
            type=int,
            default=0,
        )
        parser.add_argument(
            '-pt',
            '--priority',
            help='Priority value for this request',
            type=int,
            default=0,
        )
        parser.add_argument(
            '-d',
            '--duration',
            help='Cleaning action duration in seconds',
            type=float,
            default=5.0,
        )
        parser.add_argument(
            '--trash-room',
            help='Waypoint where the robot should stop to dispose trash',
            type=str,
            default='patrol_D2',
        )
        parser.add_argument(
            '--trash-via',
            help='Intermediate waypoints to use before reaching trash room',
            type=str,
            nargs='*',
            default=[],
        )
        parser.add_argument(
            '--charger',
            help='Waypoint to return to after disposing trash',
            type=str,
        )
        parser.add_argument(
            '--visual-fallback',
            action='store_true',
            help='Kept for compatibility; fallback is enabled by default',
        )
        parser.add_argument(
            '--disable-visual-fallback',
            action='store_true',
            help='Disable direct clean/dispose fallback through the fleet manager',
        )
        parser.add_argument(
            '--fleet-manager',
            help='Fleet manager base URL',
            type=str,
            default='http://127.0.0.1:22011',
        )
        parser.add_argument(
            '--arrival-threshold',
            help='Distance in meters for considering a waypoint reached',
            type=float,
            default=0.8,
        )
        parser.add_argument(
            '--fallback-timeout',
            help='Seconds to wait for each fallback navigation step',
            type=float,
            default=180.0,
        )
        parser.add_argument(
            '--use_sim_time',
            action='store_true',
            help='Use sim time, default: false',
        )
        parser.add_argument(
            '--requester',
            help='Entity that is requesting this task',
            type=str,
            default='office_tasks'
        )

        self.args = parser.parse_args(argv[1:])
        self.response = asyncio.Future()

        transient_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=1,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL,
        )

        self.pub = self.create_publisher(
            ApiRequest, 'task_api_requests', transient_qos
        )

        # enable ros sim time
        if self.args.use_sim_time:
            self.get_logger().info('Using Sim Time')
            param = Parameter('use_sim_time', Parameter.Type.BOOL, True)
            self.set_parameters([param])

        # Construct task
        msg = ApiRequest()
        msg.request_id = 'clean_' + str(uuid.uuid4())
        payload = {}
        if self.args.fleet and self.args.robot:
            self.get_logger().info("Using 'robot_task_request'")
            payload['type'] = 'robot_task_request'
            payload['robot'] = self.args.robot
            payload['fleet'] = self.args.fleet
        else:
            self.get_logger().info("Using 'dispatch_task_request'")
            payload['type'] = 'dispatch_task_request'
        request = {}

        # Set task request request time and start time
        now = self.get_clock().now().to_msg()
        now.sec = now.sec + self.args.start_time
        start_time = now.sec * 1000 + round(now.nanosec / 10**6)
        request['unix_millis_request_time'] = start_time
        request['unix_millis_earliest_start_time'] = start_time

        request['requester'] = self.args.requester

        # Define task request category
        request['category'] = 'compose'

        if self.args.fleet:
            request['fleet_name'] = self.args.fleet

        # Define task request description with cleaning zone
        description = {}  # task_description_Compose.json
        description['category'] = 'clean'
        description['phases'] = []
        activities = []

        # Send robot to start waypoint first
        activities.append(
            {'category': 'go_to_place', 'description': self.args.clean_start}
        )
        charger = self.args.charger
        if charger is None and self.args.robot:
            charger = f'{self.args.robot}_charger'
        self.charger = charger

        description['phases'].append(
            {
                'activity': {
                    'category': 'sequence',
                    'description': {'activities': activities},
                }
            }
        )

        request['description'] = description
        payload['request'] = request
        msg.json_msg = json.dumps(payload)

        def receive_response(response_msg: ApiResponse):
            if response_msg.request_id == msg.request_id:
                self.response.set_result(json.loads(response_msg.json_msg))

        self.sub = self.create_subscription(
            ApiResponse, 'task_api_responses', receive_response, 10
        )

        print(f'Json msg payload: \n{json.dumps(payload, indent=2)}')
        self.pub.publish(msg)

    def run_visual_fallback(self):
        if self.args.disable_visual_fallback or not self.args.robot:
            return

        waypoints = self.load_waypoints()
        route = [self.args.clean_start, *self.args.trash_via,
                 self.args.trash_room]
        if self.charger:
            route.append(self.charger)

        missing = [waypoint for waypoint in route if waypoint not in waypoints]
        if missing:
            print(
                'Clean fallback disabled because waypoint coordinates are '
                f'missing: {missing}'
            )
            return

        def fleet_url(path, **params):
            query = '&'.join(f'{key}={value}' for key, value in params.items())
            return f'{self.args.fleet_manager}{path}?{query}'

        def robot_position():
            url = fleet_url(
                '/open-rmf/office_demos_fm/status/',
                robot_name=self.args.robot)
            response = requests.get(url, timeout=2.0)
            data = response.json()
            if not data.get('success'):
                raise RuntimeError(data.get('msg') or response.text)
            position = data['data']['position']
            return position['x'], position['y'], position.get('yaw', 0.0)

        def distance_to(waypoint):
            x, y, _ = robot_position()
            wx, wy, _ = waypoints[waypoint]
            return math.hypot(x - wx, y - wy)

        def wait_until_at(waypoint):
            deadline = time.time() + max(self.args.fallback_timeout, 1.0)
            while time.time() < deadline:
                try:
                    distance = distance_to(waypoint)
                    if distance <= self.args.arrival_threshold:
                        return True
                    print(
                        f'Waiting for {self.args.robot} to reach '
                        f'[{waypoint}], distance={distance:.2f}m')
                except Exception as e:
                    print(f'Unable to read robot position: {e}')
                time.sleep(1.0)
            print(f'Timed out waiting for {self.args.robot} at [{waypoint}]')
            return False

        def set_trash_state(state):
            url = fleet_url(
                '/open-rmf/office_demos_fm/trash_state/',
                robot_name=self.args.robot,
                state=state)
            try:
                response = requests.post(url, timeout=2.0)
                print(f'Visual fallback trash_state={state}: '
                      f'{response.status_code} {response.text}')
            except Exception as e:
                print(f'Visual fallback failed for state={state}: {e}')

        if not wait_until_at(self.args.clean_start):
            return

        print(f'Robot {self.args.robot} starting action [clean]')
        print(f'Robot {self.args.robot} performing [clean] '
              f'for {self.args.duration:.1f}s')
        time.sleep(max(self.args.duration, 0.0))
        print(f'Setting trash visualization for [{self.args.robot}] '
              'to [on_robot]')
        set_trash_state('on_robot')

        trash_route = [*self.args.trash_via, self.args.trash_room]
        if trash_route:
            self.dispatch_go_to_places(trash_route, 'clean_dispose')
            if not wait_until_at(self.args.trash_room):
                return

        print(f'Setting trash visualization for [{self.args.robot}] '
              'to [in_bin]')
        set_trash_state('in_bin')

        if self.charger:
            self.dispatch_go_to_places([self.charger], 'clean_return')
            wait_until_at(self.charger)

    def dispatch_go_to_places(self, places, label):
        msg = ApiRequest()
        msg.request_id = f'{label}_' + str(uuid.uuid4())
        payload = {}
        if self.args.fleet and self.args.robot:
            payload['type'] = 'robot_task_request'
            payload['robot'] = self.args.robot
            payload['fleet'] = self.args.fleet
        else:
            payload['type'] = 'dispatch_task_request'

        request = {}
        now = self.get_clock().now().to_msg()
        start_time = now.sec * 1000 + round(now.nanosec / 10**6)
        request['unix_millis_request_time'] = start_time
        request['unix_millis_earliest_start_time'] = start_time
        request['requester'] = self.args.requester
        request['category'] = 'compose'
        if self.args.fleet:
            request['fleet_name'] = self.args.fleet

        activities = [
            {'category': 'go_to_place', 'description': place}
            for place in places
        ]
        request['description'] = {
            'category': label,
            'phases': [
                {
                    'activity': {
                        'category': 'sequence',
                        'description': {'activities': activities},
                    }
                }
            ],
        }
        payload['request'] = request
        msg.json_msg = json.dumps(payload)
        print(f'Json msg payload: \n{json.dumps(payload, indent=2)}')
        self.pub.publish(msg)

    def load_waypoints(self):
        waypoints = {
            'coe': (5.346484897599354, -4.976813665783051, 0.0),
            'lounge': (20.64214427644111, -3.9893052188057747, 0.0),
            'patrol_D2': (10.247854072726847, -3.09205587858002, 0.0),
            'trash_room': (16.507715078359277, -0.8042220166379742, 0.0),
            'main_door_exit': (13.12998300807821, -2.1163737279947784, 0.0),
            'corridor_to_trash': (13.12998300807821, -0.8042220166379742, 0.0),
            'tinyRobot1_charger': (
                10.433053704916215, -5.5750955876973505, 0.0),
            'tinyRobot2_charger': (
                20.423692180237488, -5.312098057266895, 0.0),
        }

        graph_path = self.find_nav_graph()
        if yaml is None or graph_path is None:
            return waypoints

        try:
            with open(graph_path, 'r') as f:
                graph = yaml.safe_load(f)
            vertices = graph['levels']['L1']['vertices']
            for vertex in vertices:
                if len(vertex) < 3:
                    continue
                params = vertex[2] or {}
                name = params.get('name')
                if name:
                    waypoints[name] = (float(vertex[0]), float(vertex[1]), 0.0)
        except Exception as e:
            print(f'Unable to load nav graph [{graph_path}]: {e}')
        return waypoints

    def find_nav_graph(self):
        candidates = []
        if get_package_share_directory is not None:
            try:
                candidates.append(os.path.join(
                    get_package_share_directory('office_maps'),
                    'maps',
                    'office',
                    'nav_graphs',
                    '0.yaml'))
            except Exception:
                pass
            try:
                candidates.append(os.path.join(
                    get_package_share_directory('office_maps'),
                    'office',
                    'nav_graphs',
                    '0.yaml'))
            except Exception:
                pass

        here = os.path.dirname(os.path.abspath(__file__))
        candidates.extend([
            os.path.abspath(os.path.join(
                here, '..', '..', 'office_maps', 'generated_maps', 'office',
                'nav_graphs', '0.yaml')),
            os.path.abspath(os.path.join(
                here, '..', '..', '..', 'office_maps', 'generated_maps',
                'office', 'nav_graphs', '0.yaml')),
        ])

        for path in candidates:
            if os.path.exists(path):
                return path
        return None


###############################################################################


def main(argv=sys.argv):
    """Dispatch a cleaning."""
    rclpy.init(args=sys.argv)
    args_without_ros = rclpy.utilities.remove_ros_args(sys.argv)

    task_requester = TaskRequester(args_without_ros)
    rclpy.spin_until_future_complete(
        task_requester, task_requester.response, timeout_sec=30.0
    )
    if task_requester.response.done():
        print(f'Got response: \n{task_requester.response.result()}')
    else:
        print('Did not get a response')
    task_requester.run_visual_fallback()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
