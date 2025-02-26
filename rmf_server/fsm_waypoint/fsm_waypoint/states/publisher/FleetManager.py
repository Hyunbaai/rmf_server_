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

import sys
import math
import yaml
import json
import time
import copy
import argparse

# ros2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSReliabilityPolicy as Reliability
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy

# rmf
from rmf_fleet_msgs.msg import RobotState, Location, PathRequest, \
    DockSummary, RobotMode
from rmf_task_msgs.msg import ApiRequest, ApiResponse

# fsm
import smach

# utils
from fsm_waypoint.utils import debug, info, warning, error, critical
from pyproj import Transformer
import threading
import uuid

# websocket
import websocket
import ssl


# ------------------------------------------------------------------------------
# Fleet Manager
# ------------------------------------------------------------------------------
class State:
    def __init__(self, state: RobotState = None, destination: Location = None):
        self.state = state
        self.destination = destination
        self.last_path_request = None
        self.last_completed_request = None
        self.mode_teleop = False
        self.svy_transformer = Transformer.from_crs('EPSG:4326', 'EPSG:3414')
        self.gps_pos = [0, 0]

    def gps_to_xy(self, gps_json: dict):
        svy21_xy = \
            self.svy_transformer.transform(gps_json['lat'], gps_json['lon'])
        self.gps_pos[0] = svy21_xy[1]
        self.gps_pos[1] = svy21_xy[0]

    def is_expected_task_id(self, task_id):
        if self.last_path_request is not None:
            if task_id != self.last_path_request.task_id:
                return False
        return True


# config_file:=/home/bcc/Works1/rmf_demos_robot/rmf_demos/config/turtlebot3_world/turtlebot3_world_config.yaml
class FleetManager(smach.State):
    def __init__(self, node, config, timeout=None):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                             input_keys=['blackboard'],
                             output_keys=['blackboard']
                             )
        self.debug = False
        self.config = config
        self.node = node
        self.timeout = timeout
        self.fleet_name = self.config['rmf_fleet']['name']
        mgr_config = self.config['fleet_manager']

        self.url = 'wss://hp8idwb108.execute-api.ap-northeast-2.amazonaws.com/dev'
        self.auth = '66fc47ba2013b66a223b5c8a6fac0926'
        # self.connection_id = ''
        self.ws = None
        self.ws_thread = None
        self.connection_open = False  # WebSocket 연결 상태
        self.connection_closed = False  # 상태 전이를 위한 플래그
        self.sub_list = []  # 구독된 토픽들 목록
        self.robots = {}  # Map robot name to state

        for robot_name, _ in self.config['rmf_fleet']['robots'].items():
            self.robots[robot_name] = State()
        assert(len(self.robots) > 0)

        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.pub = self.node.create_publisher(ApiRequest, 'task_api_requests', transient_qos)
        self.sub = None
        self.pending_requests = {}

    def execute(self, userdata):
        self.sub_list = [] # reset
        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.sub = self.node.create_subscription(
            ApiResponse,
            'task_api_responses',
            self.receive_response,
            qos_profile=transient_qos)

        self.sub_list.append(self.sub)

        start_time = time.time()
        info("FleetManager state executing...")
        self.connection_open = False  # reset
        self.connection_closed = False  # reset
        if self.robots:
            # 첫 번째 키를 robot_name에 저장
            robot_name = next(iter(self.robots))
            print(f"First robot_name: {robot_name}")
        else:
            raise ValueError("self.robots is empty, no robot_name available")

        # 헤더 생성
        headers = {
            "Auth": self.auth,  # 인증 토큰
        }
        self.connect(headers)

        outcome = 'succeeded' # because timeout not mandatory

        try:
            while rclpy.ok():
                if self.preempt_requested():
                    self.service_preempt()
                    self.cleanup()  # 구독 해제 및 WebSocket 종료
                    outcome = 'preempted'
                    break

                if self.connection_closed:
                    self.cleanup()  # 구독 해제 및 WebSocket 종료
                    # outcome = 'aborted'
                    # break
                    self.connection_open = False  # reset
                    self.connection_closed = False  # reset
                    self.connect(headers)  # 다시 연결 시도
                    time.sleep(1)

                time.sleep(0.1)  # Adjust the interval as needed

                if self.timeout:
                    if time.time() - start_time > self.timeout:
                        outcome = 'succeeded'
                        break
        except KeyboardInterrupt:
            error("Interrupted by user, shutting down...")
            self.cleanup()
            return 'preempted'
        except Exception as e:
            error(f"An error occurred while creating waypoint: {str(e)}")
            self.cleanup()
            return 'aborted'

        self.cleanup()
        return outcome

    def connect(self, headers):
        self.ws = websocket.WebSocketApp(self.url,
                                         on_open=self.on_open,
                                         on_message=self.on_message,
                                         on_error=self.on_error,
                                         on_close=self.on_close,
                                         header=headers)
        self.ws_thread = threading.Thread(target=self.ws.run_forever,
                                          kwargs={"sslopt": {"cert_reqs": ssl.CERT_NONE}})
        self.ws_thread.start()

    def on_open(self, ws):
        self.connection_open = True
        info("Connection opened")

    def on_message(self, ws, message):
        info('Received message: ' + message)
        try:
            package = json.loads(message)
            if package.get('type') == 'connected':
                args = package.get('args', {})
                connection_id = args.get('connection_id')
                info(f'connection_id: {connection_id}')
                return  # 여기서 종료
            if "request" not in package:
                error(f"Invalid message format: {package}")
                return

            request = package["request"]

            if request.get("type") == "door":
                self.handle_door_request(request)
            else:
                error(f"Unknown message type: {request}")

        except json.JSONDecodeError:
            error("The received message is not in JSON format.")

    def on_error(self, ws, error):
        self.connection_open = False
        error(f"WebSocket error: {error}")
        self.connection_closed = True  # 상태 전이를 위한 플래그 설정

    def on_close(self, ws, close_status_code, close_msg):
        self.connection_open = False
        info(f"WebSocket connection closed, code: {close_status_code}, msg: {close_msg}")
        self.connection_closed = True  # 상태 전이를 위한 플래그 설정

    def cleanup(self):
        """구독 해제 및 WebSocket 종료"""
        info("Cleaning up sub_list and closing WebSocket...")
        for sub in self.sub_list:
            self.node.destroy_subscription(sub)  # 모든 구독 해제

        if self.ws:
            self.ws.close()
            if self.ws_thread and self.ws_thread.is_alive():
                self.ws_thread.join()

    def handle_door_request(self, request):
        request_id = f"patrol_{uuid.uuid4()}"  # 고유한 request_id 생성
        self.pending_requests[request_id] = request  # 요청 저장

        fleet_name = request.get("fleet_name", "turtlebot3")
        robot_name = request.get("robot_name", "tinybot1")
        patrol_location = request.get("patrol", "patrol_A1")

        msg = ApiRequest()
        msg.request_id = request_id
        payload = {
            "type": "robot_task_request",
            "robot": robot_name,
            "fleet": fleet_name,
            "request": {
                "category": "patrol",
                "description": {
                    "places": [patrol_location],
                    "rounds": 1  # 한 번 순찰
                }
            }
        }
        msg.json_msg = json.dumps(payload)

        self.pub.publish(msg)
        info(f"Published patrol request: {msg.json_msg}")

    def receive_response(self, response_msg: ApiResponse):
        request_id = response_msg.request_id
        if request_id not in self.pending_requests:
            error(f"Received unknown response: {response_msg.json_msg}")
            return

        original_request = self.pending_requests.pop(request_id)
        response_data = json.loads(response_msg.json_msg)

        response_message = {
            "response": {
                "robot_name": original_request.get("robot_name", "tinybot1"),
                "task_id": request_id,
                "type": "door",
                "value": original_request.get("value", "open"),
                "fleet_name": original_request.get("fleet_name", "fleet_001"),
                "patrol": original_request.get("patrol", "sector_5"),
                "result": response_data
            }
        }

        if self.connection_open:
            self.ws.send(json.dumps(response_message))
            info(f"Sent response: {response_message}")
        else:
            error("WebSocket is not connected. Unable to send response.")
