# Copyright 2022 Open Source Robotics Foundation, Inc.
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

#! /usr/bin/env python3
import sys
import math
import yaml
import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from rclpy.parameter import Parameter

from rmf_fleet_msgs.msg import RobotState, FleetState, Location, PathRequest
from rmf_fleet_msgs.msg import RobotMode


import rmf_adapter.vehicletraits as traits
import rmf_adapter.geometry as geometry
import rmf_adapter.graph as graph
import rmf_adapter.plan as plan

import numpy as np

import flask
from flask import request, jsonify
import threading

# Global parameters
named_waypoints = { "charger_ecobot40_1":[[43.78, -19.36, 1.57], [43.81, -16.93, 1.57]]}

class EcobotFleetManager(Node):
    def __init__(self, config_path, nav_path, port):
        super().__init__('ecobot_fleet_manager')
        self.port = port
        # We set use_sim_time to true always
        param = Parameter("use_sim_time", Parameter.Type.BOOL, True)
        self.set_parameters([param])
        self.config = None
        with open(config_path, "r") as f:
            self.config = yaml.safe_load(f)
        self.fleet_name = self.config["rmf_fleet"]["name"]
        robot_names = []
        self.task_queue_data = []
        self.docks = {}
        for robot_name, robot_config in self.config["robots"].items():
            robot_names.append(robot_name)
            self.map_name = robot_config["rmf_config"]["start"]["map_name"]
            for path_name, waypoints in robot_config["ecobot_config"]["docks"].items():
                self.task_queue_data.append({"name":path_name, "map_name":self.map_name, "tasks":[{"name":"PlayPathTask","start_param":{"map_name":self.map_name,"path_name":path_name}}]})
                self.docks[path_name] = waypoints
        assert(len(robot_names) > 0)
        # TODO currently this fleet manager only supports one robot. This is how
        # the ecobot manager behaves. In the future we should extend this to
        # fleet managers that actually manage fleets
        self.robot_name = robot_names[0]

        # The planner is not being used currently
        ########################################################################
        profile = traits.Profile(geometry.make_final_convex_circle(
        self.config['rmf_fleet']['profile']['footprint']),
        geometry.make_final_convex_circle(self.config['rmf_fleet']['profile']['vicinity']))
        self.vehicle_traits = traits.VehicleTraits(
            linear=traits.Limits(*self.config['rmf_fleet']['limits']['linear']),
            angular=traits.Limits(*self.config['rmf_fleet']['limits']['angular']),
            profile=profile)
        # Ecobot robots are unable to reverse
        self.vehicle_traits.differential.reversible = False
        nav_graph = graph.parse_graph(nav_path, self.vehicle_traits)
        config = plan.Configuration(nav_graph, self.vehicle_traits)
        self.planner = plan.Planner(config)
      ##########################################################################

        self.get_logger().info(f"hello i am {self.fleet_name} fleet manager for robot {self.robot_name}")

        self.create_subscription(
            RobotState,
            'robot_state',
            self.robot_state_cb,
            10)

        self.path_pub = self.create_publisher(
            PathRequest,
            'robot_path_requests',
            qos_profile=qos_profile_system_default)

        self.state = None
        self.task_id = -1

        self.app = flask.Flask('ecobot_fleet_manager')
        self.app.config["DEBUG"] = False

        @self.app.route('/gs-robot/real_time_data/position', methods=['GET'])
        def position():
            position = [self.state.location.x, self.state.location.y]
            angle = math.degrees(self.state.location.yaw)
            data = {"angle":angle,"gridPosition":{"x":position[0],"y":position[1]},"mapInfo":{"gridHeight":992,"gridWidth":992,"originX":-24.8,"originY":-24.8,"resolution":0.05000000074505806},"worldPosition":{"orientation":{"w":-0.05743089347363588,"x":0,"y":0,"z":0.9983494841361015},"position":{"x":-6.189813393986145,"y":0.3017086724551712,"z":0}}}
            return jsonify(data)

        @self.app.route('/gs-robot/data/device_status', methods=['GET'])
        def device_status():
            # We add needed fields
            # Percentage of charge in %
            # ["battery"],
            # If charging Has to be > 0, i not 0
            # ["chargerCurrent"],
            # robot is localize, will return false if not charging, None if not avail
            # ["locationStatus"]

            battery_soc = self.state.battery_percent
            # We nee dto know if we are charging --> 
            # self.state.mode.mode ==  RobotMode.MODE_CHARGING
            if self.state.mode.mode ==  RobotMode.MODE_CHARGING:
                # Arbitrary number bigger than 0
                chargerCurrent = 1.0
                locationStatus = True
            else:
                chargerCurrent = 0.0
                locationStatus = False
            
            data = {"data":{"battery":battery_soc, "chargerCurrent":chargerCurrent, "locationStatus":locationStatus}}
            return jsonify(data)

        
        
        ### TODO: WIP
        @self.app.route('/gs-robot/data/current_map', methods=['GET'])
        def current_map():
            current_map_name = self.state.location.level_name
            data = {"data":{"current_map":current_map_name}}
            return jsonify(data)
        #########################


        @self.app.route('/gs-robot/cmd/is_task_queue_finished', methods=['GET'])
        def is_task_queue_finished():
            finished = False
            if ((self.state.mode.mode == 0 or self.state.mode.mode ==1) and len(self.state.path) == 0):
                finished = True
            data ={"data":finished, "errorCode":"","msg":"successed","successed":True}
            return jsonify(data)

        @self.app.route('/gs-robot/cmd/stop_task_queue', methods=['GET'])
        def stop_task_queue():
            path_request = PathRequest()
            path_request.fleet_name = self.fleet_name
            path_request.robot_name = self.robot_name
            path_request.path = []
            self.task_id = self.task_id + 1
            path_request.task_id = str(self.task_id)
            self.path_pub.publish(path_request)
            data ={"data":"", "errorCode":"","msg":"successed","successed":True}
            return jsonify(data)

        @self.app.route('/gs-robot/cmd/set_cleaning_mode', methods=['GET'])
        def set_cleaning_mode():
            cleaning_mode = request.args.get('cleaning_mode')
            data = {"data": [],"errorCode": "","msg": "successed","successed": True}
            return jsonify(data)

        @self.app.route('/gs-robot/data/task_queues', methods=['GET'])
        def task_queues():
            map_name = request.args.get('map_name')
            data = {"data": [],"errorCode": "","msg": "successed","successed": False}
            if (map_name != self.map_name):
                return jsonify(data)
            data["data"] = self.task_queue_data
            return jsonify(data)

        @self.app.route('/gs-robot/cmd/start_task_queue', methods=['POST'])
        def start_task_queue():
            data ={"data":"", "errorCode":"","msg":"successed","successed":False}
            request_data = request.get_json()
            print(f"Request data: {request_data}")
            if (request_data["map_name"] != self.map_name or\
                len(request_data['tasks']) < 1):
                return jsonify(data)
            # Process navigation tasks
            task = request_data['tasks'][0]
            # The ecobot has two broad kind of tasks: 1) Navigation: navigate in space 2) Clean
            if (task['name'] == "NavigationTask"):
                # The ecobot has two kinds of navigation tasks: 1) To a gridPosition 2) Named waypoint
                if task['start_param'].get('destination') is not None:
                    target_x = task['start_param']['destination']['gridPosition']['x']
                    target_y = task['start_param']['destination']['gridPosition']['y']
                    target_yaw = math.radians(task['start_param']['destination']['angle'])

                    # Add some noise to the actual location the robot will navigate to
                    target_x = target_x + np.random.uniform(-0.5, 0.5)
                    target_y = target_y + np.random.uniform(-0.5, 0.5)

                    t = self.get_clock().now().to_msg()

                    path_request = PathRequest()
                    cur_x = self.state.location.x
                    cur_y = self.state.location.y
                    cur_yaw = self.state.location.yaw
                    cur_loc = self.state.location
                    path_request.path.append(cur_loc)

                    disp = self.disp([target_x, target_y], [cur_x, cur_y])
                    duration = int(disp/self.vehicle_traits.linear.nominal_velocity)
                    t.sec = t.sec + duration
                    target_loc = Location()
                    target_loc.t = t
                    target_loc.x = target_x
                    target_loc.y = target_y
                    target_loc.yaw = target_yaw
                    target_loc.level_name = self.map_name

                    path_request.fleet_name = self.fleet_name
                    path_request.robot_name = self.robot_name
                    path_request.path.append(target_loc)
                    self.task_id = self.task_id + 1
                    path_request.task_id = str(self.task_id)
                    self.path_pub.publish(path_request)
                    data["successed"] = True
                    return jsonify(data)
                elif task['start_param'].get('position_name') is not None \
                  and task['start_param'].get('map_name') == self.map_name:
                    name = task['start_param']['position_name']
                    docking_path = named_waypoints[name]

                    t = self.get_clock().now().to_msg()

                    path_request = PathRequest()
                    cur_x = self.state.location.x
                    cur_y = self.state.location.y
                    cur_yaw = self.state.location.yaw
                    cur_loc = self.state.location
                    path_request.path.append(cur_loc)
                    previous_wp = [cur_x, cur_y]
                    for wp in docking_path:
                        l = Location()
                        disp = self.disp(wp, previous_wp)
                        duration = int(disp/self.vehicle_traits.linear.nominal_velocity)
                        t.sec = t.sec + duration
                        l.t = t
                        l.x = wp[0]
                        l.y = wp[1]
                        l.yaw = wp[2]
                        l.level_name = self.map_name
                        path_request.path.append(l)
                        previous_wp = wp

                    path_request.fleet_name = self.fleet_name
                    path_request.robot_name = self.robot_name
                    self.task_id = self.task_id + 1
                    path_request.task_id = str(self.task_id)
                    self.path_pub.publish(path_request)
                    data["successed"] = True
                    return jsonify(data)
                else:
                    return jsonify(data)

            elif (task["name"] == "PlayPathTask"):
                t = self.get_clock().now().to_msg()
                path_request = PathRequest()
                cur_x = self.state.location.x
                cur_y = self.state.location.y
                cur_yaw = self.state.location.yaw
                cur_loc = self.state.location
                path_request.path.append(cur_loc)
                previous_wp = [cur_x, cur_y]
                for wp in self.docks[task["start_param"]["path_name"]]:
                    l = Location()
                    disp = self.disp(wp, previous_wp)
                    duration = int(disp/self.vehicle_traits.linear.nominal_velocity)
                    t.sec = t.sec + duration
                    l.t = t
                    l.x = wp[0]
                    l.y = wp[1]
                    l.yaw = wp[2]
                    l.level_name = self.map_name
                    path_request.path.append(l)
                    previous_wp = wp

                path_request.fleet_name = self.fleet_name
                path_request.robot_name = self.robot_name
                self.task_id = self.task_id + 1
                path_request.task_id = str(self.task_id)
                self.path_pub.publish(path_request)
                data["successed"] = True
                return jsonify(data)

            return jsonify(data)

        self.app_thread = threading.Thread(target=self.start_app)
        self.app_thread.start()

    def start_app(self):
        self.app.run(port=self.port,threaded=True)

    def robot_state_cb(self, msg):
        if (msg.name == self.robot_name):
            self.state = msg

    def disp(self, A, B):
        return math.sqrt((A[0]-B[0])**2 + (A[1]-B[1])**2)


def main(argv=sys.argv):
    rclpy.init(args=argv)
    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    parser = argparse.ArgumentParser()
    parser.add_argument('-c', '--config', required=True, help='Path to config file')
    parser.add_argument('-n', '--nav_graph', required=True, help='Path to nav graph file')
    parser.add_argument('-p', '--port', help='Port for API Server', default=8888)
    args = parser.parse_args(args_without_ros[1:])

    n = EcobotFleetManager(args.config, args.nav_graph, args.port)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main(sys.argv)
