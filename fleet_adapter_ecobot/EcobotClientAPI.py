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

import requests
import json
import math
from urllib.error import HTTPError

# Sample prefix: http://10.7.5.88:8080
# This class simplifies the process of sending GET/POST commands to an Ecobot
# Responses returned are deserialized dictionaries
class EcobotAPI:
    def __init__(self, prefix:str, cleaning_task_prefix="", timeout=5.0, debug=False):
        self.prefix = prefix
        self.cleaning_task_prefix = cleaning_task_prefix
        self.debug = debug
        self.timeout = timeout
        # Test connectivity
        data = self.data()
        if data is not None:
            print("[EcobotAPI] successfully able to query API server")
            self.connected = True
        else:
            print("[EcobotAPI] unable to query API server")
            self.connected = False

    def online(self):
        return self.connected

    def load_map(self, map_name: str):
        url = self.prefix + f"/gs-robot/cmd/load_map?map_name={map_name}"
        print(f"url:{url}")
        try:
            response = requests.get(url, timeout=self.timeout)
            response.raise_for_status()
            if self.debug:
                print(f"Response: {response.json()}")
            return response.json()["successed"]
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")
        return False

    def localize(self, init_point:str, map_name:str, rotate=False):
        if rotate: # The robot will rotate to improve localization
            url = self.prefix + f"/gs-robot/cmd/initialize?map_name={map_name}&init_point_name={init_point}"
        else: # The specified init point must be accurate
            url = self.prefix + f"/gs-robot/cmd/initialize_directly?map_name={map_name}&init_point_name={init_point}"
        try:
            response = requests.get(url, timeout=self.timeout)
            response.raise_for_status()
            if self.debug:
                print(f"Response: {response.json()}")
            return response.json()["successed"]
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")
        return False

    def position(self):
        ''' Returns [x, y, theta] expressed in the robot's coordinate frame or None'''
        url = self.prefix + f"/gs-robot/real_time_data/position"
        try:
            response = requests.get(url, timeout=self.timeout)
            response.raise_for_status()
            data = response.json()
            if self.debug:
                print(f"Response: {data}")
            x = data["gridPosition"]["x"]
            y = data["gridPosition"]["y"]
            angle = data["angle"]
            return [x, y, angle]
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")
        return None

    def navigate(self, pose, map_name:str):
        ''' Here pose is a list containing (x,y,theta) where x and y are in grid
            coordinate frame and theta is in degrees. This functions should
            return True if the robot received the command, else False'''
        assert(len(pose) > 2)
        url = self.prefix + f"/gs-robot/cmd/start_task_queue"
        data = {}
        data["name"] = ""
        data["loop"] = False
        data["loop_count"] = 0
        data["map_name"] = map_name # this is the name of the map as stored on the robot
        task = {"name":"NavigationTask","start_param":{"destination":{"angle":pose[2],"gridPosition":{"x":pose[0], "y":pose[1]}}}}
        data["tasks"] = [task]
        try:
            response = requests.post(url, timeout=self.timeout, json=data)
            response.raise_for_status()
            if self.debug:
                print(f"Response: {response.json()}")
            return response.json()['successed']
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
            self.connected = False
        except Exception as err:
            print(f"Other error: {err}")
        return False

    # NOTE: Unstable gaussian api 2.0. Get task status
    def __navigate(self, pose):
        assert(len(pose) > 2)
        url = self.prefix + f"/gs-robot/cmd/quick/navigate?type=2"
        data = {}
        data["destination"] = {"gridPosition": {"x": pose[0], "y": pose[1]}, "angle": pose[2]}
        try:
            response = requests.post(url, timeout=self.timeout, json=data)
            response.raise_for_status()
            if self.debug:
                print(f"Response: {response.json()}")
            return response.json()['successed']
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
            self.connected = False
        except Exception as err:
            print(f"Other error: {err}")
        return False


    def navigate_to_waypoint(self, waypoint_name, map_name):
        ''' Ask the robot to navigate to a preconfigured waypoint on a map.
            Returns True if the robot received the command'''
        url = self.prefix + f"/gs-robot/cmd/start_task_queue"
        data = {}
        data["name"] = ""
        data["loop"] = False
        data["loop_count"] = 0
        data["map_name"] = map_name # this is the name of the map as stored on the robot
        task = {"name":"NavigationTask","start_param":{"map_name":map_name, "position_name":waypoint_name}}
        data["tasks"] = [task]
        try:
            response = requests.post(url, timeout=self.timeout, json=data)
            response.raise_for_status()
            if self.debug:
                print(f"Response: {response.json()}")
            return response.json()['successed']
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")
        return False

    # NOTE: Unstable gaussian api 2.0
    def __navigate_to_waypoint(self, waypoint_name, map_name):
        ''' Ask the robot to navigate to a preconfigured waypoint on a map.
            Returns True if the robot received the command'''
        url = self.prefix + f"/gs-robot/cmd/start_cross_task?map_name={map_name}&position_name={waypoint_name}"
        try:
            response = requests.get(url, timeout=self.timeout)
            response.raise_for_status()
            if self.debug:
                print(f"Response: {response.json()}")
            return response.json()['successed']
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")
        return False


    def start_task(self, name:str, map_name:str):
        ''' Returns True if the robot has started a task/cleaning process, else False'''
        # we first get the relevant task queue and then start a new task
        data = {}
        response = self.task_queues(map_name)
        print(f"task_queues: {data}")
        if response is None:
            return False

        response_data = response["data"]
        if self.debug:
            print(f"Response data:{response_data}")
        for _data in response_data:
            if _data["name"] == self.cleaning_task_prefix + name:
                data = _data
                print(f"Data found for task!")
                break
        print(f"Data: {data}")
        url = self.prefix + "/gs-robot/cmd/start_task_queue"
        try:
            response = requests.post(url, timeout=15, json=data)
            response.raise_for_status()
            if self.debug:
                print(f"Response: {response.json()}")
            return response.json()['successed']
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")
        return False

    def get_task_queue(self, name:str, map_name:str):
        ''' Returns True if the robot has started a task/cleaning process, else False'''
        # we first get the relevant task queue and then start a new task
        data = {}
        response = self.task_queues(map_name)
        if response is None:
            return False
        response_data = response["data"]
        # if self.debug:
        #     print(f"Response data:{response_data}")
        for _data in response_data:
            if _data["name"] == self.cleaning_task_prefix + name:
                data = _data
                print(f"Data found for task!")
                break
        print(f"Data: {data}")

    def pause(self):
        url = self.prefix + f"/gs-robot/cmd/pause_task_queue"
        try:
            response = requests.get(url, timeout=self.timeout)
            response.raise_for_status()
            if self.debug:
                print(f"Response: {response.json()}")
            return response.json()["successed"]
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")
        return False

    def resume(self):
        url = self.prefix + f"/gs-robot/cmd/resume_task_queue"
        try:
            response = requests.get(url, timeout=self.timeout)
            response.raise_for_status()
            if self.debug:
                print(f"Response: {response.json()}")
            return response.json()["successed"]
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")
        return False

    def stop(self):
        ''' Returns true if robot was successfully stopped; else False'''
        url = self.prefix + f"/gs-robot/cmd/stop_task_queue"
        try:
            response = requests.get(url, timeout=self.timeout)
            response.raise_for_status()
            if self.debug:
                print(f"Response: {response.json()}")
            return response.json()['successed']
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")
        return False

    def current_map(self):
        url = self.prefix + f"/gs-robot/real_time_data/robot_status"
        try:
            response = requests.get(url, timeout=self.timeout)
            response.raise_for_status()
            if self.debug:
                # print(f"Response: \n {response.json()}")
                print(json.dumps(response.json(), indent=2))

            return response.json()["data"]["robotStatus"]["map"]["name"]
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")
        return None

    #NOTE: Unstable gaussian api 2.0. Get task status
    def __state(self):
        url = self.prefix + f"/gs-robot/real_time_data/robot_status"
        try:
            response = requests.get(url, timeout=self.timeout)
            response.raise_for_status()
            if self.debug:
                # print(f"Response: \n {response.json()}")
                data = response.json()["data"]["statusData"]
                print(json.dumps(data, indent=2))
            return data
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")
        return None

    def data(self):
        url = self.prefix + f"/gs-robot/data/device_status"
        try:
            response = requests.get(url, timeout=self.timeout)
            response.raise_for_status()
            if self.debug:
                print(f"Response: {response.json()}")
            return response.json()
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")
        return None

    def get_map(self, map_name):
        url = self.prefix + f"/gs-robot/data/map_png?map_name={map_name}"
        try:
            response = requests.get(url, timeout=self.timeout)
            response.raise_for_status()
            if self.debug:
                print(f"Response: {response.json()}")
            return response
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")
        return None

    def task_queues(self, map_name):
        url = self.prefix + f"/gs-robot/data/task_queues?map_name={map_name}"
        try:
            response = requests.get(url, timeout=self.timeout)
            response.raise_for_status()
            if self.debug:
                print(f"Response: {response.json()}")
            return response.json()
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")
        return None

    def is_task_queue_finished(self):
        # the task is completed when response["data"] is True
        url = self.prefix + f"/gs-robot/cmd/is_task_queue_finished"
        try:
            response = requests.get(url, timeout=self.timeout)
            response.raise_for_status()
            if self.debug:
                print(f"Response: {response.json()}")
            return response.json()["data"]
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")
        return False

    def navigation_completed(self):
        return self.is_task_queue_finished()

    def task_completed(self):
        ''' For ecobots the same function is used to check completion of navigation & cleaning'''
        return self.is_task_queue_finished()

    def battery_soc(self):
        response = self.data()
        if response is not None:
            return  response["data"]["battery"]/100.0
        else:
            return None

    def set_cleaning_mode(self, cleaning_config:str):
        url = self.prefix + f"/gs-robot/cmd/set_cleaning_mode?cleaning_mode={cleaning_config}"
        try:
            response = requests.get(url, timeout=self.timeout)
            response.raise_for_status()
            if self.debug:
                print(f"Response: {response.json()}")
            return response.json()['successed']
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")
        return False

    def is_charging(self):
        """Check if robot is charging, will return false if not charging, None if not avail"""
        response = self.data()
        if response is not None:
            # return response["data"]["charge"] # Faced an edge case: robot didnt dock well
            if response["data"]["chargerCurrent"] > 0.0:
                return True
            else:
                return False
        else:
            return None

    def is_localize(self):
        """Check if robot is localize, will return false if not charging, None if not avail"""
        response = self.data()
        if response is not None:
            return response["data"]["locationStatus"]
        else:
            return None
