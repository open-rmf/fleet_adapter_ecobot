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

from re import L
import requests
import json
import math
from urllib.error import HTTPError
from packaging.version import Version, parse

# Sample prefix: http://10.7.5.88:8080
# This class simplifies the process of sending GET/POST commands to an Ecobot
# Responses returned are deserialized dictionaries
class EcobotAPI:
    def __init__(self, prefix:str, cleaning_task_prefix="", api_version = "",timeout=10.0, debug=False):
        """Constructor for EcobotAPI

        :param prefix: prefix added to API endpoints. Usually the IP Address or Host name
        :type prefix: str
        :param cleaning_task_prefix: Prefix added to the task name when requesting a task
        :type cleaning_task_prefix: str
        :param api_version: Optional API Version to be specified for robot. If not provided,
            api_version will fetched from the robot.
        :type api_version: str
        :param timeout: Timeout for REST API requests
        :type timeout: double
        :param debug: If true, then debug statements will be printed
        :type debug: bool
        """
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

        # Get REST API Version
        if api_version == "":
            self.api_version = Version(self.get_api_version())
        else:
            self.api_version = api_version

    def online(self):
        """Indicates if robot is connected

        :returns: True if robot is connected, else False
        :rtype: bool
        """
        return self.connected

    def load_map(self, map_name: str):
        """Load a map with the given name onto the robot

        :param map_name: Name of map to load
        :type map_name: str

        :returns: True if loading of map succeeded, else False
        :rtype: bool
        """
        url = self.prefix + f"/gs-robot/cmd/load_map?map_name={map_name}"

        success = self.get_request(url)["successed"]

        if success is None:
            return False
        return success

    def localize(self, init_point:str, map_name:str, rotate=False):
        """Localize the robot

        :param init_point: Position to localize robot to
        :type init_point: str
        :param map_name: Name of current map
        :type map_name: str
        :param rotate: True if yaw is provided in 'init_point' argument
        :type rotate: bool

        :returns: True if localization has succeeded, else False
        :rtype: bool
        """
        if rotate: # The robot will rotate to improve localization
            url = self.prefix + f"/gs-robot/cmd/initialize?map_name={map_name}&init_point_name={init_point}"
        else: # The specified init point must be accurate
            url = self.prefix + f"/gs-robot/cmd/initialize_directly?map_name={map_name}&init_point_name={init_point}"
        
        success = self.get_request(url)["successed"]
        if success is None:
            return False
        return success

    def position(self):
        """Get current robot position

        :returns: 
            - Robot position: [x, y, theta] expressed in the robot's 
                coordinate frame. Note that theta is in degrees.
            - None if not available
        :rtype: bool
        """
        url = self.prefix + f"/gs-robot/real_time_data/position"

        position = self.get_request(url)

        x = position["gridPosition"]["x"]
        y = position["gridPosition"]["y"]
        angle = position["angle"]

        return [x, y, angle]

    def navigate(self, pose, map_name:str):
        """Navigate to the given pose in (x,y,theta)

        :param pose: List containing 3 values in the form of [x,y,theta],
            where x and y are in grid coordinate frame and theta is in degrees.
        :type pose: list
        :param map_name: Name of current map
        :type map_name: str

        :returns: True if the robot received the command, else False
        :rtype: bool
        """
        assert len(pose) >= 3, "argument 'pose' must be a list with at least 3 values"

        if self.api_at_least("3.6.6"):
            url = self.prefix + f"/gs-robot/cmd/quick/navigate?type=2"
            data = {}
            data["destination"] = {"gridPosition": {"x": pose[0], "y": pose[1]}, "angle": pose[2]}

            success = self.post_request(url, data)['successed']
        else:
            url = self.prefix + f"/gs-robot/cmd/start_task_queue"
            data = {}
            data["name"] = ""
            data["loop"] = False
            data["loop_count"] = 0
            data["map_name"] = map_name # this is the name of the map as stored on the robot
            task = {"name":"NavigationTask","start_param":{"destination":{"angle":pose[2],"gridPosition":{"x":pose[0], "y":pose[1]}}}}
            data["tasks"] = [task]

            success = self.post_request(url, data)['successed']

        if success is None:
            return False
        return success

    def navigate_to_waypoint(self, waypoint_name, map_name):
        """Ask the robot to Navigate to a preconfigured waypoint on a map. This action
        is also used to dock the robot to a charging point.

        :param waypoint_name: Name of waypoint to navigate
        :type waypoint_name: str
        :param map_name: Name of current map
        :type map_name: str

        :returns: True if robot has received the command, else False
        :rtype: bool
        """
        if self.api_at_least("3.6.6"):
            url = self.prefix + f"/gs-robot/cmd/start_cross_task?map_name={map_name}&position_name={waypoint_name}"
            
            success = self.get_request(url)["successed"]
        else:
            url = self.prefix + f"/gs-robot/cmd/start_task_queue"
            data = {}
            data["name"] = ""
            data["loop"] = False
            data["loop_count"] = 0
            data["map_name"] = map_name # this is the name of the map as stored on the robot
            task = {"name":"NavigationTask","start_param":{"map_name":map_name, "position_name":waypoint_name}}
            data["tasks"] = [task]
            
            success = self.post_request(url, data)['successed']

        if success is None:
            return False
        return success

    def start_task(self, name:str, map_name:str):
        """Start a task.

        :param name: Task name
        :type name: str
        :param map_name: Name of current map
        :type map_name: str

        :returns: True if robot has started a task/cleaning process, else False
        :rtype: bool
        """
        # we first get the relevant task queue and then start a new task
        task_data = {}
        response = self.task_queues(map_name)
        if response is None:
            return False

        available_tasks = response["data"]
        if self.debug:
            print(f"Response from querying task queues: \n{available_tasks}")
        for task in available_tasks:
            if task["name"] == self.cleaning_task_prefix + name:
                task_data = task
                print(f"Data found for task!")
                break
        print(f"Task Data: {task_data}")

        url = self.prefix + "/gs-robot/cmd/start_task_queue"

        success = self.post_request(url, task_data)['successed']
        if success is None:
            return False
        return success

    def pause(self):
        """Pause the task

        :returns: True if task is paused successfully, else False
        :rtype: bool
        """
        url = self.prefix + f"/gs-robot/cmd/pause_task_queue"

        success = self.get_request(url)["successed"]
        if success is None:
            return False
        return success

    def resume(self):
        """Resume the task

        :returns: True if task is resumed successfully, else False
        :rtype: bool
        """
        url = self.prefix + f"/gs-robot/cmd/resume_task_queue"

        success = self.get_request(url)['successed']
        if success is None:
            return False
        return success

    def stop(self):
        """Stop the ongoing task

        :returns: True if robot was successfully stopped, else False
        :rtype: bool
        """
        if self.api_at_least("3.6.6"):
            url = self.prefix + f"/gs-robot/cmd/stop_cross_task"

            success = self.get_request(url)['data']
        else:
            url = self.prefix + f"/gs-robot/cmd/stop_task_queue"
            
            success = self.get_request(url)['successed']

        if success is None:
            return False
        return success

    def current_map(self):
        """Get name of currently loaded map

        :returns: current map name
        :rtype: str
        """
        url = self.prefix + f"/gs-robot/real_time_data/robot_status"

        current_map_name = self.get_request(url)["data"]["robotStatus"]["map"]["name"]

        return current_map_name

    #NOTE: Unstable gaussian api 2.0. Get task status
    def __state(self):
        url = self.prefix + f"/gs-robot/real_time_data/robot_status"

        robot_state = self.get_request(url)["data"]["statusData"]

        return robot_state

    def data(self):
        """Get robot status

        :returns: dictionary of robot status
        :rtype: dict
        """
        url = self.prefix + f"/gs-robot/data/device_status"

        robot_status = self.get_request(url)

        return robot_status
        
    def get_map(self, map_name):
        """Get map image

        # TODO Find return type
        :returns: png of map 
        :rtype: ??
        """
        url = self.prefix + f"/gs-robot/data/map_png?map_name={map_name}"

        print("Retrieving map")

        map_png = self.get_request(url, return_dict=False)

        print("Map retrieved")
        
        return map_png

    def task_queues(self, map_name):
        """Get list of available tasks

        :returns: dictionary of available tasks
        :rtype: dict
        """
        url = self.prefix + f"/gs-robot/data/task_queues?map_name={map_name}"

        task_queues = self.get_request(url)

        return task_queues

    def is_task_queue_finished(self):
        """Check if task completed

        :returns: 
            - True if task is finished, else False
            - None if data is not available
        :rtype: bool or NoneType
        """
        # TODO Check if this only checks navigation tasks or also cleaning tasks
        # If it is only used to check navigation tasks, then we might need to update 
        # navigation_completed and task_completed methods to use different endpoints for checking
        if self.api_at_least("3.6.6"):
            url = self.prefix + f"/gs-robot/cmd/is_cross_task_finished"

            success = self.get_request(url)['data']
        else:
            # the task is completed when response["data"] is True
            url = self.prefix + f"/gs-robot/cmd/is_task_queue_finished"

            success = self.get_request(url)['data']
            
        if success is None:
            return False
        return success

    def navigation_completed(self):
        """Check navigation task completed

        :returns: 
            - True if task is finished, else False
            - None if data is not available
        :rtype: bool or NoneType
        """
        return self.is_task_queue_finished()

    def task_completed(self):
        """Check task completed. For ecobots the same function is used 
        to check completion of navigation & cleaning

        :returns: 
            - True if task is finished, else False
            - None if data is not available
        :rtype: bool or NoneType
        """
        return self.is_task_queue_finished()

    def battery_soc(self):
        """Get battery state of charge

        :returns: 
            - Percentage level of battery
            - None if data is not available
        :rtype: double or NoneType
        """
        response = self.data()
        if response is not None:
            return response["data"]["battery"]/100.0
        else:
            return None

    def set_cleaning_mode(self, cleaning_config:str):
        """Check if robot is charging

        :returns: 
            - True if robot is charging, else False
            - None if data is not available
        :rtype: bool or NoneType
        """
        url = self.prefix + f"/gs-robot/cmd/set_cleaning_mode?cleaning_mode={cleaning_config}"

        success = self.get_request(url)['successed']
        if success is None:
            return False
        return success

    def is_charging(self):
        """Check if robot is charging

        :returns: 
            - True if robot is charging, else False
            - None if data is not available
        :rtype: bool or NoneType
        """
        response = self.data()
        if response is None:
            return None
        # return response["data"]["charge"] # Faced an edge case: robot didnt dock well
        return response["data"]["chargerCurrent"] > 0.0

    def is_localize(self):
        """Check if robot is localized

        :returns: 
            - True if robot is localized, else False
            - None if data is not available
        :rtype: bool or NoneType
        """
        response = self.data()
        if response is None:
            return None
        return response["data"]["locationStatus"]

    def get_api_version(self):
        """Get the REST API version. 
        This affects the REST Endpoints being used for the client API

        :returns: The semantic version of the API in the form "X.X.X"
        :rtype: str
        """
        url = self.prefix + f"/gs-robot/info"
        api_version = self.get_request(url)["data"]["version"].split("_")[1][1:]
        api_version = api_version.replace("-", ".")
        return api_version
    
    def api_at_least(self, min_sem_version):
        """Check if robot is using a new API. This is done by comparing
            against the 'min_sem_version' argument specified
        
        :param min_sem_version: The minimum semantic version considered 
            to be the new API, in the form "X.X.X" 
        :type min_sem_version: str

        :returns: 
            - True if current API meets specified minimum semantic version, 
            else False
        :rtype: bool
        """

        return self.api_version >= parse(min_sem_version)

    def get_request(self, url, return_dict=True):
        """Send a GET request

        :param url: URL to send GET request to
        :type url: str
        :param return_dict: True if response is to be converted 
            to a dictionary, else return as type 'requests.models.Response'
        :type return_dict: bool

        :returns: Response data from GET request
        :rtype: dict, requests.models.Response
        """
        try:
            response = requests.get(url, timeout=self.timeout)
            response.raise_for_status()
            if self.debug:
                print(f"Response from GET request to '{url}':\n" \
                  f"{json.dumps(response.json(), indent=2)}")
            if response is not None:
                if return_dict:
                    return response.json()
                else: 
                    return response
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")
        return None

    def post_request(self, url, data):
        """Send a POST request

        :param url: URL to send POST request to
        :type url: str
        :param data: body of POST Request
        :type data: dict

        :returns: Response data from POST request
        :rtype: dict
        """
        try:
            response = requests.post(url, timeout=self.timeout, json=data)
            response.raise_for_status()
            if self.debug:
                print(f"Response from POST request to '{url}':\n" \
                  f"{json.dumps(response.json(), indent=2)}")
            if response is not None:
                return response.json()
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")
        return None