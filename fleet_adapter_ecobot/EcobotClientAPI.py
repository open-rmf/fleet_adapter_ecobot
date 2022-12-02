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
from urllib.error import HTTPError
from packaging.version import Version, parse

class EcobotAPI:
    """
    This class that simplifies the process of sending GET/POST commands to an Ecobot.

    Responses returned from these requests are deserialized dictionaries.
    """

    def __init__(
        self,
        prefix:str, cleaning_task_prefix="", api_version = "",
        timeout=10.0, debug=False):
        """_summary_

        Parameters
        ----------
        prefix : str
            prefix added to API endpoints. Usually the IP Address or Host name.
            A sample prefix is 'http://10.7.5.88:8080'
        cleaning_task_prefix : str, optional
            Prefix added to the task name when requesting a task, by default ""
        api_version : str, optional
            Optional API Version to be specified for robot. If not provided,
            api_version will fetched from the robot., by default ""
        timeout : float, optional
            Timeout for REST API requests, by default 10.0
        debug : bool, optional
            If true, then debug statements will be printed, by default False

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

        Returns
        -------
        bool
            True if robot is connected, else False

        """
        return self.connected

    def load_map(self, map_name: str):
        """Load a map with the given name onto the robot

        Parameters
        ----------
        map_name : str
            Name of map to load

        Returns
        -------
        bool
            True if loading of map succeeded, else False

        """
        url = self.prefix + f"/gs-robot/cmd/load_map?map_name={map_name}"

        success = self.get_request(url).get('successed', False)

        return success

    def localize(self, init_point:str, map_name:str, rotate=False):
        """Localize the robot

        Parameters
        ----------
        init_point : str
            Position to localize robot to
        map_name : str
            Name of current map
        rotate : bool, optional
            True if yaw is provided in 'init_point' argument, by default False

        Returns
        -------
        bool
            True if localization has succeeded, else False

        """
        if rotate: # The robot will rotate to improve localization
            url = self.prefix + f"/gs-robot/cmd/initialize?map_name={map_name}&init_point_name={init_point}"
        else: # The specified init point must be accurate
            url = self.prefix + f"/gs-robot/cmd/initialize_directly?map_name={map_name}&init_point_name={init_point}"

        success = self.get_request(url).get('successed', False)

        return success

    def position(self):
        """Get current robot position

        Returns
        -------
        bool, None
            Robot position [x, y, theta] expressed in the robot's
            coordinate frame. Note that theta is in degrees. If no
            valid response then None is returned

        """
        url = self.prefix + r"/gs-robot/real_time_data/position"

        position = self.get_request(url, return_dict=False)

        if position is None:
            return None

        x = position.get('gridPosition', {}).get('x', None)
        y = position.get('gridPosition', {}).get('y', None)
        angle = position.get('angle', None)

        return [x, y, angle]

    def navigate(self, pose, map_name:str):
        """Navigate to the given pose in (x,y,theta)

        Parameters
        ----------
        pose : list
            List containing 3 values in the form of [x,y,theta],
            where x and y are in grid coordinate frame and theta is in degrees.
        map_name : str
            Name of current map

        Returns
        -------
        bool
            True if the robot received the command, else False
        """

        assert len(pose) >= 3, "argument 'pose' must be a list with at least 3 values"

        if self.api_at_least("3.6.6"):
            url = self.prefix + r"/gs-robot/cmd/quick/navigate?type=2"
            data = {}
            data["destination"] = {
                "gridPosition": {
                    "x": pose[0],
                    "y": pose[1]
                },
                "angle": pose[2]
            }

            success = self.post_request(url, data).get('successed')
        else:
            url = self.prefix + r"/gs-robot/cmd/start_task_queue"
            data = {}
            data["name"] = ""
            data["loop"] = False
            data["loop_count"] = 0
            data["map_name"] = map_name # this is the name of the map as stored on the robot
            task =  {
                "name":"NavigationTask",
                "start_param":{
                    "destination":{
                        "angle":pose[2],
                        "gridPosition":{
                            "x":pose[0],
                            "y":pose[1]
                        }
                    }
                }
            }
            data["tasks"] = [task]

            success = self.post_request(url, data).get('successed')

        if success is None:
            return False
        return success

    def navigate_to_waypoint(self, waypoint_name: str, map_name: str):
        """Ask the robot to Navigate to a preconfigured waypoint on a map. This action
        is also used to dock the robot to a charging point.

        Parameters
        ----------
        waypoint_name : str
            Name of waypoint to navigate to
        map_name : str
            Name of current map

        Returns
        -------
        bool
            True if robot has received the command, else False

        """
        if self.api_at_least("3.6.6"):
            url = self.prefix + f"/gs-robot/cmd/start_cross_task?map_name={map_name}&position_name={waypoint_name}"

            success = self.get_request(url).get('successed', False)
        else:
            url = self.prefix + r"/gs-robot/cmd/start_task_queue"
            data = {}
            data["name"] = ""
            data["loop"] = False
            data["loop_count"] = 0
            data["map_name"] = map_name # this is the name of the map as stored on the robot
            task =  {   "name":"NavigationTask",
                        "start_param":{
                            "map_name":map_name,
                            "position_name":waypoint_name
                            }
                    }
            data["tasks"] = [task]

            success = self.post_request(url, data).get('successed', False)

        return success

    def start_task(self, name:str, map_name:str):
        """Start a task.

        Parameters
        ----------
        name : str
            Task name
        map_name : str
            Name of current map

        Returns
        -------
        bool
            True if robot has started a task/cleaning process, else False

        """
        task_exists = False

        data = {}
        data["loop"] = False
        data["loop_count"] = 0
        data["name"] = name
        data["map_name"] = map_name # this is the name of the map as stored on the robot
        data["tasks"] = []

        response = self.task_queues(map_name)
        if response is None:
            return False

        # we first get the relevant task queue and then start a new task
        if self.debug:
            print(f"Response from querying task queues: \n{response['data']}")
        for task in response["data"]:
            if task["name"] == self.cleaning_task_prefix + name:
                print(f"Task Data: {data}")
                print(r"Data found for task!")
                task_exists = True
                break

        if not task_exists:
            return False 

        url = self.prefix + "/gs-robot/cmd/start_task_queue"

        success = self.post_request(url, data).get('successed')
        if success is None:
            return False
        return success

    def pause(self):
        """Pause the task

        Returns
        -------
        bool
            True if task is paused successfully, else False
        """

        url = self.prefix + r"/gs-robot/cmd/pause_task_queue"

        success = self.get_request(url).get('successed', False)
        
        return success

    def resume(self):
        """Resume the task.

        Returns
        -------
        bool
            True if task is resumed successfully, else False
        """

        url = self.prefix + r"/gs-robot/cmd/resume_task_queue"

        success = self.get_request(url).get('successed', False)

        return success

    def stop(self):
        """Stop the ongoing task.

        Returns
        -------
        bool
            True if robot was successfully stopped, else False
        """

        # TODO Pending clarification, do not use
        if self.api_at_least("3.6.6"):
            url = self.prefix + r"/gs-robot/cmd/stop_cross_task"

            success = self.get_request(url).get('successed', False)
        else:
            url = self.prefix + r"/gs-robot/cmd/stop_task_queue"

            success = self.get_request(url).get('successed', False)

        return success

    def current_map(self):
        """Get name of currently loaded map.

        Returns
        -------
        str
            current map name
        """

        url = self.prefix + r"/gs-robot/real_time_data/robot_status"

        robot_status = self.get_request(url).get('data', {}).get('robotStatus', None)

        if robot_status is not None:
            current_map_name = robot_status.get('map', {}).get('name', None)
            return current_map_name

        return None

    #NOTE: Unstable gaussian api 2.0. Get task status
    def __state(self):
        """Get current robot status

        Returns
        -------
        str
            Current robot status
        """
        url = self.prefix + r"/gs-robot/real_time_data/robot_status"

        robot_state = self.get_request(url).get('data',{}).get('statusData', None)

        return robot_state

    def data(self):
        """Get robot device status

        Returns
        -------
        dict
            dictionary of robot device status
        """
        url = self.prefix + r"/gs-robot/data/device_status"

        robot_status = self.get_request(url, return_dict=False)

        return robot_status

    def get_map(self, map_name: str):
        """Get map image

        Parameters
        ----------
        map_name : name of current map

        Returns
        -------
        # TODO Find return type
        ??
            png of map
        """

        url = self.prefix + f"/gs-robot/data/map_png?map_name={map_name}"

        print("Retrieving map")

        map_png = self.get_request(url, return_dict=False)

        print("Map retrieved")

        return map_png

    def task_queues(self, map_name: str):
        """Get list of available tasks

        Parameters
        ----------
        map_name : str
            Current map name

        Returns
        -------
        dict
            dictionary of available tasks
        """

        url = self.prefix + f"/gs-robot/data/task_queues?map_name={map_name}"

        task_queues = self.get_request(url, return_dict=False)

        return task_queues

    def is_task_queue_finished(self):
        """Check if task completed.

        Returns
        -------
        bool or NoneType
            True if task is finished, else False.
            If data not available, return None
        """

        # TODO The new API seems to be unstable, it returns true for tasks that still being executed,
        # resulting in premature completion of cleaning tasks.
        
        # if self.api_at_least("3.6.6"):
        #     # This only works for navigation tasks and not cleaning tasks
        #     url = self.prefix + r"/gs-robot/cmd/is_cross_task_finished"

        #     success = self.get_request(url)['data']
        # else:
            # # This works for both navigation and cleaning tasks
            # the task is completed when response["data"] is True
            # url = self.prefix + r"/gs-robot/cmd/is_task_queue_finished"

            # success = self.get_request(url)['data']
            
        url = self.prefix + r"/gs-robot/cmd/is_task_queue_finished"

        success = self.get_request(url).get('data', False)

        return success

    def navigation_completed(self):
        """Check navigation task completed.

        Returns
        -------
        bool or NoneType
            True if task is finished, else False.
            Return None if data is not available
        """
        return self.is_task_queue_finished()

    def task_completed(self):
        """Check task completed. For ecobots the same function is used
        to check completion of navigation & cleaning

        Returns
        -------
        bool or NoneType
            True if task is finished, else False.
            Return None if data is not available

        """

        return self.is_task_queue_finished()

    def battery_soc(self):
        """Get battery state of charge.

        Returns
        -------
        double or NoneType
            Returns Percentage level of battery. If data not
            available then return None.

        """
        response = self.data()
        if response is not None:
            return response["data"]["battery"]/100.0
        return None

    def set_cleaning_mode(self, cleaning_config:str):
        """Set cleaning mode of robot

        Parameters
        ----------
        cleaning_config : str
            cleaning mode configuration

        Returns
        -------
        bool or NoneType
            Returns True if robot's cleaning mode set successfully.

        """
        url = self.prefix + f"/gs-robot/cmd/set_cleaning_mode?cleaning_mode={cleaning_config}"

        success = self.get_request(url).get('successed', False)

        return success

    def is_charging(self):
        """Check if robot is charging

        Returns
        -------
        bool or NoneType
            Returns True if robot is charging. If data not
            available then return None.

        """
        response = self.data()
        if response is None:
            return None
        # return response["data"]["charge"] # Faced an edge case: robot didnt dock well
        return response["data"]["chargerCurrent"] > 0.0

    def is_localize(self):
        """Check if robot is localized

        Returns
        -------
        bool or NoneType
            True if robot is localized, else False. Returns
            None if data is not available

        """
        response = self.data()
        if response is None:
            return None
        return response.get("data", {}).get("locationStatus", None)

    def get_api_version(self):
        """Get the REST API version.

        This API Version is used to decide which endpoints will be
        used across different firmware versions.

        Returns
        -------
        str
            The semantic version of the API in the form "X.X.X"

        """
        url = self.prefix + r"/gs-robot/info"
        api_version_raw = self.get_request(url).get('data', {}).get('version', None)
        if api_version_raw is not None:
            api_version = api_version_raw.split("_")[1][1:].replace("-", ".")
            return api_version
        return None

    def api_at_least(self, min_sem_version: str):
        """Check if the current robot's API version is at least 'min_sem_version'.

        Parameters
        ----------
        min_sem_version : str
            The minimum semantic version considered
            to be the new API, in the form "X.X.X"

        Returns
        -------
        bool
            True if current API is at least specified minimum semantic version

        """
        return self.api_version >= parse(min_sem_version)

    def get_request(self, url: str, return_dict=True):
        """Send a GET request.

        Parameters
        ----------
        url : str
            URL to send GET request to
        return_dict : bool, optional
            True if response is to be converted
            to a dictionary, else return as type
            'requests.models.Response', by default True

        Returns
        -------
        dict, requests.models.Response
            If return_dict is True, then return Response data in python dictionary form,
            else return as requests.models.Response type.

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
                return response
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")

        if return_dict:
            return {}
        return None

    def post_request(self, url: str, data):
        """Send a POST request.

        Parameters
        ----------
        url : str
            URL to send POST request to
        data : str
            body of POST Request

        Returns
        -------
        dict
            Response data from POST request

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
