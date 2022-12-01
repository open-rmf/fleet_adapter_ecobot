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

import sys
import yaml

class ClientAPI:
    def __init__(self, config_file):
        self.connected = True
        with open(config_file, "r") as f:
            config_yaml = yaml.safe_load(f)

        # These configs are provided in the yaml file
        self.mock_clean_path = config_yaml["mock_clean_path"]
        self.mock_dock_path = config_yaml["mock_dock_path"]
        self.dock_position = config_yaml["dock_position"]
        self.mock_location = config_yaml["mock_location"]
        self.mock_robot_map_name = config_yaml["mock_robot_map_name"]

        self.is_mock_cleaning = False
        self.is_mock_docking = False
        self.task_wp_idx = 0
        print("[TEST ECOBOT CLIENT API] successfully setup mock client api class")

    def online(self):
        return True

    def current_map(self):
        print(f"[TEST CLIENT API] return testing map: {self.mock_robot_map_name}")
        return self.mock_robot_map_name

    def position(self):
        ''' Returns [x, y, theta] expressed in the robot's coordinate frame or None'''
        if self.is_mock_cleaning:
            print(f"[TEST CLIENT API] provide mock cleaning position")
            return self.mock_clean_path[self.task_wp_idx]
        elif self.is_mock_docking:
            print(f"[TEST CLIENT API] provide mock docking position")
            return self.mock_dock_path[self.task_wp_idx]
        print(f"[TEST CLIENT API] provide position [{self.mock_location}]")
        return self.mock_location

    def navigate(self, pose, map_name:str):
        ''' Here pose is a list containing (x,y,theta) where x and y are in grid
            coordinate frame and theta is in degrees. This functions should
            return True if the robot received the command, else False'''
        assert(len(pose) > 2)
        self.mock_location = pose
        print(f"[TEST CLIENT API] moved to mock location {pose}")
        return True

    def navigate_to_waypoint(self, waypoint_name, map_name):
        ''' Ask the robot to navigate to a preconfigured waypoint on a map.
            Returns True if the robot received the command'''
        print(f"[TEST CLIENT API] moved to mock waypoint {waypoint_name}")
        self.is_mock_docking = True
        return True

    def start_task(self, name:str, map_name:str):
        ''' Returns True if the robot has started the generic task, else False'''
        print(f"[TEST CLIENT API] Start mock task : {name}")
        if not self.is_mock_docking:
            self.is_mock_cleaning = True
        return True

    def pause(self):
        print(f"[TEST CLIENT API] Pause Robot")
        self.is_mock_cleaning = True
        return True

    def resume(self):
        print(f"[TEST CLIENT API] Resume Robot")
        self.is_mock_cleaning = True
        return True

    def stop(self):
        ''' Returns true if robot was successfully stopped; else False'''
        print(f"[TEST CLIENT API] STOP TASK##")
        return True

    def navigation_completed(self):
        print(f"[TEST CLIENT API] MOCK NAV completed")
        return True

    def task_completed(self):
        ''' For ecobots the same function is used to check completion of navigation & cleaning'''
        self.task_wp_idx += 1
        if self.is_mock_docking:
            if self.task_wp_idx < len(self.mock_dock_path):
                print(f"[TEST CLIENT API] Mock nav/dock task in process")
                return False
            else:
                self.task_wp_idx = 0
                self.is_mock_docking = False
                self.mock_location = self.dock_position
                print(f"[TEST CLIENT API] Mock nav/dock task COMPLETED")
                return True
        else:
            if self.task_wp_idx < len(self.mock_clean_path):
                print(f"[TEST CLIENT API] MOCK CLEANING in process")
                return False
            else:
                self.task_wp_idx = 0
                self.is_mock_cleaning = False
                print(f"[TEST CLIENT API] MOCK CLEANING completed")
                return True

    def battery_soc(self):
        print(f"[TEST CLIENT API] get mock battery 100%")
        return 1.0

    def set_cleaning_mode(self, cleaning_config:str):
        print(f"[TEST CLIENT API] Set mock CLEANING MODE: {cleaning_config}")
        return True

    def is_charging(self):
        """Check if robot is charging, will return false if not charging, None if not avail"""
        dx, dy, _ = self.dock_position
        x, y, _= self.mock_location
        if (abs(x-dx) < 5.0 and abs(y-dy) < 5.0):
            print(f"[TEST CLIENT API] Mock robot at dock, is charging")
            return True
        return False

    def is_localize(self):
        return True
