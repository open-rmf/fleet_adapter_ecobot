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

import json
import math
from urllib.error import HTTPError

class ClientAPI:
    def __init__(self):
        self.connected = True
        self.fake_clean_path = [
            [989.96853, 1136.82267, 0],
            [1056.7065, 1087.30105, 180],
            [1056.7065, 1087.30105, 150],
            [1056.7065, 1087.30105, 130],
            [801.63, 1492.17, 0],
            [801.60, 1485.17, 20],
            [801.55, 1475.17, 40],
        ]
        self.fake_dock_path = [
            [968.51, 1325.25, 0],
            [965.21, 1338.18, 0],
            [963.21, 1348.18, 0],
            [962.59, 1360.78, 0],
            [960.59, 1370.78, 0],
            [960.59, 1363.78, 0],
            [964.36, 1356.75, 0]
        ]
        self.is_fake_cleaning = False
        self.is_fake_docking = False
        self.task_wp_idx = 0
        self.task_wp_idx = 0
        print("[TEST ECOBOT CLIENT API] successfully setup fake client api class")
        self.connected = True
        self.dock_position = [977, 1372, 0]
        self.fake_location = [977.5834, 1192.0576, 0]
        self.fake_robot_map_name = "sim_test_robot_map"
        # self.fake_location = [1072.43, 899.82, 0] #Offgrid start

    def current_map(self):
        print(f"[TEST CLIENT API] return testing map: {self.fake_robot_map_name}")
        return self.fake_robot_map_name

    def position(self):
        ''' Returns [x, y, theta] expressed in the robot's coordinate frame or None'''
        if self.is_fake_cleaning:
            print(f"[TEST CLIENT API] provide fake cleaning position")
            return self.fake_clean_path[self.task_wp_idx]
        elif self.is_fake_docking:
            print(f"[TEST CLIENT API] provide fake docking position")
            return self.fake_dock_path[self.task_wp_idx]
        print(f"[TEST CLIENT API] provide position [{self.fake_location}]")
        return self.fake_location

    def navigate(self, pose, map_name:str):
        ''' Here pose is a list containing (x,y,theta) where x and y are in grid
            coordinate frame and theta is in degrees. This functions should
            return True if the robot received the command, else False'''
        assert(len(pose) > 2)
        self.fake_location = pose
        print(f"[TEST CLIENT API] moved to fake location {pose}")
        return True

    def navigate_to_waypoint(self, waypoint_name, map_name):
        ''' Ask the robot to navigate to a preconfigured waypoint on a map.
            Returns True if the robot received the command'''
        print(f"[TEST CLIENT API] moved to fake waypoint {waypoint_name}")
        self.is_fake_docking = True
        return True

    def start_task(self, name:str, map_name:str):
        ''' Returns True if the robot has started the generic task, else False'''
        print(f"[TEST CLIENT API] Start fake task : {name}")
        self.is_fake_docking = True
        return True

    def start_clean(self, name:str, map_name:str):
        ''' Returns True if the robot has started the cleaning process, else False'''
        print(f"[TEST CLIENT API] Set fake start CLEANING : {name}")
        self.is_fake_cleaning = True
        return True

    def pause(self):
        print(f"[TEST CLIENT API] Pause Robot")
        self.is_fake_cleaning = True
        return True

    def resume(self):
        print(f"[TEST CLIENT API] Resume Robot")
        self.is_fake_cleaning = True
        return True

    def stop(self):
        ''' Returns true if robot was successfully stopped; else False'''
        print(f"[TEST CLIENT API] STOP TASK##")
        return True

    def navigation_completed(self):
        print(f"[TEST CLIENT API] FAKE NAV completed")
        return True

    def task_completed(self):
        ''' For ecobots the same function is used to check completion of navigation & cleaning'''
        self.task_wp_idx += 1
        if self.is_fake_docking:
            if self.task_wp_idx < len(self.fake_dock_path):
                print(f"[TEST CLIENT API] Fake nav/dock task in process")
                return False
            else:
                self.task_wp_idx = 0
                self.is_fake_docking = False
                self.fake_location = self.dock_position
                print(f"[TEST CLIENT API] Fake nav/dock task COMPLETED")
                return True
        else:
            if self.task_wp_idx < len(self.fake_clean_path):
                print(f"[TEST CLIENT API] FAKE CLEANING in process")
                return False
            else:
                self.task_wp_idx = 0
                self.is_fake_cleaning = False
                print(f"[TEST CLIENT API] FAKE CLEANING completed")
                return True

    def battery_soc(self):
        print(f"[TEST CLIENT API] get fake battery 100%")
        return 1.0

    def set_cleaning_mode(self, cleaning_config:str):
        print(f"[TEST CLIENT API] Set fake CLEANING MODE: {cleaning_config}")
        return True

    def is_charging(self):
        """Check if robot is charging, will return false if not charging, None if not avail"""
        dx, dy, _ = self.dock_position
        x, y, _= self.fake_location
        if (abs(x-dx) < 5.0 and abs(y-dy) < 5.0):
            print(f"[TEST CLIENT API] Fake robot at dock, is charging")
            return True
        return False

    def is_localize(self):
        return True
