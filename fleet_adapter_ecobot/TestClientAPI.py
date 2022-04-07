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
            [1027.0187, 1271.659514, -60],
            [1027.0187, 1271.659514, -30],
            [1027.0187, 1271.659514, -100],
            [1027.0187, 1271.659514, -60],
            [1027.0187, 671.659514, -30],
            [1027.0187, 671.659514, -60],
            [1027.0187, 671.659514, -100],
            [1027.0187, 671.659514, -60],
            [1027.0187, 671.659514, -30],
            [500.0187, 905.659514, 0],
            [500.0187, 905.659514, 20],
            [500.0187, 905.659514, 40],
            [500.0187, 905.659514, 60],
            [500.0187, 905.659514, 80],
            [500.0187, 905.659514, 100],
            [936.238485, 981.95061, 0],
            [909.963465, 1139.7444, 180]
        ]
        self.is_fake_cleaning = False
        self.clean_wp_idx = 0
        print("[TEST ECOBOT CLIENT API] successfully setup fake client api class")
        self.connected = True
        # self.fake_location = [977, 1372, 0 ]
        self.fake_location = [977.5834309628409, 1192.0576445043025, 0]

    def position(self):
        ''' Returns [x, y, theta] expressed in the robot's coordinate frame or None'''
        if self.is_fake_cleaning:
            print(f"[TEST CLIENT API] provide fake cleaning position")
            return self.fake_clean_path[self.clean_wp_idx]
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
        self.fake_location = [977, 1372, 0 ] # assume original charging waypoint
        self.attempts = 0
        return True

    def start_task(self, name:str, map_name:str):
        ''' Returns True if the robot has started the generic task, else False'''
        print(f"[TEST CLIENT API] Start fake task : {name}")
        self.fake_location = [977, 1372, 0 ] # assume original charging waypoint
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
        if not self.is_fake_cleaning: # note, if immediately, the robot will head back to staging point
            if self.attempts < 4:
                print(f"[TEST CLIENT API] Fake task in process")
                self.attempts += 1
                return False
            else:
                print(f"[TEST CLIENT API] No fake task in process")
                return True
        self.clean_wp_idx += 1
        if self.clean_wp_idx < len(self.fake_clean_path):
            print(f"[TEST CLIENT API] FAKE DOCK/CLEAN in process")
            return False
        else:
            self.clean_wp_idx = 0
            self.is_fake_cleaning = False
            print(f"[TEST CLIENT API] FAKE DOCK/CLean completed")
            return True

    def battery_soc(self):
        print(f"[TEST CLIENT API] get fake battery 100%")
        return 1.0

    def set_cleaning_mode(self, cleaning_config:str):
        print(f"[TEST CLIENT API] Set fake CLEANING MODE: {cleaning_config}")
        return True

    ## TODO: check is charge
