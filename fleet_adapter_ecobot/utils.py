#!/usr/bin/env python3

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

from typing import List, Tuple, Optional

import nudged
import numpy as np
import math
import sys

class RmfMapTransform:
    def __init__(self, tx=0.0, ty=0.0, theta=0.0, scale=1.0):
        """
        set transformation from rmf map (parent frame) to robot map (child frame)
        default no transformation
        Parameters:
            [tx, ty] translation; theta: radians; scale: robot map scale
        """

        s = math.cos(theta)*scale
        r = math.sin(theta)*scale
        self.to_rmf_tf = nudged.Transform(s, r, tx, ty)
        tx, ty = self.to_rmf_tf.get_translation()
        r = self.to_rmf_tf.get_rotation()
        s = self.to_rmf_tf.get_scale()
        self.__apply_inverse_mat()

    def estimate(
            self,
            robot_map_points: List[Tuple[float, float]],
            rmf_map_points: List[Tuple[float, float]]
        ) -> Optional[float]:
        '''
        Compute transformation by providing 2 set of coordinates, rmf and robot map

        Parameters
            robot_map_points:   list of [x, y] 2D lists
            rmf_map_points:     list of [x, y] 2D lists
        Return
            mean square error, return None if invalid
        '''

        if len(rmf_map_points) != len(robot_map_points):
            print("Err! Input sets have different number of waypoints")
            return None

        self.to_rmf_tf = nudged.estimate(robot_map_points, rmf_map_points)
        print(self.to_rmf_tf.tx, self.to_rmf_tf.ty)

        self.__apply_inverse_mat()
        return nudged.estimate_error(self.to_rmf_tf, rmf_map_points, robot_map_points)

    def to_rmf_map(
            self,
            robot_coor: Tuple[float, float, float]
        ) -> Tuple[float, float, float]:
        '''
        Get coordinate from robot map to rmf map

        Parameters
            robot_coor:         coordinate [x, y, theta] in robot map
        Return
            coordinate [x, y, theta] in rmf map
        '''

        # Note: this is similar to mat mul of tf mat [3x3] and coor [3x1]
        # mat = self.to_rmf_tf.get_matrix()
        # x = np.matmul(np.array(mat), np.array([robot_coor[0],robot_coor[1], 1]))
        x,y = self.to_rmf_tf.transform([robot_coor[0],robot_coor[1]])
        theta = robot_coor[2] + self.to_rmf_tf.get_rotation()
        theta = (theta + math.pi) % (2*math.pi) - math.pi  #convert to within range -pi, pi
        return (x, y, theta)


    def to_robot_map(
            self,
            rmf_coor: Tuple[float, float, float]
        ) -> Tuple[float, float, float]:
        '''
        Get coordinate from rmf map to robot map

        Parameters
            rmf_coor:         coordinate [x, y, theta] in rmf map
        Return
            coordinate [x, y, theta] in robot map
        '''
        x,y = self.to_robot_tf.transform([rmf_coor[0],rmf_coor[1]])
        theta = rmf_coor[2] + self.to_robot_tf.get_rotation()
        theta = (theta + math.pi) % (2*math.pi) - math.pi  #convert to within range -pi, pi
        return (x, y, theta)


    def to_rmf_map_transform(self) -> Tuple[float, float, float, float]:
        '''
        Get coordinate from robot map to rmf map

        Return
            transformation [tx, ty, theta, scale]
        '''
        tx, ty = self.to_rmf_tf.get_translation()
        r = self.to_rmf_tf.get_rotation()
        s = self.to_rmf_tf.get_scale()
        return (tx, ty, r, s)


    def to_robot_map_transform(self) -> Tuple[float, float, float, float]:
        '''
        Get coordinate from rmf map to robot map

        Return
            transformation [tx, ty, theta, scale]
        '''
        tx, ty = self.to_robot_tf.get_translation()
        r = self.to_robot_tf.get_rotation()
        s = self.to_robot_tf.get_scale()
        return (tx, ty, r, s)


    def __apply_inverse_mat(self):
        # calculate to robot tf
        mat = self.to_rmf_tf.get_matrix()
        np_mat = np.array(mat)
        inv_mat = np.linalg.inv(np_mat)

        self.to_robot_tf = nudged.Transform(
            inv_mat[0][0], inv_mat[1][0], inv_mat[0][2], inv_mat[1][2])
