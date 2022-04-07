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


import rmf_adapter as adpt
import rmf_adapter.plan as plan
import rmf_adapter.schedule as schedule

from rmf_fleet_msgs.msg import DockSummary

import numpy as np

import threading
import math
import copy
import enum
import time

from datetime import timedelta

from typing import Optional, Tuple

# States for EcobotCommandHandle's state machine used when guiding robot along
# a new path
class EcobotState(enum.IntEnum):
    IDLE = 0
    WAITING = 1
    MOVING = 2
    CLEANING =3

# Custom wrapper for Plan::Waypoint. We use this to modify position of waypoints
# to prevent backtracking
class PlanWaypoint:
    def __init__(self, index, wp:plan.Waypoint):
        self.index = index # this is the index of the Plan::Waypoint in the waypoints in follow_new_path
        self.position = wp.position
        self.time = wp.time
        self.graph_index = wp.graph_index
        self.approach_lanes = wp.approach_lanes


class EcobotCommandHandle(adpt.RobotCommandHandle):
    def __init__(self,
                 name,
                 config,
                 node,
                 graph,
                 vehicle_traits,
                 transforms,
                 map_name,
                 rmf_map_name,
                 position,
                 charger_waypoint,
                 update_frequency,
                 adapter,
                 api):
        adpt.RobotCommandHandle.__init__(self)
        self.name = name
        self.config = config
        self.node = node
        self.graph = graph
        self.vehicle_traits = vehicle_traits
        self.transforms = transforms
        self.map_name = map_name
        self.rmf_map_name = rmf_map_name

        # Get the index of the charger waypoint
        waypoint = self.graph.find_waypoint(charger_waypoint)
        # assert waypoint, f"Charger waypoint {charger_waypoint} does not exist in the navigation graph"
        if waypoint is None:
            node.get_logger().error(f"Charger waypoint {charger_waypoint} does not exist in the navigation graph")
            return
        self.charger_waypoint_index = waypoint.index
        self.charger_is_set = False
        self.update_frequency = update_frequency
        self.update_handle = None # RobotUpdateHandle
        self.battery_soc = 1.0
        self.api = api
        self.position = position # (x,y,theta) in RMF coordinates (meters, radians)
        self.initialized = False
        self.state = EcobotState.IDLE
        self.adapter = adapter

        self.requested_waypoints = [] # RMF Plan waypoints
        self.remaining_waypoints = []
        self.path_finished_callback = None
        self.next_arrival_estimator = None
        self.path_index = 0
        self.docking_finished_callback = None
        self.perform_filtering = self.config["filter_waypoints"]

        # RMF location trackers
        self.last_known_lane_index = None
        self.last_known_waypoint_index = None
        self.on_waypoint = None # if robot is waiting at a waypoint. This is a Graph::Waypoint index
        self.on_lane = None # if robot is travelling on a lane. This is a Graph::Lane index
        self.target_waypoint = None # this is a Plan::Waypoint
        self.dock_waypoint_index = None # The graph index of the waypoint the robot is currently docking into

        # Threading variables
        self._lock = threading.Lock()
        self._follow_path_thread = None
        self._quit_path_event = threading.Event()
        self._dock_thread = None
        self._quit_dock_event = threading.Event()

        self.action_execution = None
        self.participant = None

        print(f"{self.name} is starting at: [{self.position[0]:.2f}, {self.position[1]:.2f}, {self.position[2]:.2f}")

        self.state_update_timer = self.node.create_timer(
            1.0 / self.update_frequency,
            self.update)

        # The Ecobot robot has various cleaning modes. Here we turn off the
        # cleaning systems. These will be activated only during cleaning
        self.api.set_cleaning_mode(self.config['inactive_cleaning_config'])
        self.initialized = True

    def clear(self):
        with self._lock:
            print(f"Clearing internal states with current waypoint {self.on_waypoint}")
            self.requested_waypoints = []
            self.remaining_waypoints = []
            self.path_finished_callback = None
            self.next_arrival_estimator = None
            self.docking_finished_callback = None
            # self.target_waypoint = None
            self.state = EcobotState.IDLE

    # override function
    def stop(self):
        # Stop motion of the robot. Tracking variables should remain unchanged.
        while True:
            self.node.get_logger().info("Requesting robot to stop...")
            if self.api.stop():
                break
            time.sleep(1.0)

    # override function
    def follow_new_path(
        self,
        waypoints,
        next_arrival_estimator,
        path_finished_callback):

        if self._follow_path_thread is not None:
            self._quit_path_event.set()
            if self._follow_path_thread.is_alive():
                self._follow_path_thread.join()
            self._follow_path_thread = None
            self.clear()
        self._quit_path_event.clear()

        print("Received new path to follow...")
        # The list of waypoints received will contain waypoints with similar
        # positions but different orientations. The ecobot robot is unable to
        # turn on the spot and hence commands to re-orient the robot at the same
        # position will cause the robot to take a long route to "steer" into the
        # desired orientation. Hence, we filter waypoints such that we retain
        # only the last waypoint for a unique [x,y] position. It is important to
        # keep the last as it contains the duration the robot is expected to wait
        # at the waypoint.
        # wait:(index, waypoint), entries:[(index, waypoint)]
        wait, entries = self.filter_waypoints(waypoints)
        ########################################################################
        # HELPFUL PRINTOUTS
        index = 0
        print(f"  [i]:{self.position}")
        for w in waypoints:
            print(f"  [{index}]:{w.position},{w.time}")
            index = index +1
        index = 0
        print("Filtered waypoints:")
        if wait is not None:
            print(f" i[{wait.index}]:{wait.position},{wait.time}")
        else:
            print("  i[None]")
        for e in entries:
            print(f"  [{e.index}]:{e.position},{e.time}")
        ########################################################################

        self.remaining_waypoints = copy.copy(entries)
        assert next_arrival_estimator is not None
        assert path_finished_callback is not None
        self.next_arrival_estimator = next_arrival_estimator
        self.path_finished_callback = path_finished_callback

        # Make the robot wait at its current position
        if (wait is not None):
            self.path_index = wait.index
            self.target_waypoint = wait
            self.state = EcobotState.WAITING
            with self._lock:
                if (self.target_waypoint.graph_index is not None):
                    self.on_waypoint = self.target_waypoint.graph_index
                else:
                    self.on_waypoint = None # we are still on a lane
                self.last_known_waypoint_index = self.on_waypoint

        def _follow_path():
            target_pose = []
            while (self.remaining_waypoints or self.state == EcobotState.MOVING or self.state == EcobotState.WAITING):
                # Check if we need to abort
                if self._quit_path_event.is_set():
                    self.node.get_logger().info("Aborting previously followed path")
                    return
                # State machine
                if self.state == EcobotState.IDLE:
                    print("State: Idle...")
                    # Assign the next waypoint
                    self.target_waypoint = self.remaining_waypoints[0]
                    self.path_index = self.remaining_waypoints[0].index
                    # Move robot to next waypoint
                    target_pose =  self.target_waypoint.position
                    [x,y, yaw] = self.transforms.to_robot_map(target_pose[:3])
                    theta = math.degrees(yaw)

                    print(f"Requesting robot to navigate to "
                        f"[{self.path_index}][{x:.0f},{y:.0f},{theta:.0f}] "
                        f"grid coordinates and [{target_pose[0]:.2f}. {target_pose[1]:.2f}, {target_pose[2]:.2f}] "
                        f"RMF coordinates...")

                    response = self.api.navigate([x, y, theta], self.map_name)
                    if response:
                        self.remaining_waypoints = self.remaining_waypoints[1:]
                        self.state = EcobotState.MOVING
                        # with self._lock:
                        #     if self.on_waypoint is not None: # robot starts at a graph waypoint
                        #         self.last_known_waypoint_index = self.on_waypoint
                    else:
                        self.node.get_logger().info(f"Robot {self.name} failed to navigate to [{x:.0f}, {y:.0f}, {theta:.0f}] grid coordinates. Retrying...")
                        time.sleep(1.0)

                elif self.state == EcobotState.WAITING:
                    time.sleep(0.1)
                    time_now = self.adapter.now()
                    with self._lock:
                        if self.target_waypoint is not None:
                            waypoint_wait_time = self.target_waypoint.time
                            if (waypoint_wait_time < time_now):
                                self.state = EcobotState.IDLE
                                # self.target_waypoint = None
                            else:
                                if self.path_index is not None:
                                    self.node.get_logger().info(f"Waiting for {(waypoint_wait_time - time_now).seconds}s")
                                    self.next_arrival_estimator(self.path_index, timedelta(seconds=0.0))


                elif self.state == EcobotState.MOVING:
                    time.sleep(0.5)
                    self.node.get_logger().info("Moving...")
                    # Check if we have reached the target
                    with self._lock:
                        if (self.api.navigation_completed()):
                            print(f"Robot [{self.name}] has reached its target waypoint")
                            self.state = EcobotState.WAITING
                            if (self.target_waypoint.graph_index is not None):
                                self.on_waypoint = self.target_waypoint.graph_index
                                self.last_known_waypoint_index = self.on_waypoint
                            else:
                                self.on_waypoint = None # we are still on a lane
                        else:
                            # Update the lane the robot is on
                            lane = self.get_current_lane()
                            if lane is not None:
                                self.on_waypoint = None
                                self.on_lane = lane
                            else:
                                # The robot may either be on the previous
                                # waypoint or the target one
                                if self.target_waypoint.graph_index is not None and self.dist(self.position, target_pose) < 0.5:
                                    self.on_waypoint = self.target_waypoint.graph_index
                                elif self.last_known_waypoint_index is not None and \
                                        self.dist(self.position, self.graph.get_waypoint(self.last_known_waypoint_index).location) < 0.5:
                                    self.on_waypoint = self.last_known_waypoint_index
                                else:
                                    self.on_lane = None # update_off_grid()
                                    self.on_waypoint = None
                            # Find next arrival estimate
                            if self.path_index is not None:
                                dist_to_target = self.dist(self.position, target_pose)
                                ori_delta = abs(abs(self.position[2]) - abs(target_pose[2]))
                                if ori_delta > np.pi:
                                    ori_delta = ori_delta - (2 * np.pi)
                                if ori_delta < -np.pi:
                                    ori_delta =  (2 * np.pi) + ori_delta
                                duration = dist_to_target/self.vehicle_traits.linear.nominal_velocity +\
                                  ori_delta/self.vehicle_traits.rotational.nominal_velocity
                                self.next_arrival_estimator(self.path_index, timedelta(seconds=duration))
            self.path_finished_callback()
            self.node.get_logger().info(f"Robot {self.name} has successfully navigated along requested path.")
            # self.target_waypoint = None
        self._follow_path_thread = threading.Thread(
            target=_follow_path)
        self._follow_path_thread.start()

    # override function
    def dock(
        self,
        dock_name,
        docking_finished_callback):

        self._quit_dock_event.clear()
        if self._dock_thread is not None:
            self._dock_thread.join()

        assert docking_finished_callback is not None
        self.docking_finished_callback = docking_finished_callback

        # Get the waypoint that the robot is trying to dock into. The dock_name and waypoint_name must match
        self.node.get_logger().info(f"[DOCK] Start docking to charger with dock param: {dock_name}")

        # NOTE: Now only assume that dock is only used by during charging.
        self.dock_waypoint_index = self.charger_waypoint_index
        self.on_waypoint = self.dock_waypoint_index
        def _dock():
            # TODO, clean up implementation of dock
            # Check if the dock waypoint is a charger or cleaning zone and call the
            # appropriate API. Charger docks should have dock_name as "charger_<robot_name>"
            # todo [YV]: Map dock names to API callbacks instead of checking substrings
            while True:
                self.node.get_logger().info(f"Requesting robot {self.name} to charge at {dock_name}")
                # The Ecobot75 requires a TF path to dock to charging location. This path is
                # named the same as the dock name (charger_ecobot75_x)
                if self.name[:8] == "ecobot75":
                    self.api.set_cleaning_mode(self.config['inactive_cleaning_config'])
                    if self.api.start_task(dock_name, self.map_name):
                        break
                # For Ecobot40 and 50, we will use the navigate_to_waypoint API to
                # dock into the charger
                else:
                    if self.api.navigate_to_waypoint(dock_name, self.map_name):
                        break
                time.sleep(1.0)
            while (not self.api.task_completed()):
                # Check if we need to abort
                if self._quit_dock_event.is_set():
                    self.node.get_logger().info("Aborting docking")
                    return
                time.sleep(0.5)
            # Here we assume that the robot has successfully reached waypoint with name same as dock_name
            with self._lock:
                self.on_waypoint = self.dock_waypoint_index
                self.dock_waypoint_index = None
                self.docking_finished_callback()
                self.node.get_logger().info("Docking completed")
            self.api.set_cleaning_mode(self.config['inactive_cleaning_config'])

        self._dock_thread = threading.Thread(target=_dock)
        self._dock_thread.start()

    def get_position(self):
        ''' This helper function returns the live position of the robot in the
        RMF coordinate frame'''
        position = self.api.position()
        if position is not None:
            x,y,theta = self.transforms.to_rmf_map([position[0],position[1], math.radians(position[2])])
            print(f"Convert pos from {position} grid coor to {x},{y}, {theta} rmf coor")
            return [x,y,theta]
        else:
            self.node.get_logger().error("Unable to retrieve position from robot. Returning last known position...")
            return self.position

    def get_battery_soc(self):
        battery_soc = self.api.battery_soc()
        if battery_soc is not None:
            return battery_soc
        else:
            self.node.get_logger().error("Unable to retrieve battery data from robot")
            return self.battery_soc

    def update(self):
        self.position = self.get_position()
        self.battery_soc = self.get_battery_soc()
        if self.update_handle is not None:
            self.update_state()

    # call this when starting cleaning execution
    def _action_executor(self, 
                        category: str,
                        description: dict,
                        execution:
                        adpt.robot_update_handle.ActionExecution):
        with self._lock:
            # only accept clean category
            assert(category == "clean")
            attempts = 0
            while attempts < 2:
                self.node.get_logger().info(f"Requesting robot {self.name} to clean {description}")
                self.api.set_cleaning_mode(self.config['active_cleaning_config'])
                if self.api.start_clean(description["clean_task_name"], self.map_name):
                    self.start_clean_action_time = self.adapter.now()
                    self.on_waypoint = None
                    self.on_lane = None
                    self.action_execution = execution
                    # robot moves slower during cleaning
                    self.vehicle_traits.linear.nominal_velocity *= 0.2
                    break
                attempts+=1
                time.sleep(1.0)
            self.node.get_logger().warn(f"Failed to initiate cleaning action for robot {self.name}")
            # TODO: issue error ticket
            execution.error("Failed to initiate cleaning action for robot {self.name}")

    def update_state(self):
        self.update_handle.update_battery_soc(self.battery_soc)

        # only run this during init
        if not self.charger_is_set:
            if ("max_delay" in self.config.keys()):
                max_delay = self.config["max_delay"]
                print(f"Setting max delay to {max_delay}s")
                self.update_handle.set_maximum_delay(max_delay)
            if (self.charger_waypoint_index < self.graph.num_waypoints):
                self.update_handle.set_charger_waypoint(self.charger_waypoint_index)
            else:
                self.node.get_logger().info("Invalid waypoint supplied for charger. Using default nearest charger in the map")
            self.update_handle.set_action_executor(self._action_executor)
            self.charger_is_set = True
            self.participant = self.update_handle.get_unstable_participant()

        # Update states and positions
        with self._lock:
            if (self.action_execution):
                print("Executing action, cleaning state")
                # check if action is completed/killed/canceled
                action_ok = self.action_execution.okay()
                if self.api.task_completed() or not action_ok:
                    if action_ok:
                        self.node.get_logger().info("Cleaning is completed")
                        self.action_execution.finished()
                    else:
                        self.node.get_logger().warn("cleaning task is killed/canceled")
                    self.api.set_cleaning_mode(self.config['inactive_cleaning_config'])
                    self.action_execution = None
                    self.start_clean_action_time = None
                    self.vehicle_traits.linear.nominal_velocity *= 5 # change back vel
                else:
                    assert(self.participant)
                    assert(self.start_clean_action_time)
                    total_action_time = timedelta(hours=1.0)  #TODO: populate actual total time``
                    remaining_time = total_action_time - (self.adapter.now() - self.start_clean_action_time)
                    print(f"Still cleaning =) Estimated remaining time: [{remaining_time}]")
                    self.action_execution.update_remaining_time(remaining_time)

                    # create a short segment of the trajectory according to robot current heading
                    _x, _y, _theta = self.position
                    fake_pos = [_x + math.cos(_theta)*2.5, _y + math.sin(_theta)*2.5, _theta]
                    positions = [self.position, fake_pos] 

                    # update robot current location with lost position, TODO: fix this as it is
                    # using the front() of the starts list, it the first start might not be the nearest
                    # self.update_handle.update_lost_position(
                    #     self.rmf_map_name, self.position,
                    #     max_merge_waypoint_distance = 1.0, max_merge_lane_distance=15.0)

                    ## Get Closest point on graph and update location
                    closest_wp = self.get_closest_waypoint_idx(self.position)
                    if closest_wp:
                        self.update_handle.update_off_grid_position(
                            self.position, closest_wp)
                    else:
                        self.node.get_logger().error(f"Cant find closest waypoint!")
                        self.update_handle.update_off_grid_position(
                            self.position, self.target_waypoint.graph_index)

                    trajectory = schedule.make_trajectory(
                        self.vehicle_traits,
                        self.adapter.now(),
                        positions)
                    route = schedule.Route(self.rmf_map_name,trajectory)
                    self.participant.set_itinerary([route])
            elif (self.on_waypoint is not None): # if robot is on a waypoint
                print(f"[update] Calling update_current_waypoint() on waypoint with " \
                      f"pose[{self.position_str()}] and waypoint[{self.on_waypoint}]")
                self.update_handle.update_current_waypoint(
                    self.on_waypoint, self.position[2])
            elif (self.on_lane is not None): # if robot is on a lane
                # We only keep track of the forward lane of the robot. However, when
                # calling this update it is recommended to also pass in the reverse
                # lane so that the planner does not assume the robot can only head
                # forwards. This would be helpful when the robot is still rotating on a waypoint.
                forward_lane = self.graph.get_lane(self.on_lane)
                entry_index = forward_lane.entry.waypoint_index
                exit_index = forward_lane.exit.waypoint_index
                reverse_lane = self.graph.lane_from(exit_index, entry_index)
                lane_indices = [self.on_lane]
                if reverse_lane is not None: # Unidirectional graph
                    lane_indices.append(reverse_lane.index)
                print(f"[update] Calling update_current_lanes() with pose[{self.position_str()}] "\
                      f"and lanes:{lane_indices}")
                self.update_handle.update_current_lanes(
                    self.position, lane_indices)
            elif (self.dock_waypoint_index is not None):
                print(f"[update] Calling update_off_grid_position() dock with pose" \
                      f"[{self.position_str()}] and waypoint[{self.dock_waypoint_index}]")
                self.update_handle.update_off_grid_position(
                    self.position, self.dock_waypoint_index)
            elif (self.target_waypoint is not None and self.target_waypoint.graph_index is not None): # if robot is merging into a waypoint
                print(f"[update] Calling update_off_grid_position() with pose " \
                      f"[{self.position_str()}] and waypoint[{self.target_waypoint.graph_index}]")
                self.update_handle.update_off_grid_position(
                    self.position, self.target_waypoint.graph_index)
            else: # if robot is lost
                print("[update] Calling update_lost_position()")
                self.update_handle.update_lost_position(
                    self.rmf_map_name, self.position)

    def position_str(self):
        return f"{self.position[0]:.2f}, {self.position[1]:.2f}, {self.position[2]:.2f}"

    def get_closest_waypoint_idx(
            self, position: Tuple[float, float, float],
            max_merge_lane_distance = 40.0
        ) -> Optional[int]:
        """
        Find closest graph waypoint to the provided position, return waypoint idx
        """
        starts = plan.compute_plan_starts(
            self.graph,
            self.rmf_map_name,
            position,
            self.adapter.now(),
            max_merge_waypoint_distance = 1.0,
            max_merge_lane_distance = max_merge_lane_distance)
        if not starts:
            return None
        cx, cy, _ = position
        closest_idx = -1
        closest_dist_sq = max_merge_lane_distance**2
        for s in starts:
            idx = s.waypoint
            x, y = self.graph.get_waypoint(idx).location
            dist_sq = (x-cx)**2 + (y-cy)**2
            if (dist_sq < closest_dist_sq):
                closest_idx = idx
                closest_dist_sq = dist_sq
        return closest_idx

    def get_current_lane(self):
        def projection(current_position,
                       target_position,
                       lane_entry,
                       lane_exit):
            px, py , _ = current_position
            p = np.array([px, py])
            t = np.array(target_position)
            entry = np.array(lane_entry)
            exit = np.array(lane_exit)
            return np.dot(p - t, exit - entry)

        if self.target_waypoint == None:
            return None
        approach_lanes = self.target_waypoint.approach_lanes
        if approach_lanes == None or len(approach_lanes) == 0: # Spin on the spot
            return None
        # Determine which lane the robot is currently on
        for lane_index in approach_lanes:
            lane = self.graph.get_lane(lane_index)
            p0 = self.graph.get_waypoint(lane.entry.waypoint_index).location
            p1 = self.graph.get_waypoint(lane.exit.waypoint_index).location
            p = self.position
            before_lane = projection(p, p0, p0, p1) < 0.0
            after_lane = projection(p, p1, p0, p1) >= 0.0
            if not before_lane and not after_lane: # The robot is on this lane
                return lane_index
        return None

    # TODO: remove the usage of sqrt for efficiency
    def dist(self, A, B):
        ''' Euclidian distance between A(x,y) and B(x,y)'''
        assert(len(A) > 1)
        assert(len(B) > 1)
        return math.sqrt((A[0] - B[0])**2 + (A[1] - B[1])**2)

    def filter_waypoints(self, wps:list, threshold = 1.0):
        ''' Return filtered PlanWaypoints'''

        assert(len(wps) > 0)
        first = None
        second = []
        threshold = 1.0
        last_pose = copy.copy(self.position)
        waypoints = []
        for i in range(len(wps)):
            waypoints.append(PlanWaypoint(i, wps[i]))

        # We assume the robot will backtack if the first waypoint in the plan
        # is behind the current position of the robot
        first_position = waypoints[0].position
        if len(waypoints) > 2 and self.dist(first_position, last_pose) > threshold:
            changed = False
            index = 0
            while (not changed):
                if self.dist(waypoints[index].position, first_position) > 0.1:
                    changed = True
                    break
                waypoints[index].position = last_pose
                index = index + 1

        if (self.perform_filtering is False):
            return (first, waypoints)

        changed = False
        # Find the first waypoint
        index = 0
        while (not changed and index < len(waypoints)):
            if (self.dist(last_pose,waypoints[index].position) < threshold):
                first = waypoints[index]
                last_pose = waypoints[index].position
            else:
                break
            index = index + 1

        while (index < len(waypoints)):
            parent_index = copy.copy(index)
            wp = waypoints[index]
            if (self.dist(wp.position, last_pose) >= threshold):
                changed = False
                while (not changed):
                    next_index = index + 1
                    if (next_index < len(waypoints)):
                        if (self.dist(waypoints[next_index].position, waypoints[index].position) < threshold):
                            if (next_index == len(waypoints) - 1):
                                # append last waypoint
                                changed = True
                                wp = waypoints[next_index]
                                wp.approach_lanes = waypoints[parent_index].approach_lanes
                                second.append(wp)
                        else:
                            # append if next waypoint changes
                            changed = True
                            wp = waypoints[index]
                            wp.approach_lanes = waypoints[parent_index].approach_lanes
                            second.append(wp)
                    else:
                        # we add the current index to second
                        changed = True
                        wp = waypoints[index]
                        wp.approach_lanes = waypoints[parent_index].approach_lanes
                        second.append(wp)
                    last_pose = waypoints[index].position
                    index = next_index
            else:
                index = index + 1

        return (first, second)
