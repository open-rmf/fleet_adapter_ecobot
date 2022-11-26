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

import numpy as np

import threading
import math
import copy
import enum
import time

from datetime import timedelta

from typing import List, Optional, Tuple

# States for EcobotCommandHandle's state machine used when guiding robot along
# a new path
class EcobotState(enum.IntEnum):
    IDLE = 0
    WAITING = 1
    MOVING = 2

# Custom wrapper for Plan::Waypoint. We use this to modify position of waypoints
# to prevent backtracking
class PlanWaypoint:
    def __init__(self, index, wp:plan.Waypoint):
        # the index of the Plan::Waypoint in the waypoints in follow_new_path
        self.index = index
        self.position = wp.position
        self.time = wp.time
        self.graph_index = wp.graph_index
        self.approach_lanes = wp.approach_lanes

##############################################################################
class EcobotCommandHandle(adpt.RobotCommandHandle):
    def __init__(self,
                 name,
                 config,
                 node,
                 graph,
                 vehicle_traits,
                 transforms,
                 charger_waypoint,
                 update_frequency,
                 adapter,
                 api,
                 max_merge_lane_distance):
        """
        :param config
            robot config defined in yaml file
        :param max_merge_lane_distance
            means how far will the robot diverge from the defined graph
        """
        adpt.RobotCommandHandle.__init__(self)
        self.name = name
        self.config = config
        self.node = node
        self.graph = graph
        self.vehicle_traits = vehicle_traits
        self.transforms = transforms
        self.max_merge_lane_distance = max_merge_lane_distance

        # Get the index of the charger waypoint
        self.charger_waypoint = self.graph.find_waypoint(charger_waypoint)
        self.update_frequency = update_frequency
        self.update_handle = None # RobotUpdateHandle
        self.battery_soc = 1.0
        self.api = api
        self.state = EcobotState.IDLE
        self.adapter = adapter

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

        # Threading variables
        self._lock = threading.Lock()
        self._follow_path_thread = None
        self._quit_path_event = threading.Event()
        self._dock_thread = None
        self._quit_dock_event = threading.Event()

        self.action_execution = None
        self.stubbornness = None
        self.in_error = False
        self.is_online = False
        self.action_category = None

        # Get the latest position (x,y,theta) in RMF coordinates (meters, radians)
        robot_pos = self.get_robot_position()
        while robot_pos is None:
            print("Not able to retrieve latest position, trying again...")
            time.sleep(2.0)
            robot_pos = self.get_robot_position()

        self.position = robot_pos
        print(f"{self.name} is starting at: [{self.position_str()}]")

        # The Ecobot robot has various cleaning modes. Here we turn off the
        # cleaning systems. These will be activated only during cleaning
        self.api.set_cleaning_mode(self.config['inactive_cleaning_config'])

    ##############################################################################
    # Init RobotUpdateHandle class member
    def init_handler(self, handle):
        self.update_handle = handle
        if ("max_delay" in self.config.keys()):
            max_delay = self.config["max_delay"]
            print(f"Setting max delay to {max_delay}s")
            self.update_handle.set_maximum_delay(max_delay)
        if self.charger_waypoint is not None:
            self.update_handle.set_charger_waypoint(self.charger_waypoint.index)
        else:
            self.node.get_logger().error(
                f"Charger waypoint {self.charger_waypoint} does not exist in the nav graph."
                " Using default nearest charger in the map")
        self.update_handle.set_action_executor(self._action_executor)
        self.participant = self.update_handle.get_unstable_participant()

        self.location_update_timer = self.node.create_timer(
            1.0 / self.update_frequency,
            self.update_location)

        # Note: only update robot status 4.5 times the update period, prevent overload
        self.status_update_timer = self.node.create_timer(
            4.5 / self.update_frequency,
            self.update_robot_status)

        self.node.get_logger().info(
            f"Start State Update with freq: {self.update_frequency}")

    ##############################################################################
    def clear(self):
        with self._lock:
            print(f"Clearing internal states with current waypoint {self.on_waypoint}")
            self.remaining_waypoints = []
            self.path_finished_callback = None
            self.next_arrival_estimator = None
            self.docking_finished_callback = None
            # self.target_waypoint = None
            self.state = EcobotState.IDLE

    ##############################################################################
    # override function
    def stop(self):
        # Stop motion of the robot. Tracking variables should remain unchanged.
        while True:
            self.node.get_logger().info("Requesting robot to stop...")
            if self.api.stop():
                break
            time.sleep(1.0)

    ##############################################################################
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
            while (self.remaining_waypoints or
                   self.state == EcobotState.MOVING or
                   self.state == EcobotState.WAITING):
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
                    [x,y, yaw] = self.transforms[self.robot_map_name]["tf"].to_robot_map(target_pose[:3])
                    theta = math.degrees(yaw)

                    print(f"Requesting robot to navigate to "
                        f"[{self.path_index}][{x:.0f},{y:.0f},{theta:.0f}] "
                        f"grid coordinates and [{target_pose[0]:.2f}. {target_pose[1]:.2f}, "
                        f"{target_pose[2]:.2f}] RMF coordinates...")

                    response = self.api.navigate([x, y, theta], self.robot_map_name)
                    if response:
                        self.remaining_waypoints = self.remaining_waypoints[1:]
                        self.state = EcobotState.MOVING
                        # with self._lock:
                        #     if self.on_waypoint is not None: # robot starts at a graph waypoint
                        #         self.last_known_waypoint_index = self.on_waypoint
                    else:
                        self.node.get_logger().info(
                            f"Robot {self.name} failed to navigate to [{x:.0f}, {y:.0f}, {theta:.0f}]" \
                            "grid coordinates. Retrying...")
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
                            elif self.path_index is not None:
                                # self.node.get_logger().info(f"Waiting for {(waypoint_wait_time - time_now).seconds}s")
                                self.next_arrival_estimator(self.path_index, timedelta(seconds=0.0))

                elif self.state == EcobotState.MOVING:
                    time.sleep(0.5)
                    self.node.get_logger().info("Moving...")
                    # Check if we have reached the target
                    with self._lock:
                        lane = self.get_current_lane()
                        if (self.api.navigation_completed()):
                            print(f"Robot [{self.name}] has reached its target waypoint")
                            self.state = EcobotState.WAITING
                            if (self.target_waypoint.graph_index is not None):
                                self.on_waypoint = self.target_waypoint.graph_index
                                self.last_known_waypoint_index = self.on_waypoint
                            else:
                                self.on_waypoint = None # we are still on a lane
                        elif lane is not None:
                            self.on_waypoint = None
                            self.on_lane = lane
                        # The robot may either be on the previous
                        # waypoint or the target one
                        elif (self.target_waypoint.graph_index is not None and 
                            self.dist(self.position, target_pose) < 0.5):
                            self.on_waypoint = self.target_waypoint.graph_index
                        elif self.last_known_waypoint_index is not None and \
                                self.dist(
                                    self.position,
                                    self.graph.get_waypoint(self.last_known_waypoint_index).location) < 0.5:
                            self.on_waypoint = self.last_known_waypoint_index
                        else:
                            self.on_lane = None # update_off_grid()
                            self.on_waypoint = None
                        # Find next arrival estimate
                        if self.path_index is not None:
                            dist_to_target = self.dist(self.position, target_pose)
                            ori_delta = abs(abs(self.position[2]) - abs(target_pose[2]))
                            ori_delta = (ori_delta + math.pi) % (2*math.pi) - math.pi  #convert to within range -pi, pi
                            duration = dist_to_target/self.vehicle_traits.linear.nominal_velocity +\
                                ori_delta/self.vehicle_traits.rotational.nominal_velocity
                            self.next_arrival_estimator(self.path_index, timedelta(seconds=duration))
            self.path_finished_callback()
            self.node.get_logger().info(f"Robot {self.name} has successfully navigated along requested path.")
            # self.target_waypoint = None
        self._follow_path_thread = threading.Thread(
            target=_follow_path)
        self._follow_path_thread.start()

    ##############################################################################
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

        # NOTE: Docking called when robot is heading back to charger
        self.target_waypoint.graph_index = self.charger_waypoint.index
        self.on_waypoint = None
        self.on_lane = None
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
                    if self.api.start_task(dock_name, self.robot_map_name):
                        break
                # For Ecobot40 and 50, we will use the navigate_to_waypoint API to
                # dock into the charger
                else:
                    if self.api.navigate_to_waypoint(dock_name, self.robot_map_name):
                        break
                time.sleep(1.0)
            while (not self.api.task_completed()):
                # Check if we need to abort
                if self._quit_dock_event.is_set():
                    self.node.get_logger().info("Aborting docking")
                    return
                time.sleep(1.0)
            # Here we assume that the robot has successfully reached waypoint with name same as dock_name
            with self._lock:
                self.on_waypoint = self.charger_waypoint.index
                self.docking_finished_callback()
                self.node.get_logger().info("Docking completed")
            self.api.set_cleaning_mode(self.config['inactive_cleaning_config'])

        self._dock_thread = threading.Thread(target=_dock)
        self._dock_thread.start()

    ##############################################################################
    def get_robot_position(self) -> List[int]:
        ''' This helper function returns the live position of the robot in the
        RMF coordinate frame
        '''
        position = self.api.position()
        map_name = self.api.current_map()

        if position is None or map_name is None:
            self.node.get_logger().error(
                "Unable to retrieve position from robot. Returning last known position...")
            self.is_online = False
            return self.position

        if map_name not in self.transforms:
            self.node.get_logger().error(
                f"Robot map name [{map_name}] is not known. return last known position.")
            self.is_online = True
            self.in_error = True
            return self.position

        self.in_error = False
        tf = self.transforms[map_name]
        x,y,theta = tf['tf'].to_rmf_map([position[0],position[1], math.radians(position[2])])
        print(f"Convert pos from {position} grid coor to {x},{y}, {theta} rmf coor")
        self.is_online = True
        # will update the member function directly
        self.robot_map_name = map_name
        self.rmf_map_name = tf['rmf_map_name']
        return [x,y,theta]

    ##############################################################################
    def get_battery_soc(self):
        battery_soc = self.api.battery_soc()
        if battery_soc is not None:
            self.is_online = True
            return battery_soc
        else:
            self.is_online = False
            self.node.get_logger().error("Unable to retrieve battery data from robot")
            return self.battery_soc

    ##############################################################################
    # call this when starting cleaning execution
    def _action_executor(self, 
                        category: str,
                        description: dict,
                        execution:
                        adpt.robot_update_handle.ActionExecution):
        with self._lock:
            # only accept clean and manual_control
            assert(category in ["clean", "manual_control"])

            self.action_category = category
            if (category == "clean"):
                attempts = 0
                self.api.set_cleaning_mode(self.config['active_cleaning_config'])
                # Will try to make max 3 attempts to start the clean task
                while True:
                    self.node.get_logger().info(
                        f"Requesting robot {self.name} to clean {description}")
                    if self.api.start_task(description["clean_task_name"], self.robot_map_name):
                        self.check_task_completion = self.api.task_completed # check api func
                        break
                    if (attempts > 3):
                        self.node.get_logger().error(
                            f"Failed to initiate cleaning action for robot [{self.name}]")
                        # TODO: kill_task() or fail the task (api is not avail in adapter)
                        execution.error("Failed to initiate cleaning action for robot {self.name}")
                        execution.finished()
                        return
                    attempts+=1
                    time.sleep(1.0)
            elif (category == "manual_control"):
                self.check_task_completion = lambda:False  # user can only cancel the manual_control

            # start Perform Action
            self.node.get_logger().warn(f"Robot [{self.name}] starts [{category}] action")
            self.start_action_time = self.adapter.now()
            self.on_waypoint = None
            self.on_lane = None
            self.action_execution = execution
            self.stubbornness = self.update_handle.unstable_be_stubborn()
            # robot moves slower during perform action
            self.vehicle_traits.linear.nominal_velocity *= 0.2

    ##############################################################################
    # Update robot state's status
    def update_robot_status(self):
        if self.api.is_charging():
            self.update_handle.override_status("charging")
        elif not self.is_online:
            self.node.get_logger().warn(f"Robot {self.name} is offline")
            self.update_handle.override_status("offline")
        elif self.in_error:
            self.update_handle.override_status("error")
        elif not self.api.is_localize():
            self.node.get_logger().warn(f"Robot {self.name} is not localized")
            self.update_handle.override_status("uninitialized")
        else:
            self.update_handle.override_status(None)

    ##############################################################################
    # This function will be called periodically to check if the action if completed
    def check_perform_action(self):
        print(f"Executing perform action [{self.action_category}]")
        # check if action is completed/killed/canceled
        action_ok = self.action_execution.okay()
        if self.check_task_completion() or not action_ok:
            if action_ok:
                self.node.get_logger().info(
                    f"action [{self.action_category}] is completed")
                self.action_execution.finished()
            else:
                self.node.get_logger().warn(
                    f"action [{self.action_category}] is killed/canceled")
            self.api.set_cleaning_mode(self.config['inactive_cleaning_config'])
            self.stubbornness.release()
            self.stubbornness = None
            self.action_execution = None
            self.start_action_time = None
            self.vehicle_traits.linear.nominal_velocity *= 5 # change back vel
            return

        # still executing perform action
        assert(self.participant)
        assert(self.start_action_time)
        total_action_time = timedelta(hours=1.0)  #TODO: populate actual total time
        remaining_time = total_action_time - (self.adapter.now() - self.start_action_time)
        print(f"Still performing action, Estimated remaining time: [{remaining_time}]")
        self.action_execution.update_remaining_time(remaining_time)

        # create a short segment of the trajectory according to robot current heading
        _x, _y, _theta = self.position
        mock_pos = [_x + math.cos(_theta)*2.5, _y + math.sin(_theta)*2.5, _theta]
        positions = [self.position, mock_pos] 

        starts = self.get_start_sets()
        if starts is not None:
            self.update_handle.update_position(starts)
        else:
            self.node.get_logger().error(f"Cant get startset during perform action")
            self.update_handle.update_off_grid_position(
                self.position, self.target_waypoint.graph_index)

        trajectory = schedule.make_trajectory(
            self.vehicle_traits,
            self.adapter.now(),
            positions)
        route = schedule.Route(self.rmf_map_name, trajectory)
        self.participant.set_itinerary([route])

    ##############################################################################
    # Get start sets, for update_position(startsets)
    def get_start_sets(self):
        return plan.compute_plan_starts(
            self.graph,
            self.rmf_map_name,
            self.position,
            self.adapter.now(),
            max_merge_waypoint_distance = 0.5,
            max_merge_lane_distance = self.max_merge_lane_distance)

    ##############################################################################
    # Update location and check cleaning action
    def update_location(self):
        self.position = self.get_robot_position()
        self.battery_soc = self.get_battery_soc()
        self.update_handle.update_battery_soc(self.battery_soc)

        ## TODO: there's a tendency that the robot stop updating the system when it is offline, why!!!
        ## TODO: clean up complex lock and multi threaded system
        ## TODO: make it single threaded, and only use api.update_position()
        ## TODO: check if waypoint is close enough, and mark navigation ask as completed.
        # Update states and positions
        with self._lock:
            if (self.action_execution):
                self.check_perform_action()
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
            # if robot is merging into a waypoint
            elif (self.target_waypoint is not None and self.target_waypoint.graph_index is not None):
                print(f"[update] Calling update_off_grid_position() with pose " \
                        f"[{self.position_str()}] and waypoint[{self.target_waypoint.graph_index}]")
                self.update_handle.update_off_grid_position(
                    self.position, self.target_waypoint.graph_index)
            else: # if unsure which waypoint the robot is near to
                starts = self.get_start_sets()
                if starts is not None:
                    print("[update] Calling generic update_position()")
                    self.update_handle.update_position(starts)
                else:
                    print("[update] Calling update_lost_position()")
                    self.update_handle.update_lost_position(
                            self.rmf_map_name, self.position)

    ########################################################################
    ## Utils
    ########################################################################

    def position_str(self):
        return f"{self.position[0]:.2f}, {self.position[1]:.2f}, {self.position[2]:.2f}"

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
            if (self.dist(last_pose,waypoints[index].position) > threshold):
                break
            first = waypoints[index]
            last_pose = waypoints[index].position
            index = index + 1

        while (index < len(waypoints)):
            parent_index = copy.copy(index)
            wp = waypoints[index]
            if (self.dist(wp.position, last_pose) >= threshold):
                changed = False
                while (not changed):
                    next_index = index + 1
                    if (next_index >= len(waypoints)):
                        # we add the current index to second
                        changed = True
                        wp = waypoints[index]
                        wp.approach_lanes = waypoints[parent_index].approach_lanes
                        second.append(wp)
                    elif (self.dist(
                            waypoints[next_index].position,
                            waypoints[index].position) >= threshold):
                        # append if next waypoint changes
                        changed = True
                        wp = waypoints[index]
                        wp.approach_lanes = waypoints[parent_index].approach_lanes
                        second.append(wp)
                    elif (next_index == len(waypoints) - 1):
                        # append last waypoint
                        changed = True
                        wp = waypoints[next_index]
                        wp.approach_lanes = waypoints[parent_index].approach_lanes
                        second.append(wp)
                    last_pose = waypoints[index].position
                    index = next_index
            else:
                index = index + 1

        return (first, second)
