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
import argparse
import yaml
import threading
import time

import rclpy
import rclpy.node
from rclpy.parameter import Parameter

import rmf_adapter as adpt
import rmf_adapter.vehicletraits as traits
import rmf_adapter.battery as battery
import rmf_adapter.geometry as geometry
import rmf_adapter.graph as graph
import rmf_adapter.plan as plan

from rmf_task_msgs.msg import TaskProfile, TaskType

from functools import partial

from .EcobotCommandHandle import EcobotCommandHandle
from .EcobotClientAPI import EcobotAPI
from .utils import RmfMapTransform

#------------------------------------------------------------------------------
# Helper functions
#------------------------------------------------------------------------------
def initialize_fleet(config_yaml, nav_graph_path, node, server_uri, args):
    # Profile and traits
    fleet_config = config_yaml['rmf_fleet']
    profile = traits.Profile(geometry.make_final_convex_circle(
        fleet_config['profile']['footprint']),
        geometry.make_final_convex_circle(fleet_config['profile']['vicinity']))
    vehicle_traits = traits.VehicleTraits(
        linear=traits.Limits(*fleet_config['limits']['linear']),
        angular=traits.Limits(*fleet_config['limits']['angular']),
        profile=profile)
    # Ecobot robots are unable to reverse
    vehicle_traits.differential.reversible = fleet_config['reversible']

    # Battery system
    voltage = fleet_config['battery_system']['voltage']
    capacity = fleet_config['battery_system']['capacity']
    charging_current = fleet_config['battery_system']['charging_current']
    battery_sys = battery.BatterySystem.make(voltage, capacity, charging_current)

    # Mechanical system
    mass = fleet_config['mechanical_system']['mass']
    moment = fleet_config['mechanical_system']['moment_of_inertia']
    friction = fleet_config['mechanical_system']['friction_coefficient']
    mech_sys = battery.MechanicalSystem.make(mass, moment, friction)

    # Power systems
    ambient_power_sys = battery.PowerSystem.make(
        fleet_config['ambient_system']['power'])
    tool_power_sys = battery.PowerSystem.make(
        fleet_config['cleaning_system']['power'])

    # Power sinks
    motion_sink = battery.SimpleMotionPowerSink(battery_sys, mech_sys)
    ambient_sink = battery.SimpleDevicePowerSink(battery_sys, ambient_power_sys)
    tool_sink = battery.SimpleDevicePowerSink(battery_sys, tool_power_sys)

    nav_graph = graph.parse_graph(nav_graph_path, vehicle_traits)

    # Adapter
    adapter = adpt.Adapter.make('ecobot_fleet_adapter')
    if args.use_sim_time:
        adapter.node.use_sim_time()

    assert adapter, ("Unable to initialize ecobot adapter. Please ensure "
                     "RMF Schedule Node is running")
    adapter.start()
    time.sleep(1.0)

    fleet_name = fleet_config['name']
    fleet_handle = adapter.add_fleet(fleet_name, vehicle_traits, nav_graph, server_uri)

    if not fleet_config['publish_fleet_state']:
        fleet_handle.fleet_state_publish_period(None)

    task_capabilities_config = fleet_config['task_capabilities']

    # Account for battery drain
    drain_battery = fleet_config['account_for_battery_drain']
    recharge_threshold = fleet_config['recharge_threshold']
    recharge_soc = fleet_config['recharge_soc']
    finishing_request = task_capabilities_config['finishing_request']
    node.get_logger().info(f"Finishing request: [{finishing_request}]")
    # Set task planner params
    ok = fleet_handle.set_task_planner_params(
        battery_sys,
        motion_sink,
        ambient_sink,
        tool_sink,
        recharge_threshold,
        recharge_soc,
        drain_battery,
        finishing_request)
    assert ok, ("Unable to set task planner params")

    task_capabilities = []
    if task_capabilities_config['loop']:
        node.get_logger().info(
            f"Fleet [{fleet_name}] is configured to perform Loop tasks")
        task_capabilities.append(TaskType.TYPE_LOOP)
    if task_capabilities_config['delivery']:
        node.get_logger().info(
            f"Fleet [{fleet_name}] is configured to perform Delivery tasks")
        task_capabilities.append(TaskType.TYPE_DELIVERY)
    if task_capabilities_config['clean']:
        node.get_logger().info(
            f"Fleet [{fleet_name}] is configured to perform Clean tasks")
        task_capabilities.append(TaskType.TYPE_CLEAN)

    # Callable for validating requests that this fleet can accommodate
    def _task_request_check(task_capabilities, msg: TaskProfile):
        if msg.description.task_type in task_capabilities:
            return True
        else:
            return False

    fleet_handle.accept_task_requests(
        partial(_task_request_check, task_capabilities))

    def _consider(description: dict):
        confirm = adpt.fleet_update_handle.Confirmation()

        # Currently there's no way for user to submit a robot_task_request
        # .json file via the rmf-web dashboard. Thus the short term solution
        # is to add the fleet_name info into action description. NOTE
        # only matching fleet_name action will get accepted
        if (description["category"] == "manual_control" and
            description["description"]["fleet_name"] != fleet_name):
                return confirm

        node.get_logger().warn(
            f"Accepting action: {description} ")
        confirm.accept()
        return confirm

    # Configure this fleet to perform action category
    if 'action_categories' in task_capabilities_config:
        for cat in task_capabilities_config['action_categories']:
            node.get_logger().info(
                f"Fleet [{fleet_name}] is configured"
                f" to perform action of category [{cat}]")
            fleet_handle.add_performable_action(cat, _consider)

    def updater_inserter(cmd_handle, update_handle):
        """Insert a RobotUpdateHandle."""
        cmd_handle.init_handler(update_handle)

    # Initialize robots for this fleet
    missing_robots = config_yaml['robots']

    def _add_fleet_robots():
        robots = {}
        while len(missing_robots) > 0:
            time.sleep(0.2)
            for robot_name in list(missing_robots.keys()):
                node.get_logger().debug(f"Connecting to robot: {robot_name}")
                robot_config = missing_robots[robot_name]['ecobot_config']

                if args.test_client_api:
                    from .TestClientAPI import ClientAPI
                    api = ClientAPI()
                else:
                    api = EcobotAPI(robot_config['base_url'], robot_config['cleaning_task_prefix'])

                if not api.connected:
                    continue

                robot_map_name = api.current_map()
                if robot_map_name is None:
                    node.get_logger().warn(f"Failed to get robot map name: [{robot_map_name}]")
                    continue

                # use defined transfrom param if avail, else use ref coors
                # note that the robot's map_name should be identical to the one in config
                if "rmf_transform" in config_yaml:
                    if robot_map_name not in config_yaml["rmf_transform"]:
                        assert False, f"Robot map name: {robot_map_name} isnt defined in config"
                    tx, ty, r, s = config_yaml["rmf_transform"][robot_map_name]["transform"]
                    rmf_transform = RmfMapTransform(tx, ty, r, s)
                    rmf_map_name = config_yaml["rmf_transform"][robot_map_name]['rmf_map_name']
                else:
                    rmf_transform = RmfMapTransform()
                    rmf_coordinates = config_yaml['reference_coordinates']['rmf']
                    ecobot_coordinates = config_yaml['reference_coordinates']['ecobot']
                    rmf_map_name = config_yaml['reference_coordinates']['rmf_map_name']
                    mse = rmf_transform.estimate(ecobot_coordinates, rmf_coordinates)
                    print(f"Coordinate transformation error: {mse}")

                print(f"Coordinate Transform from [{robot_map_name}] to [{rmf_map_name}]")
                tx, ty, r, s = rmf_transform.to_robot_map_transform()
                print(f"RMF to Ecobot transform :: trans [{tx}, {ty}]; rot {r}; scale {s}")
                tx, ty, r, s = rmf_transform.to_rmf_map_transform()
                print(f"Ecobot to RMF transform :: trans [{tx}, {ty}]; rot {r}; scale {s}")

                # get robot coordinates and transform to rmf_coor
                ecobot_pos = api.position()
                if ecobot_pos is None:
                    node.get_logger().warn(f"Failed to get [{robot_name}] position")
                    continue

                node.get_logger().info(f"Initializing robot: {robot_name}")
                rmf_config = missing_robots[robot_name]['rmf_config']

                starts = []
                time_now = adapter.now()

                x,y,_ = rmf_transform.to_rmf_map([ecobot_pos[0],ecobot_pos[1], 0])
                position = [x, y, 0]

                # Identify the current location of the robot in rmf's graph
                node.get_logger().info(
                    f"Running compute_plan_starts for robot: "
                    f"{robot_name}, with pos: {position}")
                starts = plan.compute_plan_starts(
                    nav_graph,
                    rmf_map_name,
                    position,
                    time_now,
                    max_merge_waypoint_distance = 1.0,
                    max_merge_lane_distance = rmf_config["max_merge_lane_distance"])

                if starts is None or len(starts) == 0:
                    node.get_logger().error(
                        f"Unable to determine StartSet for {robot_name} "
                        f"with map {rmf_map_name}")
                    continue

                robot = EcobotCommandHandle(
                    name=robot_name,
                    config=robot_config,
                    node=node,
                    graph=nav_graph,
                    vehicle_traits=vehicle_traits,
                    transforms=rmf_transform,
                    map_name=robot_map_name,
                    rmf_map_name=rmf_map_name,
                    position=position,
                    charger_waypoint=rmf_config['charger']['waypoint'],
                    update_frequency=rmf_config.get(
                        'robot_state_update_frequency', 1),
                    adapter=adapter,
                    api=api,
                    max_merge_lane_distance=rmf_config["max_merge_lane_distance"])

                robots[robot_name] = robot
                # Add robot to fleet
                fleet_handle.add_robot(robot,
                                        robot_name,
                                        profile,
                                        starts,
                                        partial(updater_inserter,
                                                robot))
                node.get_logger().info(
                    f"Successfully added new robot: {robot_name}")
                del missing_robots[robot_name]
        return

    add_robots = threading.Thread(target=_add_fleet_robots, args=())
    add_robots.start()
    return adapter


#------------------------------------------------------------------------------
# Main
#------------------------------------------------------------------------------
def main(argv=sys.argv):
    # Init rclpy and adapter
    rclpy.init(args=argv)
    adpt.init_rclcpp()
    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    parser = argparse.ArgumentParser(
        prog="fleet_adapter_ecobot",
        description="Configure and spin up fleet adapter for Gaussian Ecobot robots ")
    parser.add_argument("-c", "--config_file", type=str, required=True,
                        help="Path to the config.yaml file for this fleet adapter")
    parser.add_argument("-n", "--nav_graph", type=str, required=True,
                    help="Path to the nav_graph for this fleet adapter")
    parser.add_argument("-s", "--server_uri", type=str, required=False, default="",
                    help="URI of the api server to transmit state and task information.")
    parser.add_argument("--use_sim_time", action="store_true",
                    help='Use sim time for testing offline, default: false')
    parser.add_argument("--test_client_api", action="store_true",
                    help='Use Test Client api to test instead of ecobot api')
    args = parser.parse_args(args_without_ros[1:])
    print(f"Starting ecobot fleet adapter...")

    config_path = args.config_file
    nav_graph_path = args.nav_graph

    # Load config and nav graph yamls
    with open(config_path, "r") as f:
        config_yaml = yaml.safe_load(f)

    # ROS 2 node for the command handle
    node = rclpy.node.Node('ecobot_command_handle')
    # Enable sim time for testing offline
    if args.use_sim_time:
        param = Parameter("use_sim_time", Parameter.Type.BOOL, True)
        node.set_parameters([param])

    if args.server_uri == "":
        server_uri = None
    else:
        server_uri = args.server_uri

    adapter = initialize_fleet(
        config_yaml,
        nav_graph_path,
        node,
        server_uri,
        args)

    # Create executor for the command handle node
    rclpy_executor = rclpy.executors.SingleThreadedExecutor()
    rclpy_executor.add_node(node)

    # Start the fleet adapter
    rclpy_executor.spin()

    # Shutdown
    node.destroy_node()
    rclpy_executor.shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main(sys.argv)
