# Fleet Adapter Ecobot

![](../media/media/fleet_adapter_ecobot.gif)

This repository contains an RMF fleet adapter developed for [Gaussian Ecobot](https://www.gaussianrobotics.com/) cleaning robots.
It has been tested on S40, S50 and S75 variants of robots.
The adapter communicates with the robots via REST API calls.
For more information on the APIs, please reference [API 2.0 document](http://download.gs-robot.com/gs_api/api.html#1). 

> Note: The latest Gaussian API is at 3.0. Please refer to the relevant document for the latest endpoints.

## Requirements
* [Open-RMF](https://github.com/open-rmf/rmf)
* nudged: `pip3 install nudged`

## Setup
* Clone this repository into a workspace
* Source Open-RMF
* `colcon build --packages-up-to fleet_adapter_ecobot --cmake-args -DCMAKE_BUILD_TYPE=Release`

## Run the fleet adapter
Ensure the robot can be pinged.

```
ros2 run fleet_adapter_ecobot fleet_adapter_ecobot -c CONFIG_FILE -n NAV_GRAPH
```

To run the fleet adapter with [rmf-web](https://github.com/open-rmf/rmf-web/), specify the server_uri (`-s`):
```bash
ros2 run fleet_adapter_ecobot fleet_adapter_ecobot -c CONFIG_FILE -n NAV_GRAPH -s ws://localhost:8000/_internal
```

## Test the fleet adapter in "Mock Mode"
The adapter can be tested in mock mode with the help of the [TestClientAPI](fleet_adapter_ecobot/TestClientAPI.py). This class emulates the member functions of `EcobotClientAPI.py` which calls the rest apis of the robot. This "mock mode" is enabled by providing `-tf` argument.

![](../media/media/office-world-rviz.png)

Run this example in office world:
```bash
ros2 launch rmf_demos office.launch.xml run_fleet_adapters:=0
```

Then run the ecobot fleet adapter
```bash
# Note the addition of "--test_api_config_file" and "-tf"
ros2 run fleet_adapter_ecobot fleet_adapter_ecobot \
    -c src/fleet_adapter_ecobot/configs/robot_config.yaml \
    -n install/rmf_demos_maps/share/rmf_demos_maps/maps/office/nav_graphs/0.yaml \
    -s "ws://localhost:8000/_internal" \
    -tf src/fleet_adapter_ecobot/configs/test_api_config.yaml
```

Different to the simulation running on gazebo, this `TestClientAPI` mocks the behavior of the fleet adapter when receives command from RMF. Thus, the script is running on actual system wall time.

### Patrol Task

Now try to command robot to move to `Pantry`
```bash
ros2 run rmf_demos_tasks dispatch_patrol -p pantry
```

### Custom Clean Task

Send the robot to clean an area. This custom clean task is created by composing `go_to_place` and custom `perform_action`.
```bash
ros2 run rmf_demos_tasks dispatch_action -s patrol_D2 -a clean -ad '{ "clean_task_name": "clean_hallway" }'
```

### Show overlayed ecobot map
Show overlayed lidar map on rviz2 office map
```bash
ros2 launch rmf_demos map_server.launch.py map_name:=ecobot_office tx:=1.33 ty:=0.057 yaw:=-1.598
```

### Docking to Charger

Add a `dock_name` on a charger waypoint in traffic editor. This will then call the `dock()` function when the robot is approaching the charger. Note that this is not demonstrated in this demo.

## Get the Ecobot to RMF map Transformation with `traffic-editor`

To get the transformation of the robot map to rmf map, user can add a "floorplan" of a robot map. Then annotate and the corresponding "constraint-pairs", lastly `ctrl-T` to let traffic-editor calculate the respective transformation.

![](../media/media/traffic-editor-transform.png)

Specify this transformation to the `rmf_transform` in the `config.yaml` file. Note that the value of Y-axis is applied with a -ve.


Then if you wish to configure your custom waypoints in the `configs/test_api_config.yaml`, you can use rviz to obtain those points in ecobot coordinates. Run this on a separate terminal.
```bash
# first run the office map
ros2 launch rmf_demos office.launch.xml run_fleet_adapters:=0
# then run this on a seperate terminal
ros2 run fleet_adapter_ecobot clicked_point_transform -tf 1.33 0.057 -1.598 0.049
```

![](../media/media/rviz2_publish_point.png)

Subsequently, select "publish point" on rviz, then click on the respective point on the map. Immediately, the point in rmf and robot coordinate will get printed on `clicked_point_transform` terminal. These coordinates are helpful during debugging.

# TODO 
- Traffic editior
  - Add portion explaining the need to specify the 'dock_name' of the charging point in traffic editor to match the name of the charging point stored within the robot's internal database.
- Update test scripts for new API