# Fleet Adapter Ecobot

![](../media/media/fleet_adapter_ecobot.gif)

This repository contains an RMF fleet adapter developed for [Gaussian Ecobot](https://www.gaussianrobotics.com/) cleaning robots.
It has been tested on S40, S50 and S75 variants of robots.
The adapter communicates with the robots via REST API calls.
For more information on the APIs, please reference [this document](http://download.gs-robot.com/gs_api/api.html#10206)

## Requirements
* [Open-RMF](https://github.com/open-rmf/rmf)

## Setup
* Clone this repository into a workspace
* Source Open-RMF
* `colcon build --packages-up-to fleet_adapter_ecobot --cmake-args -DCMAKE_BUILD_TYPE=Release`

## Run the fleet adapter
Ensure the robot can be pinged.

```
ros2 run fleet_adapter_ecobot fleet_adapter_ecobot -c CONFIG_FILE -n NAV_GRAPH

```

## Test the fleet adapter in simulation
The adapter can be tested in simulation with the help of the [ecobot_sim_server](fleet_adapter_ecobot/ecobot_sim_server.py).
This script emulates the API server of the robot and can be used to command robots in simulation to follow commands from Open-RMF.

First run the server
```
ros2 run fleet_adapter_ecobot ecobot_sim_server -c CONFIG_FILE -n NAV_GRAPH -p PORT

```

Then run the fleet adapter
```
ros2 run fleet_adapter_ecobot fleet_adapter_ecobot -c CONFIG_FILE -n NAV_GRAPH --USE_SIM_TIME
```

Ensure that the `base_url` in the config matches the `LOCALHOST:PORT` specified to the server.
