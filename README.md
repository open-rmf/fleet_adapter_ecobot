# Ecobot Fleet Adapter

## To run fleet adapter:
```
ros2 run fleet_adapter_ecobot fleet_adapter_ecobot -c CONFIG_FILE -n NAV_GRAPH

```


## To test fleet adapter offline:
Launch the simulation environment

```
ros2 launch cag test_adapter_offline.launch.xml 
```

Start the mock ecobot fleet manager
```
python3 src/cag/fleet_adapter_ecobot/fleet_adapter_ecobot/mock_sim_server.py -c src/cag/fleet_adapter_ecobot/config.yaml -n install/cag_maps/share/cag_maps/maps/test_adapter_offline/nav_graphs/0.yaml
```

Start the ecobot fleet adapter.
Ensure the configuration file has the right set of coordinates for performing coordinate transformations

```
ros2 run fleet_adapter_ecobot fleet_adapter_ecobot -c src/cag/fleet_adapter_ecobot/config.yaml -n install/cag_maps/share/cag_maps/maps/test_adapter_offline/nav_graphs/0.yaml --use_sim_time

```