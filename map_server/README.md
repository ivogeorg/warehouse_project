### `map_server`

#### Saving the map for further use

1. The cartographer node creates the map and publishes it to the `/map` topic.
   1. Launch the cartographer.  
      ```ros2 launch cartographer_slam cartographer.launch.py```
   2. Launch the teleop node to move the robot around to create the map.  
      ```ros2 run teleop_twist_keyboard teleop_twist_keyboard```
2. While the cartographer is still running after it has created a sufficiently complete map, the map saver reads the `/map` topic and saves the map to the specified file in the directory it is run.  
   ```cd ~/ros2_ws/src/warehouse_project/map_server/config```
   ```ros2 run nav2_map_server map_saver_cli -f [filename]```

#### Single point of map serving

Note that above the created map from the `/map` topic is saved in the `map_server/config` directory. Likewise, all other nodes which require the created map, will take it from the same directory `map_server/config`. This avoids having to copy the map from one `config` to another, and eliminates potential errors.
