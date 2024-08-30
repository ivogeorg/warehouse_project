### `warehouse_project`

#### Preliminaries

```
cd ~/ros2_ws/src
git clone https://github.com/ivogeorg/warehouse_project.git
cd ~/ros2_ws
colcon build
source install/setup.bash
```  

#### Task 1 - Mapping

##### _Task 1 - Launching_

1. _(personal note)_ The `odom` frame in the real warehouse lab is `robot_odom`, while the `odom` topic is unchanged. Make sure that in [`cartographer_slam/config/cartographer.lua`](cartographer_slam/config/cartographer.lua) the following lines are correct:
   ```
   published_frame = "robot_odom",
   odom_frame = "robot_odom",
   ```
2. `ros2 launch cartographer_slam cartographer.launch.py`
3. `rviz2 -d ~/ros2_ws/src/warehouse_project/cartographer_slam/rviz/config.rviz`
4. Move around the robot with the joystick.
5. `ros2 launch map_server map_server.launch.py map_file:=warehouse_map_real.yaml`

##### _Task 1 - Notes_

1. In the `map_server` test, the originally created map will be used, not the one that might have been accumulated during the `cartographer_slam` test.
2. In the cartographer phase, the `map` topic and `map` frame are published by `occupancy_grid_node`.
3. In the map server phase, the `map` topic is published by `map_server`. No TF frames are published by `map_server`.

#### Task 2 - Localization

##### _Task 2 - Launching_

1. `ros2 launch localization_server localization.launch.py map_file:=warehouse_map_real.yaml`
2. In Rviz2, set a **2D Pose Estimate**
3. `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/robot/cmd_vel`

##### _Task 2 - Notes_

1. `map_server` publishes the `map` topic. `amcl` subscribes to `map`.
2. `amcl` publishes the `map` TF frame, after an initial pose (**2D Pose Estimate**) is specified.

#### Task 3 - Navigation

1. `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/robot/cmd_vel`
2. In Rviz2, set a **2D Pose Estimate**
3. `ros2 launch path_planner_server pathplanner.launch.py`
4. In Rviz2, set a **2D Goal Pose**

#### Configuration

| Parameter | Location | Check | Simulator | Real robot | Documentation | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| `cmd_vel` | ROS2 topic | `ros2 topic list \| grep cmd_vel` | `/diffbot_base_controller/cmd_vel_unstamped` | | |
| `odom` | ROS2 topic | `ros2 topic list \| grep odom`, `ros2 topic echo /odom` | `odom` | | | |
| `odom` | TF frame | Rviz2, `ros2 run tf2_tools view_frames` | `odom` | | | |
| Robot base link | Rviz2 | Rviz2, `ros2 run tf2_tools view_frames` | `robot_base_footprint` | | | |
| Map topic | ROS2 topic | Rviz2, `ros2 topic list \| grep map`, `ros2 topic info map -v` | `map` (`cartographer_occupancy_grid_node`) | | | |
| Map frame | TF frame | Rviz2, `ros2 run tf2_tools view_frames` | `map` (`cartographer_occupancy_grid_node`) | | | Assuming the publisher of the `map` topic |
| Occupancy grid node | Package under `cartographer_ros`, [launch file](cartographer_slam/launch/cartographer.launch.py) | `sudo find / -name "cartographer_ros"` | `cartographer_occupancy_grid_node` | | | |
| `map_frame` | Param in [`cartographer.lua`](cartographer_slam/config/cartographer.lua) | File | `"map"` | `"map"` | | |
| `tracking_frame` | Param in [`cartographer.lua`](cartographer_slam/config/cartographer.lua) | File | `"robot_base_footprint"` | `"robot_base_footprint"` | | |
| `published_frame` | Param in [`cartographer.lua`](cartographer_slam/config/cartographer.lua) | File | `"odom"` | `"robot_odom"` | | |
| `odom_frame` | Param in [`cartographer.lua`](cartographer_slam/config/cartographer.lua) | File | `"odom"` | `"robot_odom"` | | |
| `'use_sim_time'` | Param in launch files | Files: [map](map_server/launch/map_server.launch.py), [loc](localization_server/launch/localization.launch.py), [path](path_planner_server/launch/pathplanner.launch.py) | `True` | `False` | | |
| `base_frame_id` | Param in [`amcl_config.yaml`](localization_server/config/amcl_config.yaml) | File | `"robot_base_footprint"` |`"robot_base_footprint"` | | | 
| `global_frame_id` | Param in [`amcl_config.yaml`](localization_server/config/amcl_config.yaml) | File | `"map"` | `"map"` | | | 
| `odom_frame_id` | Param in [`amcl_config.yaml`](localization_server/config/amcl_config.yaml) | File | `"odom"` | `"robot_odom"` | |
| `global_frame` | Param in [`planner_server.yaml`](path_planner_server/config/planner_server.yaml) | File | `map` | `map` | |
| `robot_base_frame` | Param in [`planner_server.yaml`](path_planner_server/config/planner_server.yaml) | File | `robot_base_footprint` | `robot_base_footprint` | | 
| `vel_cmd` topic | ROS2 topic | `ros2 topic list \| grep cmd_vel` | `/diffbot_base_controller/cmd_vel_unstamped` | | |
| `robot_description` topic | ROS2 topic | `ros2 topic list \| grep robot_description` | `/rb1_robot/robot_description` | | |


#### `'use_sim_time'`

1. A ROS parameter.
2. Specified in config files as
   ```
   ros__parameters:
     use_sim_time: True
   ```
3. Specified in `parameters` for node launch descriptions. _**Question:** Does the launch description `parameters` value override the config file value?_
4. List parameters with `ros2 param list`. Shown by node.
5. Check as follows `ros2 param get /position_controller use_sim_time` where `position_controller` is a node and `use_sime_time` is a parameter of the node.  

**Conclusion:** Config files take precedence over launch description parameter specifications. If specified in the config file, changing the value in the node launch description has no effect. Conversely, if the parameter is not specified in the config file, it is set to the launch description value.
