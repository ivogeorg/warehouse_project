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

##### Task 1 - Launching

1. _(personal note)_ The `odom` frame in the real warehouse lab is `robot_odom`, while the `odom` topic is unchanged. Make sure that in [`cartographer_slam/config/cartographer.lua`](cartographer_slam/config/cartographer.lua) the following lines are correct:
   ```
   published_frame = "robot_odom",
   odom_frame = "robot_odom",
   ```
2. `ros2 launch cartographer_slam cartographer.launch.py`
3. `rviz2 -d ~/ros2_ws/src/warehouse_project/cartographer_slam/rviz/config.rviz`
4. Move around the robot with the joystick.
5. `ros2 launch map_server map_server.launch.py map_file:=warehouse_map_real.yaml`

##### Task 1 - Notes

1. In the `map_server` test, the originally created map will be used, not the one that might have been accumulated during the `cartographer_slam` test.
2. In the cartographer phase, the `map` topic and `map` frame are published by `occupancy_grid_node`.
3. In the map server phase, the `map` topic is published by `map_server`. No TF frames are published by `map_server`.

#### Task 2 - Localization

1. `ros2 launch localization_server localization.launch.py map_file:=warehouse_map_real.yaml`
2. In Rviz2, set a **2D Pose Estimate**
3. `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/robot/cmd_vel`

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
