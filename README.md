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

1. _(personal note)_ The `odom` topic in the real warehouse lab is `robot_odom`. Make sure that in [`cartographer_slam/config/cartographer.lua`](cartographer_slam/config/cartographer.lua) the following lines are correct:
   ```
   published_frame = "robot_odom",
   odom_frame = "robot_odom",
   ```
2. `ros2 launch cartographer_slam cartographer.launch.py`
3. `rviz2 -d ~/ros2_ws/src/warehouse_project/cartographer_slam/rviz/config.rviz`
4. Move around the robot with the joystick.
5. `ros2 launch map_server map_server.launch.py map_file:=warehouse_map_real.yaml`

**_Note_:** In the `map_server` test, the originally created map will be used, not the one that might have been created in the `cartographer_slam` test.

#### Task 2 - Localization

1. `ros2 launch localization_server localization.launch.py map_file:=warehouse_map_real.yaml`
2. In Rviz2, set a **2D Pose Estimate**
3. `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/robot/cmd_vel`

#### Task 3 - Navigation

1. `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/robot/cmd_vel`
2. In Rviz2, set a **2D Pose Estimate**
3. `ros2 launch path_planner_server pathplanner.launch.py`
4. In Rviz2, set a **2D Goal Pose**
