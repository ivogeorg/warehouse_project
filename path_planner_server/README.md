### `path_planner_server`

#### Implementation notes

1. Error on launch of recoveries server:
   ```
   [ERROR] [launch]: Caught exception in launch (see debug for traceback): "package 'nav2_recoveries' not found, searching: ...
   ```

Solution:
1. Package name is `nav2_behaviors`
2. Executable is `behavior_server`
3. Recoveries are behaviors now (in yaml config file):
   ```
   recovery_plugins: ["spin", "backup", "wait"]
   spin:
     plugin: "nav2_behaviors/Spin"
   backup:
     plugin: "nav2_behaviors/BackUp"
   wait:
     plugin: "nav2_behaviors/Wait"
   ```

2. Error in path planning:
   1. The two wheel links are not in the TF tree in Rviz2 but don't appear in the Gazebo model either.
   2. A **2D Goal Pose** does not trigger a path planning process although the pose is reported in the Rviz2 stdout log. 
   3. The path planner launch reports:
      ```
      [planner_server-1] [INFO] [1724650892.070262338] [global_costmap.global_costmap]: Timed out waiting for transform from base_link to map to become available, tf error: Invalid frame ID "base_link" passed to canTransform argument source_frame - frame does not exist
      ```
   4. Toggling `    robot_model_type: "nav2_amcl::DifferentialMotionModel" # "nav2_amcl::OmniMotionModel"` has no effect on these problems.
   5. Gazebo world, robot spawning, and controller initialization doesn't report any errors.
