### `warehouse_project`

#### Overview

Robot navigation around a simulated and real lab space with the [Robotnik RB1-Base](https://robotnik.eu/products/mobile-robots/rb-1-base/) robot. Lab space courtesy of The Construct.  

![Lab](assets/rb1_warehouse_lab.png)

#### Submission notes

1. Launching
   1. Preliminaries
      ```
      cd ~/ros2_ws/src
      git clone https://github.com/ivogeorg/warehouse_project.git
      git checkout path-planning
      cd ~/ros2_ws
      rm -fr build install log 
      colcon build --clean
      source install/setup.bash
      ```
   2. Cartographer SLAM
      ```
      cd ~/ros2_ws
      source install/setup.bash
      ros2 launch cartographer_slam cartographer.launch.py
      ```
   3. Map server
      ```
      cd ~/ros2_ws
      source install/setup.bash
      ros2 launch map_server map_server.launch.py map_file:=warehouse_map_real.yaml
      ```
   4. Localization server
      ```
      cd ~/ros2_ws
      source install/setup.bash
      ros2 launch localization_server localization.launch.py map_file:=warehouse_map_real.yaml
      ```
   5. Navigation (aka Path planner server)
      ```
      cd ~/ros2_ws
      source install/setup.bash
      ros2 launch path_planner_server navigation.launch.py map_file:=warehouse_map_real.yaml
      ```
2. Notes
   1. The working implementation is on the branch `path-planning`. Please, do `git checkout` before building and running.
   2. Each of the launches includes an Rviz2 node configured for it.
   3. The Path Planner Server has an integraged launch file called `navigation.launch.py` which includes the `map_server`, `amcl`, `planner_server`, `controller_server`, `behavior_server`, and `bt_navigator` node, along with a lifecycle manager node.
   4. All launches are configured for the lab including `'use_sim_time': False` and `odom_frame: robot_odom`. Since the config files override the parameters specified in the launch file, the `use_sime_time` has been commented out in the config files so that it can freely be changed for all nodes with a local variable in the launch file.
   5. The robot description topic `/robot_description` to which the **Robot Model** item is set in Rviz2 does no longer have a publisher shortly after launch. I suppose that the publisher has broadcast it with _high durability_ and then exited.
   6. The width of the local costmap has been increased to 4 m to more readily visualize the local costmap colors. It looks a little strange because it doesn't seem to be radial around the robot.
3. Expected results
   1. Map server
      ![Map](assets/map-server.png)  
   2. Localization server
      ![Localization](assets/localization-server.png)  
   3. Navigation (aka Path planner server)
      ![Navigation](assets/navigation.png)  




