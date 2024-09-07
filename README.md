### `warehouse_project`

#### Overview

Robot navigation around a simulated and real lab space with the [Robotnik RB1-Base](https://robotnik.eu/products/mobile-robots/rb-1-base/) robot. Lab space courtesy of The Construct.  

![Lab](assets/rb1_warehouse_lab.png)

#### Submission notes (Checkpoint 11)

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
   2. The `use_sim_time` argument:
      1. **All launch files declare the argument `use_sim_time`**.
      2. It controls the `'use_sim_time'` node parameter.
      3. It also controls which config files the nodes will use, for the simulator (`use_sim_time:=true`) or the real-robot lab (`use_sim_time:=false`) environment.
      4. The default is `true`, that is, for the **simulator** environment. For the **lab**, please add `use_sim_time:=false` to the launch on the command line.
   3. Cartographer SLAM
      ```
      cd ~/ros2_ws
      source install/setup.bash
      ros2 launch cartographer_slam cartographer.launch.py
      ```
   4. Map server
      ```
      cd ~/ros2_ws
      source install/setup.bash
      ros2 launch map_server map_server.launch.py map_file:=warehouse_map_real.yaml
      ```
   5. Localization server
      ```
      cd ~/ros2_ws
      source install/setup.bash
      ros2 launch localization_server localization.launch.py map_file:=warehouse_map_real.yaml
      ```
   6. Navigation (aka Path planner server)
      ```
      cd ~/ros2_ws
      source install/setup.bash
      ros2 launch path_planner_server navigation.launch.py map_file:=warehouse_map_real.yaml
      ```
2. Notes
   1. The working implementation is on the branch `path-planning`. Please, do `git checkout` before building and running.
   2. Each of the launches includes an Rviz2 node configured for it.
   3. The Path Planner Server has an integraged launch file called `navigation.launch.py` which includes the `map_server`, `amcl`, `planner_server`, `controller_server`, `behavior_server`, and `bt_navigator` node, along with a lifecycle manager node.
   4. All launches are auto-configured for the simulator or the lab depending on the value of the `use_sim_time` argument.
   5. The robot description topic `/robot_description` to which the **Robot Model** item is set in Rviz2 does no longer have a publisher shortly after launch. I suppose that the publisher has broadcast it with _high durability_ and then exited.
   6. The width of the local costmap has been increased to 2 m by the rule-of-thumb for 4x the size of the robot. The diameter of RB1 is 0.5 m.
3. Expected results (lab only shown)
   1. Map server
      ![Map](assets/map-server.png)  
   2. Localization server
      ![Localization](assets/localization-server.png)  
   3. Navigation (aka Path planner server)
      ![Navigation](assets/navigation.png)  


#### Implementation notes (Checkpoint 11)

##### 1. TODO

1. Move the publisher of `base_link` from an explicit console script to a launch file node declaration.
   ```
   static_tf_pub = Node(package='tf2_ros', executable='static_transform_publisher',
                     name='static_transform_publisher', output='screen',
                     arguments=['0', '0', '0', '0', '0', '0', 'robot_base_link', 'base_link'])
   ```  


#### Implementation notes (Checkpoint 12)

##### 1. TODO

1. Positions as 4-element lists.
   1. These are different config files in the `config` directory of `nav2_apps`.
   2. [`rclpy` parameter tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html).  
2. Using `BasicNavigator` to move the following trajectories:
   1. `goToPose` from `init_position` to `loading_position`.
   2. `goThroughPoses` from having picked up the cart and backed up with it to `loading_pose_tf` through `face_shipping_position` and `shipping_position`.
   3. `goToPose` from having backed up from under the cart in the `shipping_position` to `initial_position`.
3. Publishing:
   1. Empty string to `/elevator_up` and waiting for 3.5 seconds.
   2. Empty string to `/elevator_down` and waiting for 3.5 seconds.
   3. `Polygon` to `/local_costmap/footprint`, once when the cart is picked up, and once when it is set down.
   4. Keep-out zone filters, one for sim map, and one for lab map. _What configuration can be done here that can be dictated by the value of the `use_sim_time` argument?_
4. Rewrite `cart_pick_up` service. Main script should contain client.
   1. Action sequence:
      1. Detect the reflective plates (and the supporting rods behind them). _Note that the sim cart and real lab cart are different!_
      2. Align robot with cart frame. _Robot centroid should lie on the line through the lengthwise axis of the cart._
      3. Calculate `cart_frame` and `cart_center_frame` relative to `robot_front_laser_base_link`.
      4. Publish `cart_frame` and `cart_center_frame`.
      5. Using the TF between `robot_base_link` and `cart_frame`, approach slowly `cart_frame`.
      6. Using the TF between `robot_base_link` and `cart_center_frame`, approach slowly `cart_center_frame`.
      7. Publish once to `/elevator_up` and wait for 3.5 s.
      8. Using the coordinates of `loading_position`, create and publish TF `loading_pose_tf`.
      9. Using the TF between `robot_base_link` and `loading_pose_tf` to back up to `loading_pose_tf`.
5. Write `cart_set_down` service. Main script should contain the client.
      1. Publish once to `/elevator_down` and wait for 3.5 s.
      8. Using the coordinates of `face_shipping_position`, create and publish TF `face_shipping_pose_tf`.
      9. Using the TF between `robot_base_link` and `face_shipping_pose_tf` to back up to `face_shipping_pose_tf`.

##### 2. Required pose parameters

1. `initial_position`: Robot in square and pointing along the positive x-axis (0 rad in world frame).
2. `loading_position`: Robot facing the crate in its stow-away position (-pi/2 rad in world frame).
3. `shipping_position`: Robot between the two shelves with the cart and facing the wall (pi/2 rad in world frame).
4. `face_shipping_position`: Robot at the intersection of the line between `init_position` and `loading_position` and the line through `shipping_position` and parallel to the y-axis. Robot facing the wall (pi/2 rad in world frame).

