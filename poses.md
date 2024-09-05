### Checkpoint 12 Data

#### `PoseWithCovarianceStamped`

1. Description
```
user:~$ ros2 interface show geometry_msgs/msg/PoseWithCovarianceStamped
# This expresses an estimated pose with a reference coordinate frame and timestamp

std_msgs/Header header
        builtin_interfaces/Time stamp
                int32 sec
                uint32 nanosec
        string frame_id
PoseWithCovariance pose
        Pose pose
                Point position
                        float64 x
                        float64 y
                        float64 z
                Quaternion orientation
                        float64 x 0
                        float64 y 0
                        float64 z 0
                        float64 w 1
        float64[36] covariance
```

2. Indices:
   1. `covariance[0]` - x
   2. `covariance[7]` - y
   3. `covariance[35]` - z

3. Tolerance [0.025, 0.1]


#### 1. Simulator poses (relative to `map` frame)

Format: `{position.x, position.y, orientation.z, orientation.w}`  

1. Initial pose (`init_position`)
`{x: 0.0400, y: y: 0.1121, z: 0.0850, w: 0.9964}`  

2. Shipping pose (`shipping_position`)
`{x: 5.5888, y: -0.0618, z: -0.7192,  w: 0.6948}`  

3. Pickup pose _(under the cart)_
`{x: 5.5412, y: -1.2838, z: -0.7017, w: 0.7124}`

4. Backup pose _(with cart, before rotating -90 deg to head toward loading pose)_
`{x: 5.6353, y: -0.3010, z: -0.7257, w: 0.6880}`  

5. Face up _(with cart, after rotating to head up toward loading pose)_
`{x: 5.6460, y: -0.1604, z: -0.9979, w: 0.0640}`  

6. pre-shipping _(before turning -90 deg to face the loading pose)_
`{x: 2.5991, y: -0.1406, z: 1.0000, w: 0.0087}`  

7. face-shipping _(facing the loading pose)_
`{x: 2.4501, y: -0.1236, z: 0.7494, w: 0.6621}`  

8. Shipping pose (`shipping_position`)
`{x: 2.5062, y: 1.3838, z: 0.6914, w: 0.7225}`  

#### 2. Lab poses (relative to `map` frame)

Format: `{position.x, position.y, orientation.z, orientation.w}`  

1. Initial pose (`init_position`)
pose:
  pose:
    position:
      x: 0.027209922092298333
      y: 0.13108187691856119
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: -0.009940744287988275
      w: 0.9999505895808056
 

2. Shipping pose (`shipping_position`)
pose:
  pose:
    position:
      x: 4.407935402220254
      y: -0.13696732188141075
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: -0.7175132920439505
      w: 0.6965448124422811
  covariance:

3. Pickup pose _(under the cart, EXTREMELY TIGHT)_
pose:
  pose:
    position:
      x: 4.451899193456196
      y: -1.350635540857341
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: -0.5459039427889757
      w: 0.8378477697335303
  covariance:

4. Backup pose _(with cart, before rotating -90 deg to head toward loading pose)_
pose:
  pose:
    position:
      x: 4.441332058900151
      y: -0.13297391027464547
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: -0.7133721117393408
      w: 0.7007854380568659


5. Face up _(with cart, after rotating to head up toward loading pose)_
pose:
  pose:
    position:
      x: 4.394210202831508
      y: -0.07269371807489942
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: -0.9999945649170592
      w: 0.0032969889810993973
  covariance:

6. pre-shipping _(before turning -90 deg to face the loading pose)_
pose:
  pose:
    position:
      x: 2.1201544802356853
      y: -0.02389789104196904
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: -0.9989668470518563
      w: 0.04544489510685459

7. face-shipping _(facing the loading pose)_
pose:
  pose:
    position:
      x: 2.045330208145817
      y: -0.010055776837808096
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.7267148513422206
      w: 0.6869392439209265

8. Shipping pose (`shipping_position`)
pose:
  pose:
    position:
      x: 2.0282166067521623
      y: 1.012256892508906
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.6864194603568776
      w: 0.727205833614784

#### Notes from the lab

1. Lift/attach to cart `ros2 topic pub --once /elevator_up std_msgs/msg/String "{}"` 
2. Put down/detach from cart `ros2 topic pub --once /elevator_down std_msgs/msg/String "{}"` 
3. Extremely tight entrance under the cart. **The slightest inaccuracy will start pushing the cart around.** Robot needs to align with the cart, which is easy visually, but not sure how to do it programmatically.
4. The [`approach_service_server.cpp`](attach_shelf/src/approach_service_server.cpp) might need to be rewritten to use the reflective plates as long as possible. Most importantly, the robot should stop (or position itself) so that the distances to the two reflective plates is the same. Then, it should move slowly and correct for growing difference in distance.
5. The cart in the lab appears shorter.
6. There needs to be a server for the opposite motion, that is, detaching from the cart and backing out from under it so that it can restart the Simple Commander (with the navigator).

So:
1. (Lab only) Center the robot in the starting square (`init_position`).
2. Localize the robot at `init_position` (rotate in place +180, -360, +180 until pose coverances for x, y, and z are under 0.03). 
3. (Simiple Commander) Reach `loading_position`. _If possible, add the logic to stop where the distances to the reflective plates are the same._
4. (`approach_service_server`) Adjust position so that the distances to the reflective plates is the same.
5. (`approach_service_server`) Rotate until pointing at the middle between the plates. Compute the distance to the midpoint. Publish `cart_frame`.
6. (`approach_service_server`) Approach `cart_frame` slowly, using the plates as long as possible to correct for difference in the distances to them.
7. (`approach_service_server`) Continue slowly forward for a half-length of the cart.
8. (`approach_service_server`) Lift the cart.
9. (`approach_service_server`) Publish new dimensions (probably a 2D `Polygon` to `/local_costmap/footprint`)
10. (`approach_service_server` or SC) Go back to `loading_position`.
11. (SC) Go to pre-shipping position and then `shipping_position`.
12. (`detach_service_server`) Set the shelf down.
13. (`detach_service_server`) Back off slowly until free of the cart.
14. (SC) Go to pre-shipping and then to `init_position`.

TODO (lab):
1. Parametrize scanner.
2. Measure the cart:
   1. Width a little over the diameter of the robot.
   2. Half-length using the difference between two `/acml_pose`-sof the cart.
