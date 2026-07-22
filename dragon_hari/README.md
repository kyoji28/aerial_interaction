# dragon_hari

ROS package for developing and evaluating communicative body behaviors of DRAGON for Human-Aerial Robot Interaction.

## Purpose

This package is used for the master's research on communicative behaviors using a multi-link aerial robot.

## Fixed Square simulation demo

The Fixed Square demo moves DRAGON horizontally while maintaining the Square
joint posture, the current CoG height, and the current CoG yaw. It is intended
for Gazebo simulation only.

### Build and launch

Build and source the research workspace:

```bash
source ~/ros/jsk_aerial_robot_ws/devel/setup.bash
cd ~/ros/human_robot_interaction_ws
catkin build dragon_hari --no-deps
source devel/setup.bash
```

Start the DRAGON simulation first:

```bash
roslaunch dragon_hari human_view_demo.launch
```

Arm DRAGON and take off using the standard simulation commands:

```bash
rostopic pub -1 /dragon/teleop_command/start std_msgs/Empty "{}"
rostopic echo /dragon/flight_state
```

Wait for `data: 2` (ARM_ON), stop the echo with Ctrl-C, and then take off:

```bash
rostopic pub -1 /dragon/teleop_command/takeoff std_msgs/Empty "{}"
rostopic echo /dragon/flight_state
```

Wait for `data: 5` (HOVER) and stop the echo with Ctrl-C.

In another sourced terminal, start the Fixed Square node separately:

```bash
source ~/ros/jsk_aerial_robot_ws/devel/setup.bash
source ~/ros/human_robot_interaction_ws/devel/setup.bash
rosrun dragon_hari fixed_square_demo.py
```

At the terminal prompt, enter one absolute destination in the `world` frame as
two values in metres:

```text
Enter target x y [m] in world: 0.8 0.3
```

To inspect the current position before entering the destination, use:

```bash
rostopic echo -n 1 /dragon/uav/cog/odom
```

After input, Square preparation and movement start automatically. Once the
Square posture has settled, the node publishes exactly one goal to
`/dragon/target_pose`. The goal z and yaw are taken from the current CoG state
immediately before publication.

During movement, the node continuously publishes the Square posture to
`/dragon/joints_ctrl`. The destination is considered reached after the horizontal
error remains within `0.10` m for `1.0` s. Joint command publication then stops
and the node exits. If the vehicle leaves HOVER or required state data is lost,
joint command publication also stops and the node exits.

After the demo, land with:

```bash
rostopic pub -1 /dragon/teleop_command/land std_msgs/Empty "{}"
```
