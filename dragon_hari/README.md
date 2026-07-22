# dragon_hari

ROS package for developing and evaluating communicative body behaviors of DRAGON for Human-Aerial Robot Interaction.

## Purpose

This package is used for the master's research on communicative behaviors using a multi-link aerial robot.

## Fixed Square simulation demo

The Fixed Square demo moves DRAGON horizontally while maintaining the Square
joint posture, the current CoG height, and the current CoG yaw. It is intended
for Gazebo simulation only. The launch file keeps `real_machine=False`, disables
the upstream automatic demo, and starts RViz with a dedicated 2D Nav Goal topic.

### Build and launch

```bash
source ~/ros/jsk_aerial_robot_ws/devel/setup.bash
source ~/ros/human_robot_interaction_ws/devel/setup.bash
cd ~/ros/human_robot_interaction_ws
catkin build dragon_hari --no-deps
source devel/setup.bash
roslaunch dragon_hari fixed_square_demo.launch simulation:=True
```

Use `headless:=true show_image:=false` to omit Gazebo, RViz, and image-view
windows during automated testing. The simulator and control nodes still run.

The launch file sets `/dragon/navigation/trajectory_mean_vel` to `0.2` m/s by
default. Goals farther than `max_goal_distance` (default: `3.0` m) from the
current CoG position are rejected when execution is requested.

### Simulation flight preparation

In a second sourced terminal, arm and take off using the standard DRAGON
simulation commands. Wait until `/dragon/flight_state` is `5` (HOVER) before
requesting Fixed Square execution.

```bash
rostopic pub -1 /dragon/teleop_command/start std_msgs/Empty "{}"
rostopic echo /dragon/flight_state
```

Wait for `data: 2` (ARM_ON), then stop the echo with Ctrl-C and take off:

```bash
rostopic pub -1 /dragon/teleop_command/takeoff std_msgs/Empty "{}"
rostopic echo /dragon/flight_state
```

Wait for `data: 5` (HOVER), then stop the echo with Ctrl-C. Do not select or
execute a Fixed Square goal before HOVER is confirmed.

After the demo, land with:

```bash
rostopic pub -1 /dragon/teleop_command/land std_msgs/Empty "{}"
```

### Select a goal from the CLI

The goal is an absolute XY position in `world`. The input z and orientation are
ignored. Receiving the goal only stores it; it does not publish a joint command
or `/dragon/target_pose`.

Inspect the current world position first and replace the example x and y below
with the intended absolute destination:

```bash
rostopic echo -n 1 /dragon/uav/cog/odom/pose/pose/position
```

```bash
rostopic pub -1 /dragon_hari/fixed_square/goal geometry_msgs/PoseStamped \
  "header:
    stamp: now
    frame_id: 'world'
  pose:
    position: {x: 0.5, y: 0.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"
```

Check that the status reports `state=GOAL_SELECTED`, `goal_selected=true`, and
the stored world coordinates:

```bash
rostopic echo -n 1 /dragon_hari/fixed_square/status
```

Start movement explicitly:

```bash
rosservice call /dragon_hari/fixed_square/execute "{}"
```

The request is accepted only when a goal is selected, the vehicle is in HOVER,
all required state inputs are fresh, all six joints are present, and the goal is
within `max_goal_distance`. A rejected execute request leaves the selected goal
available so the request can be retried after the reported condition is fixed.

### Select a goal from RViz

`fixed_square_demo.launch` loads `rviz/fixed_square_demo.rviz`. Its **2D Nav
Goal** tool publishes `geometry_msgs/PoseStamped` to the dedicated topic:

```text
/dragon_hari/fixed_square/goal
```

Click and drag with **2D Nav Goal** to store the XY position. The arrow direction
is ignored, and clicking does not start movement. Confirm `GOAL_SELECTED` on the
status topic, then call the execute service shown above.

### State and interfaces

```text
IDLE -> GOAL_SELECTED -> PREPARING_SQUARE -> MOVING -> IDLE
```

- Goal input: `/dragon_hari/fixed_square/goal` (`geometry_msgs/PoseStamped`)
- Execute service: `/dragon_hari/fixed_square/execute` (`std_srvs/Trigger`)
- Status: `/dragon_hari/fixed_square/status` (`std_msgs/String`, latched)
- Square command: `/dragon/joints_ctrl` (`sensor_msgs/JointState`)
- Navigation goal: `/dragon/target_pose` (`geometry_msgs/PoseStamped`, once per
  accepted execution after Square convergence)

While waiting in `IDLE` or `GOAL_SELECTED`, another valid goal replaces the
stored goal. Goals received during `PREPARING_SQUARE` or `MOVING` are rejected.
