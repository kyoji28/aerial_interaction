# dragon_hari

ROS1 package for developing and evaluating whole-body behaviors of the
DRAGON multilink aerial robot for Human-Aerial Robot Interaction.

## Package structure

```text
dragon_hari/
├── scripts/                  # Executable ROS nodes
├── src/dragon_hari/          # Importable Python modules
│   └── motions/
│       └── follow_the_leader/
├── launch/
├── config/
├── rviz/
└── urdf/
```

The follow-the-leader implementation is separated into:

- `geometry.py`: planar geometry and joint-angle calculation
- `path.py`: reference-path and follow-the-leader shape generation
- `timing.py`: path resampling and temporal processing
- `trajectory.py`: timed joint-trajectory generation
- `joint_player.py`: joint-state reception and joint-command playback
- `pose.py`: CoG pose and quaternion/yaw utilities

## Build

```bash
cd ~/ros/human_robot_interaction_ws
catkin build dragon_hari
source devel/setup.bash
```

## Follow-the-leader nodes

### Joint-command playback only

```bash
rosrun dragon_hari follow_the_leader_joint_demo.py
```

### CoG mapping and validation

```bash
rosrun dragon_hari follow_the_leader_cog_mapping.py
```

### Synchronized joint and CoG motion

```bash
rosrun dragon_hari follow_the_leader_combined_demo.py
```

## Other executable nodes

- `approach_intention_demo.py`
- `joint_angle_input.py`
- `smooth_straight.py`
- `snake_hover_demo.py`
- `straight_line_demo.py`

## Analysis

`follow_the_leader_visualization.py` is an offline visualization script and is
not installed as a ROS executable.
