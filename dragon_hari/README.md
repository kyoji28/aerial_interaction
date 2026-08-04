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

## Follow-the-leader node

The synchronized joint and CoG motion is executed through a single ROS node.

```bash
rosrun dragon_hari follow_the_leader.py
```

## Other executable nodes

- `approach_intention_demo.py`
- `joint_angle_input.py`
- `straight_line_demo.py`: interactive launcher for the straight-line demos
- `straight_line_fixed_square.py`: straight motion with a fixed square posture
- `straight_line_vertex_forward.py`: straight motion with a vertex facing forward
- `straight_line_link4_forward.py`: straight motion with Link 4 facing forward
- `straight_line_body_wave.py`: straight motion with a body-wave joint motion
