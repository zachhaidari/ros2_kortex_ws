# ROS2 Kortex Workspace - Kinova Gen3 Lite Pick-and-Place Demo

ROS 2 Jazzy workspace for simulating the Kinova Gen3 Lite (6-DOF) in Gazebo Harmonic and executing a MoveIt 2 pick-and-place demo.

This repo is intended to be a reproducible “Gazebo + MoveIt + ros2_control” baseline that also includes forward-kinematics validation utilities.

## Workspace Structure (Key Paths)

```text
ros2_kortex_ws/
├─ README.md
├─ build/            (colcon output)
├─ install/          (colcon output)
├─ log/              (colcon output)
├─ src/
│  └─ ros2_kortex/
│     └─ kortex_bringup/
│        ├─ matlab/
│        │  └─ record_joint_positions.m
│        ├─ scripts/
│        │  ├─ pick_and_place_demo.py
│        │  ├─ kinova_FK.py
│        │  ├─ kinova_FK_symbolic.py
│        │  ├─ test_home_position.py
│        │  └─ kinova_workspace_sweep.py
│        └─ worlds/
│           └─ pick_and_place.sdf
└─ tools/
```

## What’s In This Workspace

- Gazebo world: `src/ros2_kortex/kortex_bringup/worlds/pick_and_place.sdf`
- Demo node: `src/ros2_kortex/kortex_bringup/scripts/pick_and_place_demo.py`
- FK validation: `src/ros2_kortex/kortex_bringup/scripts/kinova_FK.py`

## Current Demo Parameters (As Implemented)

From `pick_and_place_demo.py`:

- Planner: RRTConnect (joint-space goal constraints)
- Planning: 5.0 s allowed planning time, 30 planning attempts
- Execution shaping: `exec_time_scale = 0.5`, `max_traj_points = 30`
- Home joints used by demo: `[0.0, -1.0, -2.05, -1.615, 0.55, 0.0]`

## MATLAB Integration (Joint Streaming)

The pick-and-place demo publishes the 6 arm joint angles to a MATLAB-friendly topic for plotting/recording.

- Topic: `/matlab_joint_angles`
- Message type: `trajectory_msgs/JointTrajectory`
- Payload convention: `msg.points[0].positions` contains `[joint_1..joint_6]` in radians
- Publish rate: whenever `/joint_states` is received (i.e., it mirrors the joint state update rate)

MATLAB script provided:

- `src/ros2_kortex/kortex_bringup/matlab/record_joint_positions.m`

What it does:

- Subscribes to `/matlab_joint_angles`
- Logs the incoming `positions` vector
- Plots the 6 joint angles live (and also provides blocking + 3D helper variants inside the same file)

Minimal usage (MATLAB ROS 2 Toolbox required):

1) Start the simulation/demo as usual so `/matlab_joint_angles` exists.
2) In MATLAB, run:

```matlab
record_joint_positions
```

If you need to specify DDS/ROS domain separation, ensure MATLAB and ROS 2 use the same `ROS_DOMAIN_ID`.

## Prerequisites

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic
- MoveIt 2

## Build

```bash
cd ~/ros2_kortex_ws
colcon build --symlink-install
source install/setup.bash
```

## Quick Start - Simulation

If running in WSL2, expect slower-than-realtime simulation throughput.

### Option A: Launch Gazebo + MoveIt/RViz Separately (Recommended)

Terminal 1 (Gazebo + ros2_control):

```bash
cd ~/ros2_kortex_ws
source install/setup.bash
ros2 launch kortex_bringup kortex_sim_control.launch.py \
  dof:=6 name:=gen3_lite robot_type:=gen3_lite gripper:=gen3_lite_2f \
  sim_gazebo:=true use_sim_time:=true launch_rviz:=false \
  world:=pick_and_place.sdf
```

Terminal 2 (MoveIt + RViz):

```bash
cd ~/ros2_kortex_ws
source install/setup.bash
ros2 launch kinova_gen3_lite_moveit_config sim.launch.py use_sim_time:=true
```

Terminal 3 (run the demo):

```bash
cd ~/ros2_kortex_ws
source install/setup.bash
ros2 run kortex_bringup pick_and_place_demo.py
```

### Option B: Launch Gazebo With RViz, Then Start MoveIt

This is sometimes convenient if you want RViz up as soon as Gazebo starts.

Terminal 1 (Gazebo + RViz):

```bash
cd ~/ros2_kortex_ws
source install/setup.bash
ros2 launch kortex_bringup kortex_sim_control.launch.py \
  dof:=6 name:=gen3_lite robot_type:=gen3_lite gripper:=gen3_lite_2f \
  sim_gazebo:=true use_sim_time:=true launch_rviz:=true \
  world:=pick_and_place.sdf
```

Terminal 2 (MoveIt):

```bash
cd ~/ros2_kortex_ws
source install/setup.bash
ros2 launch kinova_gen3_lite_moveit_config sim.launch.py use_sim_time:=true
```

Terminal 3 (demo):

```bash
cd ~/ros2_kortex_ws
source install/setup.bash
ros2 run kortex_bringup pick_and_place_demo.py
```

### Headless Mode (No GUI)

```bash
# Terminal 1: Gazebo headless
ros2 launch kortex_bringup kortex_sim_control.launch.py \
  dof:=6 name:=gen3_lite robot_type:=gen3_lite gripper:=gen3_lite_2f \
  sim_gazebo:=true use_sim_time:=true launch_rviz:=false \
  world:=pick_and_place.sdf

# Terminal 2: MoveIt (no RViz)
ros2 launch kinova_gen3_lite_moveit_config sim.launch.py use_sim_time:=true launch_rviz:=false

# Terminal 3: Demo
ros2 run kortex_bringup pick_and_place_demo.py
```

## World Objects (Current Spawn Poses, World Frame)

From `pick_and_place.sdf`:

- Red cube: (-0.35, -0.38, 0.375)
- Blue cylinder: (-0.10, -0.38, 0.380)
- Green sphere: (-0.28, -0.30, 0.375)
- Yellow cube: (0.00, -0.25, 0.375)
- Basket: (-0.30, -0.05, 0.365)

## Key Files

| File | Location | Description |
|------|----------|-------------|
| `pick_and_place_demo.py` | `src/ros2_kortex/kortex_bringup/scripts/` | Main demo script with RRTConnect planner |
| `pick_and_place.sdf` | `src/ros2_kortex/kortex_bringup/worlds/` | Gazebo world file with optimized object positions |
| `kinova_FK.py` | `src/ros2_kortex/kortex_bringup/scripts/` | DH-based forward kinematics validation |
| `test_home_position.py` | `src/ros2_kortex/kortex_bringup/scripts/` | Home position validation against Gazebo |
| `record_joint_positions.m` | `src/ros2_kortex/kortex_bringup/matlab/` | MATLAB subscriber/plotter for `/matlab_joint_angles` |
| `gen3_lite.urdf` | `src/ros2_kortex/kortex_description/robots/` | Robot URDF (includes gripper mimic joint fix) |
| `clear_faults.py` | `src/ros2_kortex/kortex_moveit_config/kinova_gen3_lite_moveit_config/launch/` | Fault clearing utility |

## Forward Kinematics (Quick Check)

The FK scripts provide numeric checks against the expected “home” pose:

| Joint Configuration | End-Effector Position | Gripper Orientation |
|---------------------|----------------------|---------------------|
| `[0, 0, 0, 0, 0, 0]` | (0.057, -0.010, 1.003) m | Pointing UP |
| `[0.0, -1.0, -2.05, -1.615, 0.55, 0.0]` | (-0.213, -0.062, 0.508) m | Home Position, pointing toward workspace |

## Workspace Notes

Empirical testing for tabletop grasp heights suggests practical limits around:

- y approximately in [-0.10 m, -0.35 m]
- radial reach approximately 0.45 m from base

## Demo Sequence

The pick-and-place demo performs the following for each object:
1. Move to home position (gripper horizontal)
2. Open gripper
3. Move to pre-grasp position above object
4. Lower to grasp position
5. Close gripper
6. Lift object
7. Move above basket
8. Lower into basket
9. Release object
10. Repeat for next object

## Troubleshooting (Common)

- IK failures (error -31) usually mean the target pose is outside the empirical workspace bounds.
- If planning fails intermittently, increase `num_planning_attempts` (current default: 30) or `planning_time` (current default: 5.0 s) in `pick_and_place_demo.py`.
- If the gripper “stalls” while closing on an object, the demo treats this as a successful grasp.

## Real Robot (Optional)

This repo is primarily focused on simulation; however, if you are running on hardware and need to clear faults:

```bash
python3 src/ros2_kortex/kortex_moveit_config/kinova_gen3_lite_moveit_config/launch/clear_faults.py
```

## 📚 References

- [Kinova Gen3 Lite User Guide](https://www.kinovarobotics.com/)
- [ros2_kortex Repository](https://github.com/Kinovarobotics/ros2_kortex)
- [MoveIt2 Documentation](https://moveit.picknik.ai/)

## 📄 License

See individual package licenses in the `src/` directory.
