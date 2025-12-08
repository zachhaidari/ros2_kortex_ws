# ROS2 Kortex Workspace - Kinova Gen3 Lite Pick and Place Demo

This repository contains a ROS2 Jazzy workspace for simulating and controlling the **Kinova Gen3 Lite** robot with a custom pick-and-place demonstration using MoveIt2 motion planning.

## 🎯 What Was Changed from Original ros2_kortex

### Custom Additions

1. **`pick_and_place_demo.py`** - A complete pick-and-place demo script that:
   - Uses MoveIt2 motion planning with RRTstar planner for optimal paths
   - Implements joint-space planning to avoid arm spinning issues
   - Picks up 4 colored objects (red cube, blue cylinder, green sphere, yellow cube)
   - Places objects into a basket with individual drop positions
   - Includes collision-aware path planning

2. **`pick_and_place.sdf`** - Custom Gazebo world file containing:
   - A table with the robot mounted
   - 4 colored objects (cubes, cylinder, sphere) for picking
   - A basket for placing objects
   - Proper physics and collision properties

3. **`clear_faults.py`** - Utility script to clear robot faults when connecting to real hardware

4. **`kinova_FK.py`** - Forward kinematics validation script using DH parameters to:
   - Validate end-effector positions for different joint configurations
   - Confirm the home position produces the desired gripper orientation
   - Uses classical Denavit-Hartenberg parameters for accurate FK calculations

5. **`test_fk_complete.py`** - Complete symbolic FK test script using exact symbolic equations
   - Tests multiple joint configurations including home position
   - Displays full transformation matrices and gripper orientations

### Key Modifications

- **Home position**: Changed from `[0,0,0,0,0,0]` to `[0.0, -1.0, -2.05, -1.615, 0.55, 0.0]` for optimal home positioning with gripper horizontal and parallel to table. This position was validated using `kinova_FK.py` to ensure correct end-effector pose at approximately (-0.213m, -0.062m, 0.508m) with gripper pointing horizontally.
- **Motion planning**: Uses joint-space constraints instead of pose-based goals to prevent spinning
- **Planner**: RRTstar with 60 planning attempts and 10 seconds planning time for optimal path finding
- **Object positions**: All objects positioned within robot's reachable workspace (Y ≤ -0.25m) based on IK validation

## 📋 Prerequisites

- Ubuntu 24.04
- ROS2 Jazzy
- Gazebo Harmonic (gz-sim)
- MoveIt2

## 🚀 Quick Start - Simulation

> **Note:** If running in WSL2, simulation will be slower due to limited GPU acceleration. For best performance, use native Ubuntu Linux.

### Option A: Launch Gazebo + RViz/MoveIt Separately (Recommended)

This method gives you more control and allows you to restart individual components.

#### Terminal 1: Launch Gazebo Simulation
```bash
cd ~/ros2_kortex_ws
source install/setup.bash
ros2 launch kortex_bringup kortex_sim_control.launch.py \
    dof:=6 \
    name:=gen3_lite \
    robot_type:=gen3_lite \
    gripper:=gen3_lite_2f \
    sim_gazebo:=true \
    use_sim_time:=true \
    launch_rviz:=false \
    world:=pick_and_place.sdf
```
Wait for Gazebo to fully load and the robot to spawn before proceeding.

#### Terminal 2: Launch RViz + MoveIt
```bash
cd ~/ros2_kortex_ws
source install/setup.bash
ros2 launch kinova_gen3_lite_moveit_config sim.launch.py use_sim_time:=true
```
This launches RViz with the MoveIt motion planning plugin. You can:
- Use the **MotionPlanning** panel to plan and execute motions interactively
- Drag the interactive marker to set goal poses
- Click "Plan & Execute" to move the robot

#### Terminal 3: Run Pick and Place Demo (Optional)
```bash
cd ~/ros2_kortex_ws
source install/setup.bash
ros2 run kortex_bringup pick_and_place_demo.py
```

### Option B: Launch Everything Together

For a simpler setup, you can launch Gazebo with RViz in one command:

#### Terminal 1: Launch Gazebo + RViz
```bash
cd ~/ros2_kortex_ws
source install/setup.bash
ros2 launch kortex_bringup kortex_sim_control.launch.py \
    dof:=6 \
    name:=gen3_lite \
    robot_type:=gen3_lite \
    gripper:=gen3_lite_2f \
    sim_gazebo:=true \
    use_sim_time:=true \
    launch_rviz:=true \
    world:=pick_and_place.sdf
```

#### Terminal 2: Launch MoveIt (still needed for motion planning)
```bash
cd ~/ros2_kortex_ws
source install/setup.bash
ros2 launch kinova_gen3_lite_moveit_config sim.launch.py use_sim_time:=true
```

#### Terminal 3: Run Demo
```bash
cd ~/ros2_kortex_ws
source install/setup.bash
ros2 run kortex_bringup pick_and_place_demo.py
```

### Headless Mode (No GUI - Faster)

For testing without visualization:

```bash
# Terminal 1: Gazebo headless
ros2 launch kortex_bringup kortex_sim_control.launch.py \
    dof:=6 name:=gen3_lite robot_type:=gen3_lite gripper:=gen3_lite_2f \
    sim_gazebo:=true use_sim_time:=true launch_rviz:=false \
    world:=pick_and_place.sdf

# Terminal 2: MoveIt (no RViz)
ros2 launch kinova_gen3_lite_moveit_config sim.launch.py \
    use_sim_time:=true launch_rviz:=false

# Terminal 3: Run demo
ros2 run kortex_bringup pick_and_place_demo.py
```

## 🤖 Real Robot Connection

### 1. Network Setup
Ensure your computer is on the same subnet as the robot:
```bash
# Robot default IP: 192.168.1.10
# Set your computer to: 192.168.1.x (e.g., 192.168.1.100)
sudo ip addr add 192.168.1.100/24 dev eth0
```

### 2. Test Connection
```bash
ping 192.168.1.10
```

### 3. Clear Faults (if robot light is red)
```bash
python3 src/ros2_kortex/kortex_moveit_config/kinova_gen3_lite_moveit_config/launch/clear_faults.py
```

### 4. Launch Real Robot
```bash
source install/setup.bash
ros2 launch kinova_gen3_lite_moveit_config robot.launch.py \
    robot_ip:=192.168.1.10 \
    use_fake_hardware:=false \
    launch_rviz:=true
```

## 📁 Key Files

| File | Location | Description |
|------|----------|-------------|
| `pick_and_place_demo.py` | `src/ros2_kortex/kortex_bringup/scripts/` | Main demo script |
| `pick_and_place.sdf` | `src/ros2_kortex/kortex_bringup/worlds/` | Gazebo world file |
| `clear_faults.py` | `src/ros2_kortex/kortex_moveit_config/kinova_gen3_lite_moveit_config/launch/` | Fault clearing utility |

## 🔧 Building from Source

```bash
cd ~/ros2_kortex_ws
colcon build --symlink-install
source install/setup.bash
```

## 🧮 Forward Kinematics

The FK was validated against Gazebo simulation:

| Joint Configuration | End-Effector Position | Gripper Orientation |
|---------------------|----------------------|---------------------|
| `[0, 0, 0, 0, 0, 0]` | (0.057, -0.010, 1.003) m | Pointing UP |
| `[0.0, 1.0, 2.05, 1.615, 0.55, 0.0]` | Home position | Horizontal (parallel to table) |

## 🎮 Demo Sequence

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

## ⚠️ Troubleshooting

### Robot Light Colors
- **Blue**: Ready
- **Green**: Operating normally
- **Red**: Fault - run `clear_faults.py`
- **Blinking**: Boot sequence or update

### Common Issues

1. **IK Failures (-31 NO_IK_SOLUTION)**
   - Objects may be outside robot workspace
   - Reduce lift height or move objects closer

2. **Arm Spinning**
   - Solved by using joint-space planning instead of pose-based goals

3. **Planning Failures**
   - Increase `num_planning_attempts` in demo script
   - Check collision geometry in RViz

## 📚 References

- [Kinova Gen3 Lite User Guide](https://www.kinovarobotics.com/)
- [ros2_kortex Repository](https://github.com/Kinovarobotics/ros2_kortex)
- [MoveIt2 Documentation](https://moveit.picknik.ai/)

## 📄 License

See individual package licenses in the `src/` directory.
