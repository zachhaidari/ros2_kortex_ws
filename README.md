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
- **Gripper URDF fix**: Corrected mimic joint parameters for `left_finger_bottom_joint` and `left_finger_dist_joint` in `gen3_lite.urdf` (multiplier: -0.276 → -0.676, offset: 0.0 → 0.149) to ensure proper gripper closing
- **Motion planning**: Uses joint-space constraints instead of pose-based goals to prevent spinning
- **Planner**: RRTConnect with 15 planning attempts and 10 seconds planning time for faster convergence
- **Object positions**: All objects positioned within robot's reachable workspace at Y ∈ [-0.28, -0.32] and X ∈ [-0.10, -0.40] based on empirical IK validation

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
| `pick_and_place_demo.py` | `src/ros2_kortex/kortex_bringup/scripts/` | Main demo script with RRTConnect planner |
| `pick_and_place.sdf` | `src/ros2_kortex/kortex_bringup/worlds/` | Gazebo world file with optimized object positions |
| `kinova_FK.py` | `src/ros2_kortex/kortex_bringup/scripts/` | DH-based forward kinematics validation |
| `test_home_position.py` | `src/ros2_kortex/kortex_bringup/scripts/` | Home position validation against Gazebo |
| `gen3_lite.urdf` | `src/ros2_kortex/kortex_description/robots/` | Robot URDF (includes gripper mimic joint fix) |
| `clear_faults.py` | `src/ros2_kortex/kortex_bringup/scripts/` | Fault clearing utility |

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
| `[0.0, -1.0, -2.05, -1.615, 0.55, 0.0]` | (-0.213, -0.062, 0.508) m | Horizontal, pointing toward workspace |

## 📐 Workspace Characterization

The Gen3 Lite 6-DOF has a limited reachable workspace compared to longer-armed manipulators:

- **Reachable region** (at table height Z = 0.18m):
  - Y-range: -0.10m to -0.35m (25cm depth)
  - X-range: -0.10m to -0.45m (35cm width)
  - Circular approximation: ~45cm radius from base

- **IK failure zones**:
  - Y < -0.40m: Too far forward (joint limits exceeded)
  - Y > -0.10m: Behind robot (physically unreachable)
  - Combined X-Y distances > 45cm: Outside reach

**Final validated object positions:**
- Red cube: (-0.25, -0.28, 0.375)
- Blue cylinder: (-0.10, -0.32, 0.38)
- Green sphere: (-0.40, -0.25, 0.375)
- Yellow cube: (-0.15, -0.25, 0.375)
- Basket: (-0.30, -0.05, 0.365)

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
   - Gen3 Lite 6-DOF has limited reach: ~45cm radius from base
   - Keep objects within Y ∈ [-0.10, -0.35] at table height (Z ≈ 0.18m)
   - Objects beyond Y < -0.40m or Y > -0.10m will fail IK

2. **Arm Spinning**
   - Solved by using joint-space planning instead of pose-based goals

3. **Planning Failures**
   - Increase `num_planning_attempts` in demo script (default: 15)
   - Check collision geometry in RViz
   - RRTConnect typically succeeds within first 5-10 attempts

4. **Gripper Not Closing Properly**
   - Ensure URDF mimic joint fix is applied (multiplier=-0.676, offset=0.149)
   - Gripper may stall when gripping objects (this is normal)

5. **Slow Simulation in WSL2**
   - WSL2 runs at 30-50% real-time vs 80-100% on native Linux
   - Extended timeouts (120s) are set to accommodate slower execution

## 📚 References

- [Kinova Gen3 Lite User Guide](https://www.kinovarobotics.com/)
- [ros2_kortex Repository](https://github.com/Kinovarobotics/ros2_kortex)
- [MoveIt2 Documentation](https://moveit.picknik.ai/)

## 📄 License

See individual package licenses in the `src/` directory.
