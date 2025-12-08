#!/usr/bin/env python3
"""
Pick and Place Demo for Kinova Gen3 Lite

This script demonstrates a pick-and-place operation using:
- MoveIt motion planning service for collision-aware path planning
- Joint trajectory controller for arm movement
- Gripper action for gripper control

Usage:
  1. Launch Gazebo simulation with pick_and_place world
  2. Launch MoveIt
  3. Run this script: ros2 run kortex_bringup pick_and_place_demo.py
"""

import rclpy
import rclpy.parameter
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.srv import GetPositionIK, GetMotionPlan
from moveit_msgs.msg import (
    RobotState, PositionIKRequest, CollisionObject,
    Constraints, PositionConstraint, OrientationConstraint,
    JointConstraint, BoundingVolume, MotionPlanRequest
)
from control_msgs.action import GripperCommand, FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from shape_msgs.msg import SolidPrimitive

import time


class PickAndPlaceDemo(Node):
    """Pick and Place Demo Node for Kinova Gen3 Lite."""

    # Arm joint names for Gen3 Lite
    ARM_JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

    def __init__(self):
        super().__init__('pick_and_place_demo',
                         parameter_overrides=[
                             rclpy.parameter.Parameter('use_sim_time', 
                                                       rclpy.parameter.Parameter.Type.BOOL, 
                                                       True)
                         ])
        
        # Callback group for concurrent callbacks
        self.callback_group = ReentrantCallbackGroup()
        
        # IK service client
        self.ik_client = self.create_client(
            GetPositionIK, '/compute_ik',
            callback_group=self.callback_group
        )
        
        # Motion planning service client (for collision-aware planning)
        self.plan_client = self.create_client(
            GetMotionPlan, '/plan_kinematic_path',
            callback_group=self.callback_group
        )
        
        # Joint trajectory action client
        self.trajectory_client = ActionClient(
            self, FollowJointTrajectory, 
            '/joint_trajectory_controller/follow_joint_trajectory',
            callback_group=self.callback_group
        )
        
        # Gripper action client
        self.gripper_client = ActionClient(
            self, GripperCommand, 
            '/gen3_lite_2f_gripper_controller/gripper_cmd',
            callback_group=self.callback_group
        )
        
        # Joint state subscriber
        self.current_joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        
        # Configuration
        self.planning_group = "arm"
        self.ee_link = "end_effector_link"
        self.base_frame = "base_link"
        
        # Gripper positions (gen3_lite_2f: 0.0=open, max=0.96)
        # When gripping an object, gripper will stall before reaching target
        # Use 0.5 - just enough to grip 35mm collision objects
        self.GRIPPER_OPEN = 0.1
        self.GRIPPER_CLOSED = 0.8
        
        # Motion planning settings
        self.use_motion_planning = True  # Use MoveIt planning vs direct IK
        self.planner_id = "RRTstar"  # RRTstar optimizes for shortest path
        self.planning_time = 10.0  # seconds allowed for planning
        self.num_planning_attempts = 60  # number of attempts
        
        # Motion timing - fast movements
        self.move_duration = 1.50  # seconds for each motion (faster)
        
        # Define pick stations in base_link frame
        # Robot at yaw=0, objects at negative X and Y (further away)
        # Robot base_link is at world z=0.365
        # Objects center at world z=0.375 → base_link z = 0.01
        # Objects top surface at base_link z ≈ 0.035
        # Gripper finger attachment at z=0.070003 relative to gripper_base_link
        # Fingertips sweep down to approximately z = 0.02-0.04m when closing
        # For reliable grasping: position end_effector_link above object top
        # Pregrasp: z = 0.15 (clear approach above object)
        # Grasp: z = 0.06 (fingertips can reach object top at ~0.035)
        # Basket center is at (-0.30, -0.05), 30cm x 20cm, objects placed near center
        self.pick_stations = {
            'red_cube': {
                'pregrasp': self._create_pose(-0.25, -0.28, 0.21),
                'grasp': self._create_pose(-0.25, -0.28, 0.18),
                'basket_approach': self._create_pose(-0.27, -0.08, 0.30),
                'basket_drop': self._create_pose(-0.27, -0.08, 0.18),
            },
            'blue_cylinder': {
                'pregrasp': self._create_pose(-0.10, -0.32, 0.21),
                'grasp': self._create_pose(-0.10, -0.32, 0.18),
                'basket_approach': self._create_pose(-0.33, -0.08, 0.30),
                'basket_drop': self._create_pose(-0.33, -0.08, 0.16),
            },
            'green_sphere': {
                'pregrasp': self._create_pose(-0.40, -0.25, 0.21),
                'grasp': self._create_pose(-0.40, -0.25, 0.18),
                'basket_approach': self._create_pose(-0.27, -0.02, 0.30),
                'basket_drop': self._create_pose(-0.27, -0.02, 0.15),
            },
            'yellow_cube': {
                'pregrasp': self._create_pose(-0.15, -0.25, 0.21),
                'grasp': self._create_pose(-0.15, -0.25, 0.18),
                'basket_approach': self._create_pose(-0.33, -0.02, 0.30),
                'basket_drop': self._create_pose(-0.33, -0.02, 0.18),
            },
        }
        
        # Default basket positions (fallback)
        self.basket_approach = self._create_pose(-0.30, -0.05, 0.30)  # High approach
        self.basket_position = self._create_pose(-0.30, -0.05, 0.20)  # Drop position
        
        self.get_logger().info('Pick and Place Demo initialized')
        self.get_logger().info('Waiting for services and action servers...')
        
        # Wait for services
        if not self.ik_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('IK service not available!')
            return
        if self.use_motion_planning:
            if not self.plan_client.wait_for_service(timeout_sec=10.0):
                self.get_logger().warn('Motion planning service not available, falling back to IK')
                self.use_motion_planning = False
            else:
                self.get_logger().info(f'Motion planning enabled with {self.planner_id} planner')
        if not self.trajectory_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Joint trajectory action server not available!')
            return
        if not self.gripper_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Gripper action server not available!')
            return
        
        # Wait for joint states
        timeout = 10.0
        start = time.time()
        while self.current_joint_state is None and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.current_joint_state is None:
            self.get_logger().error('No joint states received!')
            return
            
        self.get_logger().info('All services and actions connected!')
        self.get_logger().info(f'Available pick stations: {", ".join(self.pick_stations.keys())}')
        
        # Publish objects to RViz
        self.publish_objects_to_rviz()

    def publish_objects_to_rviz(self):
        """Publish collision objects to RViz2 for visualization."""
        # Create publisher for collision objects
        collision_object_pub = self.create_publisher(CollisionObject, '/collision_object', 10)
        
        # Wait a moment for publisher to be ready
        time.sleep(0.5)
        
        # Objects in WORLD frame (from SDF spawn positions)
        # Robot base_link is at world z=0.365, so convert: base_link_z = world_z - 0.365
        objects_world = [
            ('red_cube', -0.25, -0.25, 0.375, 0.05, 0.05, 0.05),
            ('blue_cylinder', -0.45, -0.20, 0.38, 0.045, 0.045, 0.06),  # Moved closer
            ('green_sphere', -0.40, -0.05, 0.375, 0.05, 0.05, 0.05),
            ('yellow_cube', -0.15, -0.05, 0.375, 0.05, 0.05, 0.05),   # Moved closer
        ]
        
        # Convert z-coordinates from world to base_link frame
        # base_link z = world z - 0.365
        BASE_LINK_OFFSET = 0.365
        
        for name, x_world, y, z_world, sx, sy, sz in objects_world:
            z_base_link = z_world - BASE_LINK_OFFSET
            
            co = CollisionObject()
            co.header.frame_id = 'base_link'
            co.id = name
            co.operation = CollisionObject.ADD
            
            primitive = SolidPrimitive()
            if 'cube' in name:
                primitive.type = SolidPrimitive.BOX
                primitive.dimensions = [sx, sy, sz]
            elif 'cylinder' in name:
                primitive.type = SolidPrimitive.CYLINDER
                primitive.dimensions = [sz, sx/2]  # height, radius
            elif 'sphere' in name:
                primitive.type = SolidPrimitive.SPHERE
                primitive.dimensions = [sx/2]  # radius
            
            pose = Pose()
            pose.position.x = x_world
            pose.position.y = y
            pose.position.z = z_base_link
            pose.orientation.w = 1.0
            
            co.primitives.append(primitive)
            co.primitive_poses.append(pose)
            
            collision_object_pub.publish(co)
            self.get_logger().info(f'Published collision object: {name} at base_link z={z_base_link:.3f}')
        
        time.sleep(0.5)

    def _create_pose(self, x, y, z, qx=1.0, qy=0.0, qz=0.0, qw=0.0):
        """Create a Pose message with gripper pointing straight down.
        
        Orientation: 180° rotation around X-axis points Z-axis down.
        qx=1, qy=0, qz=0, qw=0 represents a 180° rotation around X.
        """
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation.x = float(qx)
        pose.orientation.y = float(qy)
        pose.orientation.z = float(qz)
        pose.orientation.w = float(qw)
        return pose

    def joint_state_callback(self, msg):
        """Store current joint state."""
        self.current_joint_state = msg

    def get_arm_joint_positions(self):
        """Extract arm joint positions from current joint state."""
        if self.current_joint_state is None:
            return None
        
        positions = []
        for joint_name in self.ARM_JOINTS:
            try:
                idx = self.current_joint_state.name.index(joint_name)
                positions.append(self.current_joint_state.position[idx])
            except ValueError:
                self.get_logger().error(f'Joint {joint_name} not found in joint state!')
                return None
        return positions

    def compute_ik(self, target_pose: Pose):
        """Compute inverse kinematics for target pose."""
        request = GetPositionIK.Request()
        request.ik_request = PositionIKRequest()
        request.ik_request.group_name = self.planning_group
        request.ik_request.ik_link_name = self.ee_link
        request.ik_request.robot_state = RobotState()
        request.ik_request.robot_state.joint_state = self.current_joint_state
        request.ik_request.robot_state.is_diff = False
        request.ik_request.avoid_collisions = True
        
        # Set target pose
        request.ik_request.pose_stamped = PoseStamped()
        request.ik_request.pose_stamped.header.frame_id = self.base_frame
        request.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        request.ik_request.pose_stamped.pose = target_pose
        
        request.ik_request.timeout = Duration(sec=5, nanosec=0)
        
        # Call IK service
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.result() is None:
            self.get_logger().error('IK service call failed!')
            return None
        
        response = future.result()
        if response.error_code.val != 1:  # SUCCESS = 1
            self.get_logger().error(f'IK failed with error code: {response.error_code.val}')
            return None
        
        # Extract arm joint positions from solution
        joint_positions = []
        for joint_name in self.ARM_JOINTS:
            try:
                idx = response.solution.joint_state.name.index(joint_name)
                joint_positions.append(response.solution.joint_state.position[idx])
            except ValueError:
                self.get_logger().error(f'Joint {joint_name} not in IK solution!')
                return None
        
        return joint_positions

    def plan_to_pose(self, target_pose: Pose):
        """Plan a collision-aware motion to target pose using MoveIt.
        
        Uses IK first to find joint positions, then plans in joint space.
        This avoids the spinning problem where pose-based planning might
        choose a longer joint rotation path.
        """
        # First compute IK to get target joint positions
        # This gives us a specific joint configuration to target
        target_joints = self.compute_ik(target_pose)
        if target_joints is None:
            self.get_logger().error('IK failed, cannot plan')
            return None
        
        return self.plan_to_joints(target_joints)
    
    def plan_to_joints(self, target_joints):
        """Plan a collision-aware motion to target joint positions.
        
        Planning in joint space naturally takes shorter joint paths
        and avoids the spinning problem.
        """
        request = GetMotionPlan.Request()
        
        # Set up motion plan request
        mp_request = MotionPlanRequest()
        mp_request.group_name = self.planning_group
        mp_request.planner_id = self.planner_id
        mp_request.num_planning_attempts = self.num_planning_attempts
        mp_request.allowed_planning_time = self.planning_time
        mp_request.max_velocity_scaling_factor = 0.3  # 30% of max velocity
        mp_request.max_acceleration_scaling_factor = 0.3  # 30% of max acceleration
        
        # Set start state to current state
        mp_request.start_state = RobotState()
        mp_request.start_state.joint_state = self.current_joint_state
        mp_request.start_state.is_diff = False
        
        # Create goal constraints using JOINT constraints (not pose)
        # This ensures the planner takes the shortest joint-space path
        goal_constraints = Constraints()
        goal_constraints.name = "joint_goal"
        
        for i, joint_name in enumerate(self.ARM_JOINTS):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = target_joints[i]
            joint_constraint.tolerance_above = 0.01  # ~0.5 degrees
            joint_constraint.tolerance_below = 0.01
            joint_constraint.weight = 1.0
            goal_constraints.joint_constraints.append(joint_constraint)
        
        mp_request.goal_constraints.append(goal_constraints)
        
        request.motion_plan_request = mp_request
        
        self.get_logger().info(f'Planning to joints with {self.planner_id}...')
        
        # Call planning service
        future = self.plan_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.planning_time + 5.0)
        
        if future.result() is None:
            self.get_logger().error('Motion planning service call failed!')
            return None
        
        response = future.result()
        if response.motion_plan_response.error_code.val != 1:  # SUCCESS = 1
            self.get_logger().error(f'Motion planning failed with error code: {response.motion_plan_response.error_code.val}')
            return None
        
        trajectory = response.motion_plan_response.trajectory.joint_trajectory
        self.get_logger().info(f'Motion plan found with {len(trajectory.points)} waypoints')
        
        return trajectory

    def execute_trajectory(self, trajectory: JointTrajectory):
        """Execute a planned trajectory using the trajectory controller."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        
        self.get_logger().info(f'Executing planned trajectory with {len(trajectory.points)} points...')
        
        # Send goal
        future = self.trajectory_client.send_goal_async(goal)
        
        # Wait for goal acceptance
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < 10.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not future.done():
            self.get_logger().error('Timeout waiting for goal acceptance!')
            return False
        
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('Trajectory goal rejected!')
            return False
        
        self.get_logger().info('Trajectory goal accepted, executing...')
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        max_wall_time = 120.0
        start_time = time.time()
        
        while not result_future.done() and (time.time() - start_time) < max_wall_time:
            rclpy.spin_once(self, timeout_sec=0.5)
        
        if not result_future.done():
            self.get_logger().error('Timeout waiting for trajectory result!')
            return False
        
        result = result_future.result()
        if result is None or result.result.error_code != 0:
            self.get_logger().error(f'Trajectory execution failed!')
            return False
        
        self.get_logger().info('Trajectory execution successful!')
        return True

    def move_to_joint_positions(self, joint_positions, duration=None):
        """Move arm to specified joint positions using trajectory controller."""
        if duration is None:
            duration = self.move_duration
        
        # Create trajectory goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = self.ARM_JOINTS
        goal.trajectory.header.stamp = self.get_clock().now().to_msg()
        
        # Single point trajectory
        point = JointTrajectoryPoint()
        point.positions = list(joint_positions)
        point.velocities = [0.0] * len(joint_positions)
        point.accelerations = [0.0] * len(joint_positions)
        point.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))
        goal.trajectory.points.append(point)
        
        self.get_logger().info(f'Sending trajectory to positions: {[f"{p:.3f}" for p in joint_positions]}')
        
        # Send goal
        future = self.trajectory_client.send_goal_async(goal)
        
        # Wait for goal acceptance (short timeout in wall time is OK)
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < 10.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not future.done():
            self.get_logger().error('Timeout waiting for goal acceptance!')
            return False
        
        goal_handle = future.result()
        if not goal_handle:
            self.get_logger().error('Trajectory goal handle is None!')
            return False
        if not goal_handle.accepted:
            self.get_logger().error('Trajectory goal rejected!')
            return False
        
        self.get_logger().info('Trajectory goal accepted, waiting for execution...')
        
        # Wait for result - poll with longer wall clock timeout
        # since simulation may run slower than real time
        result_future = goal_handle.get_result_async()
        max_wall_time = 120.0  # 2 minutes wall time max
        start_time = time.time()
        
        while not result_future.done() and (time.time() - start_time) < max_wall_time:
            rclpy.spin_once(self, timeout_sec=0.5)
        
        if not result_future.done():
            self.get_logger().error(f'Timeout waiting for trajectory result after {max_wall_time}s wall time!')
            return False
        
        result = result_future.result()
        if result is None:
            self.get_logger().error('Trajectory result is None!')
            return False
        
        error_code = result.result.error_code
        if error_code != 0:  # SUCCESSFUL = 0
            error_names = {
                0: 'SUCCESSFUL',
                -1: 'INVALID_GOAL',
                -2: 'INVALID_JOINTS',
                -3: 'OLD_HEADER_TIMESTAMP',
                -4: 'PATH_TOLERANCE_VIOLATED',
                -5: 'GOAL_TOLERANCE_VIOLATED',
            }
            error_name = error_names.get(error_code, f'UNKNOWN({error_code})')
            self.get_logger().error(f'Trajectory failed: {error_name}')
            return False
        
        self.get_logger().info('Trajectory execution successful!')
        return True

    def move_to_pose(self, target_pose: Pose):
        """Move end effector to target pose using motion planning or direct IK."""
        self.get_logger().info(f'Moving to pose: ({target_pose.position.x:.3f}, '
                               f'{target_pose.position.y:.3f}, {target_pose.position.z:.3f})')
        
        if self.use_motion_planning:
            # Use MoveIt motion planning for collision-aware, optimized paths
            trajectory = self.plan_to_pose(target_pose)
            if trajectory is not None:
                if self.execute_trajectory(trajectory):
                    self.get_logger().info('Motion planning succeeded!')
                    return True
                else:
                    self.get_logger().warn('Trajectory execution failed, trying direct IK...')
            else:
                self.get_logger().warn('Motion planning failed, falling back to direct IK...')
        
        # Fall back to direct IK if motion planning is disabled or fails
        joint_positions = self.compute_ik(target_pose)
        if joint_positions is None:
            self.get_logger().error('Failed to compute IK!')
            return False
        
        self.get_logger().info(f'IK solution found, executing trajectory...')
        
        # Execute trajectory
        if not self.move_to_joint_positions(joint_positions):
            self.get_logger().error('Failed to execute trajectory!')
            return False
        
        self.get_logger().info('Motion completed!')
        return True

    def move_gripper(self, position, max_effort=50.0):
        """Move gripper to specified position."""
        action = "Opening" if position < 0.1 else "Closing"
        self.get_logger().info(f'{action} gripper to position {position:.2f}')
        
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = max_effort
        
        future = self.gripper_client.send_goal_async(goal)
        
        # Wait for goal acceptance
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not future.done():
            self.get_logger().error('Timeout waiting for gripper goal acceptance!')
            return True  # Continue anyway
        
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('Gripper goal rejected!')
            return False
        
        # Wait for result with polling - use short timeout
        # Gripper may stall when gripping object before reaching target
        result_future = goal_handle.get_result_async()
        max_wall_time = 3.0
        start_time = time.time()
        
        while not result_future.done() and (time.time() - start_time) < max_wall_time:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not result_future.done():
            # Timeout is OK for gripper - check if it reached approximately the target
            self.get_logger().warn(f'Gripper action timed out - may be gripping object')
            # Check current position to see if it moved
            if self.current_joint_state:
                try:
                    idx = self.current_joint_state.name.index('right_finger_bottom_joint')
                    current_pos = self.current_joint_state.position[idx]
                    self.get_logger().info(f'Gripper current position: {current_pos:.3f}')
                    # For closing: if position > 0.2, consider it closed enough
                    if position > 0.5 and current_pos > 0.2:
                        self.get_logger().info('Gripper appears to be gripping')
                        return True
                    # For opening: if position < 0.1, consider it open enough
                    if position < 0.1 and current_pos < 0.1:
                        self.get_logger().info('Gripper appears to be open')
                        return True
                except (ValueError, IndexError):
                    pass
            # Return True anyway to not block the demo
            self.get_logger().warn('Continuing despite gripper timeout...')
            return True
        
        result = result_future.result()
        if result is None:
            self.get_logger().error('Gripper result is None!')
            return False
        
        # Check if gripper stalled (gripping object) - this is success for closing
        stalled = getattr(result.result, 'stalled', False)
        reached = getattr(result.result, 'reached_goal', True)
        
        self.get_logger().info(f'Gripper at position: {result.result.position:.3f}, stalled: {stalled}, reached: {reached}')
        
        # For closing, stalled is OK (means we're gripping something)
        if position > 0.5:  # closing
            return True  # Accept any completion when closing
        return True

    def pick_and_place(self, station_name: str):
        """Execute complete pick and place sequence."""
        if station_name not in self.pick_stations:
            self.get_logger().error(f'Unknown station: {station_name}')
            return False
        
        station = self.pick_stations[station_name]
        self.get_logger().info(f'=== Starting pick and place for {station_name} ===')
        
        try:
            # 1. Open gripper
            self.get_logger().info('Step 1: Opening gripper')
            if not self.move_gripper(self.GRIPPER_OPEN):
                return False
            time.sleep(0.3)
            
            # 1b. Move to home position (arm forward, gripper horizontal)
            # Validated FK position: horizontal and parallel to table
            self.get_logger().info('Step 1b: Moving to home position')
            home_joints = [0.0, -1.0, -2.05, -1.615, 0.55, 0.0]
            #home_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            
            if not self.move_to_joint_positions(home_joints, duration=2.0):
                self.get_logger().warn('Home move failed, continuing to pregrasp anyway...')
            time.sleep(0.5)
            
            # 2. Move to pre-grasp position
            self.get_logger().info('Step 2: Moving to pre-grasp position')
            if not self.move_to_pose(station['pregrasp']):
                return False
            time.sleep(0.5)
            
            # 3. Move down to grasp position
            self.get_logger().info('Step 3: Moving to grasp position')
            if not self.move_to_pose(station['grasp']):
                return False
            time.sleep(0.5)
            
            # 4. Close gripper (with higher effort for firm grip)
            self.get_logger().info('Step 4: Closing gripper')
            if not self.move_gripper(self.GRIPPER_CLOSED, max_effort=100.0):
                return False
            time.sleep(1.0)  # Wait for physics to settle
            
            # 5. Lift object high
            self.get_logger().info('Step 5: Lifting object')
            lift_pose = self._create_pose(
                station['pregrasp'].position.x,
                station['pregrasp'].position.y,
                0.30  # Lift to clear obstacles (0.50 is outside workspace)
            )
            if not self.move_to_pose(lift_pose):
                return False
            time.sleep(0.2)
            
            # 6. Move above basket (high approach) - use per-object position if available
            self.get_logger().info('Step 6: Moving above basket')
            basket_approach = station.get('basket_approach', self.basket_approach)
            if not self.move_to_pose(basket_approach):
                return False
            time.sleep(0.2)
            
            # 7. Lower to basket drop position - use per-object position if available
            self.get_logger().info('Step 7: Lowering to basket')
            basket_drop = station.get('basket_drop', self.basket_position)
            if not self.move_to_pose(basket_drop):
                return False
            time.sleep(0.2)
            
            # 8. Release object
            self.get_logger().info('Step 8: Releasing object')
            if not self.move_gripper(self.GRIPPER_OPEN):
                return False
            time.sleep(0.2)
            
            # 9. Move back up above basket
            self.get_logger().info('Step 9: Moving above basket')
            if not self.move_to_pose(basket_approach):
                return False
            time.sleep(0.2)
            
            self.get_logger().info(f'=== Pick and place for {station_name} completed! ===')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Pick and place failed: {str(e)}')
            return False

    def run_demo(self, stations=None):
        """Run the demo for specified stations."""
        if stations is None:
            stations = list(self.pick_stations.keys())
        
        self.get_logger().info(f'Starting demo for stations: {stations}')
        
        successes = 0
        failures = 0
        
        for station in stations:
            self.get_logger().info(f'\n{"="*50}')
            self.get_logger().info(f'Processing: {station}')
            self.get_logger().info(f'{"="*50}\n')
            
            if self.pick_and_place(station):
                successes += 1
            else:
                failures += 1
                self.get_logger().warn(f'Failed to process {station}, continuing...')
            
            time.sleep(1.0)
        
        self.get_logger().info(f'\n{"="*50}')
        self.get_logger().info(f'Demo completed: {successes} successes, {failures} failures')
        self.get_logger().info(f'{"="*50}')
        
        # Final return to home
        self.get_logger().info('Returning arm to home position...')
        home_joints = [0.0, -1.0, -2.05, -1.615, 0.55, 0.0]
        self.move_to_joint_positions(home_joints, duration=2.0)
        self.get_logger().info('Demo finished!')


def main(args=None):
    import sys
    rclpy.init(args=args)
    
    demo = PickAndPlaceDemo()
    
    try:
        # Check for command-line arguments for specific stations
        if len(sys.argv) > 1:
            stations = sys.argv[1:]
            demo.get_logger().info(f'Running demo for specified stations: {stations}')
            demo.run_demo(stations=stations)
        else:
            # Run demo for all objects
            demo.run_demo()  # No argument = all stations
        
    except KeyboardInterrupt:
        demo.get_logger().info('Demo interrupted by user')
    finally:
        demo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
