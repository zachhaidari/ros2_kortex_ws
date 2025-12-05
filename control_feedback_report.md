# ROS2 Control & Feedback Report
Generated: Tue Nov 18 04:52:16 AM UTC 2025

## Environment
- ROS_DOMAIN_ID: 0

## Nodes (all)
WARNING: Be aware that there are nodes in the graph that share an exact name, which can have unintended side effects.
/controller_manager
/gen3_lite_2f_gripper_controller
/gz_ros_control
/interactive_marker_display_108590422208880
/joint_state_broadcaster
/joint_trajectory_controller
/move_group
/move_group/moveit
/move_group_private_99484275129696
/moveit_1369027465
/moveit_simple_controller_manager
/robot_state_publisher
/ros_gz_bridge
/ros_gz_bridge
/rviz2
/rviz2
/rviz2_private_134378460199680
/transform_listener_impl_5a7afda837d0
/transform_listener_impl_62c32d2d9450
/transform_listener_impl_7a376c5df740
/twist_controller

## Topics (all)
/attached_collision_object
/clock
/collision_object
/controller_manager/activity
/controller_manager/introspection_data/full
/controller_manager/introspection_data/names
/controller_manager/introspection_data/values
/controller_manager/statistics/full
/controller_manager/statistics/names
/controller_manager/statistics/values
/diagnostics
/display_contacts
/display_planned_path
/dynamic_joint_states
/gen3_lite_2f_gripper_controller/transition_event
/joint_state_broadcaster/transition_event
/joint_states
/joint_trajectory_controller/controller_state
/joint_trajectory_controller/joint_trajectory
/joint_trajectory_controller/transition_event
/monitored_planning_scene
/parameter_events
/pipeline_state
/planning_scene
/planning_scene_world
/recognized_object_array
/robot_description
/robot_description_semantic
/rosout
/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback
/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update
/tf
/tf_static
/trajectory_execution_event
/twist_controller/commands
/twist_controller/transition_event
/wrist_mounted_camera/camera_info
/wrist_mounted_camera/depth_image
/wrist_mounted_camera/image
/wrist_mounted_camera/points

## Services (all)
/apply_planning_scene
/check_state_validity
/clear_octomap
/compute_cartesian_path
/compute_fk
/compute_ik
/controller_manager/configure_controller
/controller_manager/describe_parameters
/controller_manager/get_logger_levels
/controller_manager/get_parameter_types
/controller_manager/get_parameters
/controller_manager/get_type_description
/controller_manager/list_controller_types
/controller_manager/list_controllers
/controller_manager/list_hardware_components
/controller_manager/list_hardware_interfaces
/controller_manager/list_parameters
/controller_manager/load_controller
/controller_manager/reload_controller_libraries
/controller_manager/set_hardware_component_state
/controller_manager/set_logger_levels
/controller_manager/set_parameters
/controller_manager/set_parameters_atomically
/controller_manager/switch_controller
/controller_manager/unload_controller
/gen3_lite_2f_gripper_controller/describe_parameters
/gen3_lite_2f_gripper_controller/get_logger_levels
/gen3_lite_2f_gripper_controller/get_parameter_types
/gen3_lite_2f_gripper_controller/get_parameters
/gen3_lite_2f_gripper_controller/get_type_description
/gen3_lite_2f_gripper_controller/list_parameters
/gen3_lite_2f_gripper_controller/set_logger_levels
/gen3_lite_2f_gripper_controller/set_parameters
/gen3_lite_2f_gripper_controller/set_parameters_atomically
/get_planner_params
/get_planning_scene
/get_urdf
/gz_ros_control/describe_parameters
/gz_ros_control/get_parameter_types
/gz_ros_control/get_parameters
/gz_ros_control/get_type_description
/gz_ros_control/list_parameters
/gz_ros_control/set_parameters
/gz_ros_control/set_parameters_atomically
/interactive_marker_display_108590422208880/describe_parameters
/interactive_marker_display_108590422208880/get_parameter_types
/interactive_marker_display_108590422208880/get_parameters
/interactive_marker_display_108590422208880/get_type_description
/interactive_marker_display_108590422208880/list_parameters
/interactive_marker_display_108590422208880/set_parameters
/interactive_marker_display_108590422208880/set_parameters_atomically
/joint_state_broadcaster/describe_parameters
/joint_state_broadcaster/get_logger_levels
/joint_state_broadcaster/get_parameter_types
/joint_state_broadcaster/get_parameters
/joint_state_broadcaster/get_type_description
/joint_state_broadcaster/list_parameters
/joint_state_broadcaster/set_logger_levels
/joint_state_broadcaster/set_parameters
/joint_state_broadcaster/set_parameters_atomically
/joint_trajectory_controller/describe_parameters
/joint_trajectory_controller/get_logger_levels
/joint_trajectory_controller/get_parameter_types
/joint_trajectory_controller/get_parameters
/joint_trajectory_controller/get_type_description
/joint_trajectory_controller/list_parameters
/joint_trajectory_controller/query_state
/joint_trajectory_controller/set_logger_levels
/joint_trajectory_controller/set_parameters
/joint_trajectory_controller/set_parameters_atomically
/load_geometry_from_file
/load_map
/move_group/describe_parameters
/move_group/get_parameter_types
/move_group/get_parameters
/move_group/get_type_description
/move_group/list_parameters
/move_group/moveit/describe_parameters
/move_group/moveit/get_parameter_types
/move_group/moveit/get_parameters
/move_group/moveit/get_type_description
/move_group/moveit/list_parameters
/move_group/moveit/set_parameters
/move_group/moveit/set_parameters_atomically
/move_group/set_parameters
/move_group/set_parameters_atomically
/move_group_private_99484275129696/describe_parameters
/move_group_private_99484275129696/get_parameter_types
/move_group_private_99484275129696/get_parameters
/move_group_private_99484275129696/get_type_description
/move_group_private_99484275129696/list_parameters
/move_group_private_99484275129696/set_parameters
/move_group_private_99484275129696/set_parameters_atomically
/moveit_1369027465/describe_parameters
/moveit_1369027465/get_parameter_types
/moveit_1369027465/get_parameters
/moveit_1369027465/get_type_description
/moveit_1369027465/list_parameters
/moveit_1369027465/set_parameters
/moveit_1369027465/set_parameters_atomically
/moveit_simple_controller_manager/describe_parameters
/moveit_simple_controller_manager/get_parameter_types
/moveit_simple_controller_manager/get_parameters
/moveit_simple_controller_manager/get_type_description
/moveit_simple_controller_manager/list_parameters
/moveit_simple_controller_manager/set_parameters
/moveit_simple_controller_manager/set_parameters_atomically
/plan_kinematic_path
/plan_sequence_path
/query_planner_interface
/robot_state_publisher/describe_parameters
/robot_state_publisher/get_parameter_types
/robot_state_publisher/get_parameters
/robot_state_publisher/get_type_description
/robot_state_publisher/list_parameters
/robot_state_publisher/set_parameters
/robot_state_publisher/set_parameters_atomically
/ros_gz_bridge/describe_parameters
/ros_gz_bridge/get_parameter_types
/ros_gz_bridge/get_parameters
/ros_gz_bridge/get_type_description
/ros_gz_bridge/list_parameters
/ros_gz_bridge/set_parameters
/ros_gz_bridge/set_parameters_atomically
/rviz2/describe_parameters
/rviz2/get_parameter_types
/rviz2/get_parameters
/rviz2/get_type_description
/rviz2/list_parameters
/rviz2/reset_time
/rviz2/set_parameters
/rviz2/set_parameters_atomically
/rviz2_private_134378460199680/describe_parameters
/rviz2_private_134378460199680/get_parameter_types
/rviz2_private_134378460199680/get_parameters
/rviz2_private_134378460199680/get_type_description
/rviz2_private_134378460199680/list_parameters
/rviz2_private_134378460199680/set_parameters
/rviz2_private_134378460199680/set_parameters_atomically
/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/get_interactive_markers
/save_geometry_to_file
/save_map
/set_planner_params
/transform_listener_impl_5a7afda837d0/get_type_description
/transform_listener_impl_62c32d2d9450/get_type_description
/transform_listener_impl_7a376c5df740/get_type_description
/twist_controller/describe_parameters
/twist_controller/get_logger_levels
/twist_controller/get_parameter_types
/twist_controller/get_parameters
/twist_controller/get_type_description
/twist_controller/list_parameters
/twist_controller/set_logger_levels
/twist_controller/set_parameters
/twist_controller/set_parameters_atomically

## Actions (all)
/execute_trajectory
/gen3_lite_2f_gripper_controller/gripper_cmd
/joint_trajectory_controller/follow_joint_trajectory
/move_action
/sequence_move_group

## Controller Manager - list_controllers
waiting for service to become available...
requester: making request: controller_manager_msgs.srv.ListControllers_Request()

response:
controller_manager_msgs.srv.ListControllers_Response(controller=[controller_manager_msgs.msg.ControllerState(name='gen3_lite_2f_gripper_controller', state='active', type='position_controllers/GripperActionController', is_async=False, update_rate=1000, claimed_interfaces=['right_finger_bottom_joint/position'], required_command_interfaces=['right_finger_bottom_joint/position'], required_state_interfaces=['right_finger_bottom_joint/position', 'right_finger_bottom_joint/velocity'], is_chainable=False, is_chained=False, exported_state_interfaces=[], reference_interfaces=[], chain_connections=[]), controller_manager_msgs.msg.ControllerState(name='joint_trajectory_controller', state='active', type='joint_trajectory_controller/JointTrajectoryController', is_async=False, update_rate=1000, claimed_interfaces=['joint_1/position', 'joint_2/position', 'joint_3/position', 'joint_4/position', 'joint_5/position', 'joint_6/position'], required_command_interfaces=['joint_1/position', 'joint_2/position', 'joint_3/position', 'joint_4/position', 'joint_5/position', 'joint_6/position'], required_state_interfaces=['joint_1/position', 'joint_1/velocity', 'joint_2/position', 'joint_2/velocity', 'joint_3/position', 'joint_3/velocity', 'joint_4/position', 'joint_4/velocity', 'joint_5/position', 'joint_5/velocity', 'joint_6/position', 'joint_6/velocity'], is_chainable=False, is_chained=False, exported_state_interfaces=[], reference_interfaces=[], chain_connections=[]), controller_manager_msgs.msg.ControllerState(name='joint_state_broadcaster', state='active', type='joint_state_broadcaster/JointStateBroadcaster', is_async=False, update_rate=1000, claimed_interfaces=[], required_command_interfaces=[], required_state_interfaces=['joint_1/effort', 'joint_1/position', 'joint_1/velocity', 'joint_2/effort', 'joint_2/position', 'joint_2/velocity', 'joint_3/effort', 'joint_3/position', 'joint_3/velocity', 'joint_4/effort', 'joint_4/position', 'joint_4/velocity', 'joint_5/effort', 'joint_5/position', 'joint_5/velocity', 'joint_6/effort', 'joint_6/position', 'joint_6/velocity', 'right_finger_bottom_joint/position', 'right_finger_bottom_joint/velocity'], is_chainable=False, is_chained=False, exported_state_interfaces=[], reference_interfaces=[], chain_connections=[]), controller_manager_msgs.msg.ControllerState(name='twist_controller', state='inactive', type='picknik_twist_controller/PicknikTwistController', is_async=False, update_rate=1000, claimed_interfaces=[], required_command_interfaces=['tcp/twist.linear.x', 'tcp/twist.linear.y', 'tcp/twist.linear.z', 'tcp/twist.angular.x', 'tcp/twist.angular.y', 'tcp/twist.angular.z'], required_state_interfaces=[], is_chainable=False, is_chained=False, exported_state_interfaces=[], reference_interfaces=[], chain_connections=[])])


## Topic details (filtered: control & feedback)
## Node details (filtered: control & feedback)
## Service details (filtered: control & feedback)
