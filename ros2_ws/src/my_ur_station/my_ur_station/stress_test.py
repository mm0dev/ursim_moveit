import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, RobotState, JointConstraint
from moveit_msgs.srv import GetPositionIK, GetStateValidity, GetCartesianPath
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
import random
import math
import sys
import numpy as np

# ===== CONFIGURATION PARAMETERS =====
NUM_POSES = 30  # Number of random poses to test
MAX_CONTROL_FAILURES = 1  # Stop test after this many consecutive controller failures
MAX_POSE_GENERATION_ATTEMPTS = 500  # Max attempts to find a valid random pose (increased for collision checking)

# Workspace bounds for random pose generation (in meters, relative to base_link)
WORKSPACE_X_MIN = 0.3
WORKSPACE_X_MAX = 0.7
WORKSPACE_Y_MIN = -0.3
WORKSPACE_Y_MAX = 0.3
WORKSPACE_Z_MIN = 0.2
WORKSPACE_Z_MAX = 0.6

# Motion parameters
MAX_VELOCITY_SCALING = 0.2  # 20% of max velocity
MAX_ACCELERATION_SCALING = 0.2  # 20% of max acceleration
ALLOWED_PLANNING_TIME = 5.0  # seconds

# Constraint tolerances
ORIENTATION_TOLERANCE = 0.1  # radians
CONSTRAINT_REGION_SIZE = 0.01  # meters (size of position constraint box)

# Safety distance between tool flange and lower arm (forearm)
MIN_TOOL_FOREARM_DISTANCE = 0.03  # 3 cm minimum distance (safety layer is 2.8cm)

# Joint limits to avoid dangerous configurations (radians)
# Only enforce limits that prevent mechanical damage - let MoveIt handle the rest
SAFE_JOINT_LIMITS = {
    'shoulder_pan_joint': (-6.28, 6.28),         # ±2π - full rotation range
    'shoulder_lift_joint': (-6.28, 6.28),        # Full range
    'elbow_joint': (-6.28, 6.28),                # Full range
    'wrist_1_joint': (-6.28, 6.28),              # Full range
    'wrist_2_joint': (-6.28, 6.28),              # Full range
    'wrist_3_joint': (-6.28, 6.28),              # Full range
}

# Predefined safe orientations (quaternions: w, x, y, z)
SAFE_ORIENTATIONS = [
    (0.0, 1.0, 0.0, 0.0),  # pointing forward/down
    (0.707, 0.707, 0.0, 0.0),  # angled
    (0.707, 0.0, 0.707, 0.0),  # rotated
    (0.0, 0.707, 0.707, 0.0),  # another angle
]
# =====================================

class StressTest(Node):
    def __init__(self, num_poses=NUM_POSES):
        super().__init__('stress_test')
        self.num_poses = num_poses
        self.current_pose_index = 0
        self.success_count = 0
        self.failure_count = 0
        self.invalid_pose_count = 0
        self.unsafe_trajectory_count = 0
        self.tool_forearm_violation_count = 0
        
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        
        # Services for pose validation - only need IK
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10
        )
        self.current_joint_state = None
        
        # UR5e kinematic parameters (DH parameters - adjust if using different UR model)
        # These define the robot's geometry for forward kinematics
        self.d1 = 0.08916  # shoulder to elbow distance (Z offset)
        self.a2 = -0.42500  # upper arm length
        self.a3 = -0.39225  # forearm length
        self.d4 = 0.10915  # wrist 1 offset
        self.d5 = 0.09465   # wrist 2 offset
        self.d6 = 0.0823    # wrist 3 to flange
        
        self.get_logger().info(f'Starting stress test with {num_poses} random poses...')
        self.get_logger().info(f'Tool-Forearm minimum distance: {MIN_TOOL_FOREARM_DISTANCE*100:.1f} cm')
        self.get_logger().info('Waiting for MoveIt services...')
        
        # Wait for IK service only
        self._action_client.wait_for_server()
        
        ik_available = self.ik_client.wait_for_service(timeout_sec=5.0)
        
        if not ik_available:
            self.get_logger().warn('⚠ IK service not available - poses will NOT be validated!')
            self.get_logger().warn('  Service: /compute_ik')
        else:
            self.get_logger().info('✓ IK service ready!')
        
        self.get_logger().info('MoveIt Server Found! Ready to plan.')
        self.get_logger().info('')

    def joint_state_callback(self, msg):
        self.current_joint_state = msg

    def forward_kinematics(self, joint_positions):
        """
        Compute forward kinematics to get link positions
        Returns positions of key links: shoulder, elbow, wrist_1, wrist_2, wrist_3, tool_flange
        """
        q = joint_positions
        
        # Base to shoulder (base_link to shoulder_link)
        T0 = np.eye(4)
        
        # Shoulder to elbow (shoulder_link to upper_arm_link to elbow)
        # Joint 1: shoulder_pan
        T1 = self.dh_transform(0, 0, self.d1, q[0])
        # Joint 2: shoulder_lift  
        T2 = self.dh_transform(-np.pi/2, 0, 0, q[1])
        T2a = self.dh_transform(0, self.a2, 0, 0)  # upper arm length
        
        # Elbow to wrist_1 (forearm_link)
        # Joint 3: elbow
        T3 = self.dh_transform(0, self.a3, 0, q[2])
        
        # Wrist joints
        T4 = self.dh_transform(-np.pi/2, 0, self.d4, q[3])  # wrist_1
        T5 = self.dh_transform(np.pi/2, 0, self.d5, q[4])   # wrist_2
        T6 = self.dh_transform(0, 0, self.d6, q[5])         # wrist_3 to flange
        
        # Compute cumulative transforms
        positions = {}
        
        T_shoulder = T0 @ T1
        positions['shoulder'] = T_shoulder[:3, 3]
        
        T_elbow = T_shoulder @ T2 @ T2a
        positions['elbow'] = T_elbow[:3, 3]
        
        T_wrist1 = T_elbow @ T3
        positions['wrist_1'] = T_wrist1[:3, 3]
        
        T_wrist2 = T_wrist1 @ T4
        positions['wrist_2'] = T_wrist2[:3, 3]
        
        T_wrist3 = T_wrist2 @ T5
        positions['wrist_3'] = T_wrist3[:3, 3]
        
        T_flange = T_wrist3 @ T6
        positions['tool_flange'] = T_flange[:3, 3]
        
        return positions

    def dh_transform(self, alpha, a, d, theta):
        """Compute DH transformation matrix"""
        return np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0,              np.sin(alpha),                 np.cos(alpha),                d],
            [0,              0,                             0,                            1]
        ])

    def check_tool_forearm_distance(self, joint_positions):
        """
        Check if the distance between tool flange and forearm (lower arm) is safe.
        Returns (is_safe, distance, reason)
        """
        try:
            # Get link positions
            positions = self.forward_kinematics(joint_positions)
            
            tool_pos = positions['tool_flange']
            elbow_pos = positions['elbow']
            wrist1_pos = positions['wrist_1']
            
            # The forearm is the link between elbow and wrist_1
            # We need to check the minimum distance from tool_flange to this line segment
            
            # Vector from elbow to wrist_1 (forearm direction)
            forearm_vec = wrist1_pos - elbow_pos
            forearm_length = np.linalg.norm(forearm_vec)
            
            if forearm_length < 0.001:
                return True, float('inf'), "Degenerate forearm"
            
            forearm_dir = forearm_vec / forearm_length
            
            # Vector from elbow to tool
            elbow_to_tool = tool_pos - elbow_pos
            
            # Project tool position onto forearm line to find closest point
            projection_length = np.dot(elbow_to_tool, forearm_dir)
            
            # Clamp to forearm segment (between elbow and wrist_1)
            projection_length = np.clip(projection_length, 0, forearm_length)
            
            # Closest point on forearm to tool
            closest_point = elbow_pos + projection_length * forearm_dir
            
            # Distance from tool to closest point on forearm
            distance = np.linalg.norm(tool_pos - closest_point)
            
            if distance < MIN_TOOL_FOREARM_DISTANCE:
                return False, distance, f"Tool-forearm distance {distance*100:.2f}cm < {MIN_TOOL_FOREARM_DISTANCE*100:.1f}cm"
            
            return True, distance, "Safe distance"
            
        except Exception as e:
            self.get_logger().error(f"Error computing tool-forearm distance: {e}")
            return False, 0.0, str(e)

    def generate_random_pose(self):
        """Generate a random pose within the robot's workspace"""
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = 'base_link'
        
        # Random position within configured workspace bounds
        pose_goal.pose.position.x = random.uniform(WORKSPACE_X_MIN, WORKSPACE_X_MAX)
        pose_goal.pose.position.y = random.uniform(WORKSPACE_Y_MIN, WORKSPACE_Y_MAX)
        pose_goal.pose.position.z = random.uniform(WORKSPACE_Z_MIN, WORKSPACE_Z_MAX)
        
        # Random orientation from predefined safe orientations
        orientation = random.choice(SAFE_ORIENTATIONS)
        pose_goal.pose.orientation.w = orientation[0]
        pose_goal.pose.orientation.x = orientation[1]
        pose_goal.pose.orientation.y = orientation[2]
        pose_goal.pose.orientation.z = orientation[3]
        
        return pose_goal

    def check_joint_limits(self, joint_state):
        """Check if joint configuration is within safe limits"""
        for i, joint_name in enumerate(joint_state.name):
            if joint_name in SAFE_JOINT_LIMITS:
                joint_value = joint_state.position[i]
                min_limit, max_limit = SAFE_JOINT_LIMITS[joint_name]
                
                if joint_value < min_limit or joint_value > max_limit:
                    return False, f"Joint {joint_name} at {joint_value:.3f} exceeds safe limits [{min_limit:.3f}, {max_limit:.3f}]"
        
        return True, "Within safe limits"

    def validate_pose(self, pose_stamped):
        """
        Validate if a pose is achievable by checking:
        1. IK solution exists
        2. Tool-forearm distance is safe (>3cm)
        
        Returns: (success, message, joint_solution)
        If successful, returns the IK joint solution that should be used for execution.
        """
        # Check IK solution
        ik_request = GetPositionIK.Request()
        ik_request.ik_request.group_name = 'ur_manipulator'
        ik_request.ik_request.pose_stamped = pose_stamped
        ik_request.ik_request.avoid_collisions = True  # Important: avoid collisions
        ik_request.ik_request.timeout.sec = 1  # Integer, not float
        ik_request.ik_request.timeout.nanosec = 0
        
        # Use current joint state as seed for IK
        ik_request.ik_request.robot_state = RobotState()
        ik_request.ik_request.robot_state.joint_state = self.current_joint_state
        
        try:
            ik_future = self.ik_client.call_async(ik_request)
            rclpy.spin_until_future_complete(self, ik_future, timeout_sec=1.0)
            
            if not ik_future.done():
                return False, "IK service timeout", None
            
            ik_response = ik_future.result()
            
            # Check if IK succeeded
            if ik_response.error_code.val != 1:  # 1 = SUCCESS
                return False, f"No IK solution found", None
            
            # Check tool-forearm distance - THIS IS THE KEY SAFETY CHECK
            joint_positions = list(ik_response.solution.joint_state.position[:6])
            is_distance_safe, distance, distance_reason = self.check_tool_forearm_distance(joint_positions)
            if not is_distance_safe:
                self.tool_forearm_violation_count += 1
                return False, distance_reason, None
            
            return True, f"Valid pose (tool-forearm: {distance*100:.2f}cm)", ik_response.solution.joint_state
            
        except Exception as e:
            return False, f"Validation error: {str(e)}", None

    def validate_path_to_pose(self, target_pose):
        """
        Check if the path from current position to target is safe
        by computing a cartesian path and validating each waypoint
        """
        try:
            cartesian_request = GetCartesianPath.Request()
            cartesian_request.header.frame_id = 'base_link'
            cartesian_request.group_name = 'ur_manipulator'
            
            # Start from current state
            cartesian_request.start_state = RobotState()
            cartesian_request.start_state.joint_state = self.current_joint_state
            
            # Target waypoint
            cartesian_request.waypoints = [target_pose.pose]
            cartesian_request.max_step = 0.01  # 1cm resolution
            cartesian_request.jump_threshold = 0.0  # No jumping allowed
            cartesian_request.avoid_collisions = True
            
            cartesian_future = self.cartesian_path_client.call_async(cartesian_request)
            rclpy.spin_until_future_complete(self, cartesian_future, timeout_sec=3.0)
            
            if not cartesian_future.done():
                return False, "Cartesian path service timeout"
            
            cartesian_response = cartesian_future.result()
            
            # Check if path was found with reasonable success
            if cartesian_response.fraction < 0.95:  # Less than 95% of path is valid
                return False, f"Only {cartesian_response.fraction*100:.1f}% of path is collision-free"
            
            # Validate each waypoint in the trajectory
            for i, point in enumerate(cartesian_response.solution.joint_trajectory.points):
                # Create joint state for this waypoint
                waypoint_state = JointState()
                waypoint_state.name = cartesian_response.solution.joint_trajectory.joint_names
                waypoint_state.position = list(point.positions)
                
                # Check if within safe joint limits
                is_safe, reason = self.check_joint_limits(waypoint_state)
                if not is_safe:
                    self.unsafe_trajectory_count += 1
                    return False, f"Waypoint {i}: {reason}"
                
                # *** NEW: Check tool-forearm distance at each waypoint ***
                joint_positions = list(point.positions[:6])
                is_distance_safe, distance, distance_reason = self.check_tool_forearm_distance(joint_positions)
                if not is_distance_safe:
                    self.unsafe_trajectory_count += 1
                    self.tool_forearm_violation_count += 1
                    return False, f"Waypoint {i}: {distance_reason}"
            
            return True, "Path validated"
            
        except Exception as e:
            return False, f"Path validation error: {str(e)}"

    def generate_valid_pose(self):
        """Generate a random pose and validate it. Retry if invalid. Returns (pose, joint_solution)"""
        for attempt in range(MAX_POSE_GENERATION_ATTEMPTS):
            pose = self.generate_random_pose()
            is_valid, reason, joint_solution = self.validate_pose(pose)
            
            if is_valid:
                self.get_logger().info(f'✓ Valid pose found (attempt {attempt + 1}) - {reason}')
                return pose, joint_solution
            else:
                self.invalid_pose_count += 1
                if attempt % 10 == 0 and attempt > 0:
                    self.get_logger().warn(f'  Attempt {attempt}: {reason}')
        
        self.get_logger().error(f'Failed to generate valid pose after {MAX_POSE_GENERATION_ATTEMPTS} attempts!')
        return None, None

    def send_next_goal(self):
        if self.current_pose_index >= self.num_poses:
            self.get_logger().info('='*60)
            self.get_logger().info(f'STRESS TEST COMPLETE!')
            self.get_logger().info(f'Total Poses: {self.num_poses}')
            self.get_logger().info(f'Successful: {self.success_count}')
            self.get_logger().info(f'Failed: {self.failure_count}')
            self.get_logger().info(f'Invalid Poses Generated: {self.invalid_pose_count}')
            self.get_logger().info(f'Unsafe Trajectories Rejected: {self.unsafe_trajectory_count}')
            self.get_logger().info(f'Tool-Forearm Violations: {self.tool_forearm_violation_count}')
            self.get_logger().info(f'Success Rate: {100*self.success_count/self.num_poses:.1f}%')
            self.get_logger().info('='*60)
            rclpy.shutdown()
            return

        # Wait until we have heard from the robot
        while self.current_joint_state is None and rclpy.ok():
            self.get_logger().info('Waiting for robot current state...')
            rclpy.spin_once(self, timeout_sec=0.1)

        self.current_pose_index += 1
        self.get_logger().info('='*60)
        self.get_logger().info(f'POSE {self.current_pose_index}/{self.num_poses}')
        
        # Generate and VALIDATE random pose
        pose_goal, joint_solution = self.generate_valid_pose()
        
        if pose_goal is None or joint_solution is None:
            self.get_logger().error('Skipping this pose and moving to next...')
            self.failure_count += 1
            self.send_next_goal()
            return
        
        self.get_logger().info(f'Target: x={pose_goal.pose.position.x:.3f}, '
                              f'y={pose_goal.pose.position.y:.3f}, '
                              f'z={pose_goal.pose.position.z:.3f}')

        goal_msg = MoveGroup.Goal()
        goal_msg.request.workspace_parameters.header.frame_id = 'base_link'
        goal_msg.request.group_name = 'ur_manipulator'
        goal_msg.request.allowed_planning_time = ALLOWED_PLANNING_TIME
        
        # Set start state explicitly
        goal_msg.request.start_state = RobotState()
        goal_msg.request.start_state.joint_state = self.current_joint_state

        # Speed limiting for smooth motion
        goal_msg.request.max_velocity_scaling_factor = MAX_VELOCITY_SCALING
        goal_msg.request.max_acceleration_scaling_factor = MAX_ACCELERATION_SCALING
        
        # Use joint goal from validated IK solution instead of pose constraints
        # This avoids the "position/orientation constraint violated" errors
        constraints = Constraints()
        
        # Add joint constraints for each joint in the validated solution
        for i_joint, joint_name in enumerate(joint_solution.name):
            if any(j in joint_name for j in ['shoulder', 'elbow', 'wrist']):
                jc = JointConstraint()
                jc.joint_name = joint_name
                jc.position = joint_solution.position[i_joint]
                jc.tolerance_above = 0.01  # 0.01 rad tolerance
                jc.tolerance_below = 0.01
                jc.weight = 1.0
                constraints.joint_constraints.append(jc)
        
        goal_msg.request.goal_constraints.append(constraints)

        self.get_logger().info('Sending validated joint goal to MoveIt...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected!')
            self.failure_count += 1
            self.send_next_goal()
            return

        self.get_logger().info('Goal accepted! Executing...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        
        if result.error_code.val == 1:
            self.get_logger().info('SUCCESS: Motion Completed!')
            self.success_count += 1
        elif result.error_code.val == -4:
            self.get_logger().error('FAILED: CONTROL_FAILED (Error Code -4)')
            self.failure_count += 1
            
            if self.failure_count >= MAX_CONTROL_FAILURES:
                self.get_logger().error('TOO MANY CONTROL FAILURES - STOPPING TEST')
                rclpy.shutdown()
                return
        else:
            self.get_logger().warn(f'FAILED: Error Code {result.error_code.val}')
            self.failure_count += 1
        
        self.send_next_goal()

def main(args=None):
    rclpy.init(args=args)
    node = StressTest(num_poses=NUM_POSES)
    node.send_next_goal()
    rclpy.spin(node)

if __name__ == '__main__':
    main()