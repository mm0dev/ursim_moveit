import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
# --- ADDED RobotState to imports ---
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, PlanningScene, CollisionObject, AttachedCollisionObject, RobotState
from geometry_msgs.msg import PoseStamped, Pose
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import random
import math
import time

# =============================================================================
#                                CONFIGURATION
# =============================================================================

WORKSPACE_CONFIG = {
    'min_radius': 0.4,     # Increased slightly to avoid self-collision at start
    'max_radius': 0.7,
    'min_height': 0.2,
    'max_height': 0.6
}

VELOCITY_FACTOR = 0.2      # Slower for safety
ACCELERATION_FACTOR = 0.2

PLANNING_TIME = 2.0        
PLANNING_ATTEMPTS = 5
PLANNER_ID = "RRTConnectkConfigDefault"

TOLERANCE_POS = 0.01
TOLERANCE_ORT = 0.1

TOTAL_SUCCESSFUL_MOVES = 100

# =============================================================================
#                                  LOGIC
# =============================================================================

def get_quaternion_from_euler(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return [qx, qy, qz, qw]

class UnchainedMover(Node):
    def __init__(self):
        super().__init__('unchained_mover')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self._scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
        self.joint_state_sub = self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)
        self.current_joint_state = None
        
        self.get_logger().info('Waiting for MoveIt server...')
        self._action_client.wait_for_server()
        
        # --- CRITICAL: ADD THE SAFETY BUBBLE ---
        self.setup_safety_bubble()
        time.sleep(2.0) # Give MoveIt time to register the bubble
        
        self.get_logger().info(f'System Ready. Safety Bubble Active.')

    def joint_state_callback(self, msg):
        self.current_joint_state = msg

    def setup_safety_bubble(self):
        """
        Attaches a virtual cylinder to the tool flange.
        """
        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        
        shield = AttachedCollisionObject()
        shield.link_name = "tool0"
        shield.object.header.frame_id = "tool0"
        shield.object.id = "safety_bubble"
        
        # Radius 0.07m (7cm) - Slightly reduced from 8cm to prevent immediate self-collision
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.CYLINDER
        primitive.dimensions = [0.15, 0.07] 
        
        shield.object.primitives.append(primitive)
        
        pose = Pose()
        pose.position.z = -0.05 
        pose.orientation.w = 1.0
        shield.object.primitive_poses.append(pose)
        
        scene_msg.robot_state.is_diff = True
        scene_msg.robot_state.attached_collision_objects.append(shield)
        
        self.get_logger().info('Publishing Safety Bubble...')
        self._scene_pub.publish(scene_msg)

    def get_random_pose(self):
        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        
        radius = random.uniform(WORKSPACE_CONFIG['min_radius'], WORKSPACE_CONFIG['max_radius'])
        angle = random.uniform(-math.pi, math.pi)
        
        pose.pose.position.x = radius * math.cos(angle)
        pose.pose.position.y = radius * math.sin(angle)
        pose.pose.position.z = random.uniform(WORKSPACE_CONFIG['min_height'], WORKSPACE_CONFIG['max_height'])
        
        # Orientation: Pointing roughly down/forward
        roll = random.uniform(-3.14, 3.14)
        pitch = random.uniform(-0.5, 0.5) 
        yaw = random.uniform(-3.14, 3.14)
        
        q = get_quaternion_from_euler(roll, pitch, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        
        return pose

    def find_and_execute_path(self, target_pose, move_id):
        # 1. Wait for valid joint state
        while self.current_joint_state is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

        goal_msg = MoveGroup.Goal()
        goal_msg.request.workspace_parameters.header.frame_id = 'base_link'
        goal_msg.request.group_name = 'ur_manipulator'
        goal_msg.request.allowed_planning_time = PLANNING_TIME
        goal_msg.request.num_planning_attempts = PLANNING_ATTEMPTS
        goal_msg.request.planner_id = PLANNER_ID
        
        # --- THE FIX: Explicitly tell MoveIt where the robot is NOW ---
        goal_msg.request.start_state = RobotState()
        goal_msg.request.start_state.joint_state = self.current_joint_state
        # --------------------------------------------------------------

        goal_msg.request.max_velocity_scaling_factor = VELOCITY_FACTOR
        goal_msg.request.max_acceleration_scaling_factor = ACCELERATION_FACTOR

        constraints = Constraints()
        
        pc = PositionConstraint()
        pc.header.frame_id = 'base_link'
        pc.link_name = 'tool0'
        pc.target_point_offset.x = 0.0; pc.target_point_offset.y = 0.0; pc.target_point_offset.z = 0.0
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [TOLERANCE_POS, TOLERANCE_POS, TOLERANCE_POS]
        pc.constraint_region.primitives.append(primitive)
        pc.constraint_region.primitive_poses.append(target_pose.pose)
        pc.weight = 1.0
        constraints.position_constraints.append(pc)

        oc = OrientationConstraint()
        oc.header.frame_id = 'base_link'
        oc.link_name = 'tool0'
        oc.orientation = target_pose.pose.orientation
        oc.absolute_x_axis_tolerance = TOLERANCE_ORT
        oc.absolute_y_axis_tolerance = TOLERANCE_ORT
        oc.absolute_z_axis_tolerance = TOLERANCE_ORT
        oc.weight = 1.0
        constraints.orientation_constraints.append(oc)

        goal_msg.request.goal_constraints.append(constraints)

        # Plan Only Check
        goal_msg.planning_options.plan_only = True
        goal_msg.planning_options.look_around_attempts = 5
        goal_msg.planning_options.replan = True
        
        self.get_logger().info(f'[Move {move_id}] Computing path...')
        plan_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, plan_future)
        plan_handle = plan_future.result()
        
        if not plan_handle.accepted:
            return False

        get_result = plan_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result)
        plan_result = get_result.result().result
        
        # Check Success Code
        if plan_result.error_code.val != 1:
            self.get_logger().info(f'[Move {move_id}] Failed. Code: {plan_result.error_code.val}')
            return False

        # Execute
        self.get_logger().info(f'[Move {move_id}] Path Valid. Executing...')
        goal_msg.planning_options.plan_only = False
        exec_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, exec_future)
        exec_handle = exec_future.result()
        
        if not exec_handle.accepted:
            return False
            
        get_exec_result = exec_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_exec_result)
        final_result = get_exec_result.result().result
        
        return final_result.error_code.val == 1

def main(args=None):
    rclpy.init(args=args)
    mover = UnchainedMover()
    
    successful_moves = 0
    
    while successful_moves < TOTAL_SUCCESSFUL_MOVES:
        target = mover.get_random_pose()
        if mover.find_and_execute_path(target, successful_moves + 1):
            successful_moves += 1
            mover.get_logger().info(f'---> SUCCESS COUNT: {successful_moves}/{TOTAL_SUCCESSFUL_MOVES}')
        else:
            # Add a small sleep to prevent log flooding on failures
            time.sleep(0.5) 

    mover.get_logger().info(f'TEST COMPLETE.')
    mover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()