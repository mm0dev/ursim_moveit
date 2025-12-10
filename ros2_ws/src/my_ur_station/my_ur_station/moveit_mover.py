import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, RobotState
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState

class MoveItMover(Node):
    def __init__(self):
        super().__init__('moveit_mover')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        # We need to listen to the robot to know where it starts
        self.joint_state_sub = self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)
        self.current_joint_state = None
        
        self.get_logger().info('Waiting for MoveIt action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('MoveIt Server Found! Ready to plan.')

    def joint_state_callback(self, msg):
        self.current_joint_state = msg

    def send_goal(self):
        # Wait until we have heard from the robot at least once
        while self.current_joint_state is None and rclpy.ok():
            self.get_logger().info('Waiting for robot current state...')
            rclpy.spin_once(self, timeout_sec=0.1)

        goal_msg = MoveGroup.Goal()
        goal_msg.request.workspace_parameters.header.frame_id = 'base_link'
        goal_msg.request.group_name = 'ur_manipulator'
        goal_msg.request.allowed_planning_time = 5.0
        
        # --- FIX 1: SET START STATE EXPLICITLY ---
        # This tells MoveIt: "Plan from EXACTLY where the robot is right now"
        # This prevents the "velocity limit" error caused by mismatch.
        goal_msg.request.start_state = RobotState()
        goal_msg.request.start_state.joint_state = self.current_joint_state

        # --- FIX 2: SPEED LIMITING ---
        # Scale velocity and acceleration down to 10% (0.1) to ensure smooth startup
        goal_msg.request.max_velocity_scaling_factor = 0.1
        goal_msg.request.max_acceleration_scaling_factor = 0.1
        
# --- DEFINE TARGET POSE (SAFER) ---
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = 'base_link'
        
        # Move further out (X=0.6) so we don't hit the body
        pose_goal.pose.position.x = 0.6  
        pose_goal.pose.position.y = 0.0  # Centered
        pose_goal.pose.position.z = 0.4
        
        # Orientation: Pointing straight forward/down
        pose_goal.pose.orientation.w = 0.0
        pose_goal.pose.orientation.x = 1.0
        pose_goal.pose.orientation.y = 0.0
        pose_goal.pose.orientation.z = 0.0

        # --- CONSTRAINTS ---
        oc = OrientationConstraint()
        oc.header.frame_id = 'base_link'
        oc.link_name = 'tool0'
        oc.orientation = pose_goal.pose.orientation
        oc.absolute_x_axis_tolerance = 0.1
        oc.absolute_y_axis_tolerance = 0.1
        oc.absolute_z_axis_tolerance = 0.1
        oc.weight = 1.0

        pc = PositionConstraint()
        pc.header.frame_id = 'base_link'
        pc.link_name = 'tool0'
        pc.target_point_offset.x = 0.0
        pc.target_point_offset.y = 0.0
        pc.target_point_offset.z = 0.0
        
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.01, 0.01, 0.01]
        pc.constraint_region.primitives.append(primitive)
        pc.constraint_region.primitive_poses.append(pose_goal.pose)
        pc.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pc)
        constraints.orientation_constraints.append(oc)
        goal_msg.request.goal_constraints.append(constraints)

        self.get_logger().info('Sending smooth trajectory goal...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted! Executing...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code.val == 1:
            self.get_logger().info('SUCCESS: Motion Completed!')
        else:
            self.get_logger().info(f'FAILED: Error Code {result.error_code.val}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = MoveItMover()
    node.send_goal()
    rclpy.spin(node)

if __name__ == '__main__':
    main()