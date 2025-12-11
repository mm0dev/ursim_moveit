#!/usr/bin/env python3
"""
Keyboard Teleoperation for UR Robot using MoveIt with IKFast
Controls the end-effector position in Cartesian space using keyboard input.

Controls:
  W/S - Move forward/backward (X axis)
  A/D - Move left/right (Y axis)
  Q/E - Move up/down (Z axis)
  I/K - Rotate around X axis
  J/L - Rotate around Z axis
  R - Reset to home position
  ESC/X - Exit

Press SPACE to execute the planned motion after adjusting the target pose.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, RobotState
from geometry_msgs.msg import PoseStamped, Pose
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
import sys
import select
import termios
import tty
from math import cos, sin, pi, sqrt, atan2, asin


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        
        # Action client for MoveIt
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        
        # Subscribe to joint states to get current robot state
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10)
        self.current_joint_state = None
        
        # Current target pose (will be initialized from robot's current pose)
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = 'base_link'
        
        # Default home position
        self.home_pose = Pose()
        self.home_pose.position.x = 0.4
        self.home_pose.position.y = 0.0
        self.home_pose.position.z = 0.4
        self.home_pose.orientation.w = 0.0
        self.home_pose.orientation.x = 1.0
        self.home_pose.orientation.y = 0.0
        self.home_pose.orientation.z = 0.0
        
        # Initialize target to home
        self.target_pose.pose = self.home_pose
        
        # Movement parameters
        self.linear_step = 0.02  # 2cm steps
        self.angular_step = 0.1  # ~5.7 degrees
        
        # Terminal settings for keyboard input
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info('Waiting for MoveIt action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('MoveIt Server Found! Ready for keyboard control.')
        
        self.print_instructions()
        
    def joint_state_callback(self, msg):
        """Store current joint state."""
        self.current_joint_state = msg
    
    def print_instructions(self):
        """Print control instructions."""
        print("\n" + "="*60)
        print("KEYBOARD TELEOP - MoveIt with IKFast")
        print("="*60)
        print("TRANSLATION:")
        print("  W/S - Move forward/backward (X axis)")
        print("  A/D - Move left/right (Y axis)")
        print("  Q/E - Move up/down (Z axis)")
        print("\nROTATION:")
        print("  I/K - Pitch up/down (rotate around Y axis)")
        print("  J/L - Yaw left/right (rotate around Z axis)")
        print("  U/O - Roll left/right (rotate around X axis)")
        print("\nCOMMANDS:")
        print("  SPACE - Execute movement to current target pose")
        print("  R - Reset to home position")
        print("  P - Print current target pose")
        print("  +/- - Increase/decrease step size")
        print("  X/ESC - Exit")
        print("="*60)
        print(f"\nCurrent step size: {self.linear_step*1000:.1f}mm / {self.angular_step*180/pi:.1f}°")
        print("Ready! Press keys to adjust target, SPACE to execute.\n")
    
    def get_key(self):
        """Get keyboard input (non-blocking)."""
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def quaternion_to_euler(self, quat):
        """Convert quaternion to Euler angles (roll, pitch, yaw)."""
        # Convert quaternion to Euler angles using standard formulas
        x, y, z, w = quat.x, quat.y, quat.z, quat.w
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = pi / 2 if sinp > 0 else -pi / 2  # Use 90 degrees if out of range
        else:
            pitch = asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = atan2(siny_cosp, cosy_cosp)
        
        return [roll, pitch, yaw]
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion."""
        # Convert Euler angles to quaternion using standard formulas
        cy = cos(yaw * 0.5)
        sy = sin(yaw * 0.5)
        cp = cos(pitch * 0.5)
        sp = sin(pitch * 0.5)
        cr = cos(roll * 0.5)
        sr = sin(roll * 0.5)
        
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        
        return [x, y, z, w]
    
    def update_pose_translation(self, dx=0.0, dy=0.0, dz=0.0):
        """Update target pose with translation."""
        self.target_pose.pose.position.x += dx
        self.target_pose.pose.position.y += dy
        self.target_pose.pose.position.z += dz
        
        print(f"Target Position: X={self.target_pose.pose.position.x:.3f}, "
              f"Y={self.target_pose.pose.position.y:.3f}, "
              f"Z={self.target_pose.pose.position.z:.3f}")
    
    def update_pose_rotation(self, droll=0.0, dpitch=0.0, dyaw=0.0):
        """Update target pose with rotation."""
        # Get current orientation as Euler angles
        current_euler = self.quaternion_to_euler(self.target_pose.pose.orientation)
        
        # Apply delta
        new_euler = [
            current_euler[0] + droll,
            current_euler[1] + dpitch,
            current_euler[2] + dyaw
        ]
        
        # Convert back to quaternion
        quat = self.euler_to_quaternion(*new_euler)
        self.target_pose.pose.orientation.x = quat[0]
        self.target_pose.pose.orientation.y = quat[1]
        self.target_pose.pose.orientation.z = quat[2]
        self.target_pose.pose.orientation.w = quat[3]
        
        print(f"Target Orientation (RPY): Roll={new_euler[0]*180/pi:.1f}°, "
              f"Pitch={new_euler[1]*180/pi:.1f}°, "
              f"Yaw={new_euler[2]*180/pi:.1f}°")
    
    def reset_to_home(self):
        """Reset target pose to home position."""
        self.target_pose.pose = self.home_pose
        print("Target reset to HOME position")
        self.print_pose()
    
    def print_pose(self):
        """Print current target pose."""
        p = self.target_pose.pose
        euler = self.quaternion_to_euler(p.orientation)
        print("\n--- Current Target Pose ---")
        print(f"Position: X={p.position.x:.3f}, Y={p.position.y:.3f}, Z={p.position.z:.3f}")
        print(f"Orientation (RPY): Roll={euler[0]*180/pi:.1f}°, "
              f"Pitch={euler[1]*180/pi:.1f}°, Yaw={euler[2]*180/pi:.1f}°")
        print(f"Orientation (Quat): x={p.orientation.x:.3f}, y={p.orientation.y:.3f}, "
              f"z={p.orientation.z:.3f}, w={p.orientation.w:.3f}\n")
    
    def send_goal(self):
        """Send motion goal to MoveIt using current target pose."""
        # Wait for joint state
        if self.current_joint_state is None:
            self.get_logger().warn('No joint state received yet. Waiting...')
            timeout = 5.0
            start_time = self.get_clock().now()
            while self.current_joint_state is None and rclpy.ok():
                if (self.get_clock().now() - start_time).nanoseconds / 1e9 > timeout:
                    self.get_logger().error('Timeout waiting for joint states!')
                    return False
                rclpy.spin_once(self, timeout_sec=0.1)
        
        # Create goal message
        goal_msg = MoveGroup.Goal()
        goal_msg.request.workspace_parameters.header.frame_id = 'base_link'
        goal_msg.request.group_name = 'ur_manipulator'
        goal_msg.request.allowed_planning_time = 5.0
        
        # Set start state to current robot state
        goal_msg.request.start_state = RobotState()
        goal_msg.request.start_state.joint_state = self.current_joint_state
        
        # Speed scaling for smooth motion
        goal_msg.request.max_velocity_scaling_factor = 0.2
        goal_msg.request.max_acceleration_scaling_factor = 0.2
        
        # Set target pose constraints
        self.target_pose.header.stamp = self.get_clock().now().to_msg()
        
        # Position constraint
        pc = PositionConstraint()
        pc.header.frame_id = 'base_link'
        pc.link_name = 'tool0'
        pc.target_point_offset.x = 0.0
        pc.target_point_offset.y = 0.0
        pc.target_point_offset.z = 0.0
        
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.001, 0.001, 0.001]  # 1mm tolerance
        pc.constraint_region.primitives.append(primitive)
        pc.constraint_region.primitive_poses.append(self.target_pose.pose)
        pc.weight = 1.0
        
        # Orientation constraint
        oc = OrientationConstraint()
        oc.header.frame_id = 'base_link'
        oc.link_name = 'tool0'
        oc.orientation = self.target_pose.pose.orientation
        oc.absolute_x_axis_tolerance = 0.05
        oc.absolute_y_axis_tolerance = 0.05
        oc.absolute_z_axis_tolerance = 0.05
        oc.weight = 1.0
        
        # Add constraints to goal
        constraints = Constraints()
        constraints.position_constraints.append(pc)
        constraints.orientation_constraints.append(oc)
        goal_msg.request.goal_constraints.append(constraints)
        
        self.get_logger().info('Planning and executing motion...')
        print("\n>>> Sending goal to MoveIt (with IKFast solver)...\n")
        
        # Send goal asynchronously
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
        return True
    
    def goal_response_callback(self, future):
        """Handle goal response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by MoveIt!')
            print(">>> Motion planning FAILED or was rejected.\n")
            return
        
        self.get_logger().info('Goal accepted! Executing...')
        print(">>> Motion accepted, executing...\n")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """Handle execution result."""
        result = future.result().result
        if result.error_code.val == 1:
            self.get_logger().info('SUCCESS: Motion completed!')
            print(">>> Motion COMPLETED successfully!\n")
        else:
            self.get_logger().warn(f'Motion FAILED with error code: {result.error_code.val}')
            print(f">>> Motion FAILED (Error code: {result.error_code.val})\n")
    
    def run(self):
        """Main control loop."""
        try:
            while rclpy.ok():
                key = self.get_key()
                
                if key == '\x1b' or key == 'x':  # ESC or x
                    print("\nExiting keyboard teleop...")
                    break
                
                elif key == 'w':  # Forward (X+)
                    self.update_pose_translation(dx=self.linear_step)
                
                elif key == 's':  # Backward (X-)
                    self.update_pose_translation(dx=-self.linear_step)
                
                elif key == 'a':  # Left (Y+)
                    self.update_pose_translation(dy=self.linear_step)
                
                elif key == 'd':  # Right (Y-)
                    self.update_pose_translation(dy=-self.linear_step)
                
                elif key == 'q':  # Up (Z+)
                    self.update_pose_translation(dz=self.linear_step)
                
                elif key == 'e':  # Down (Z-)
                    self.update_pose_translation(dz=-self.linear_step)
                
                elif key == 'u':  # Roll left (X+)
                    self.update_pose_rotation(droll=self.angular_step)
                
                elif key == 'o':  # Roll right (X-)
                    self.update_pose_rotation(droll=-self.angular_step)
                
                elif key == 'i':  # Pitch up (Y+)
                    self.update_pose_rotation(dpitch=self.angular_step)
                
                elif key == 'k':  # Pitch down (Y-)
                    self.update_pose_rotation(dpitch=-self.angular_step)
                
                elif key == 'j':  # Yaw left (Z+)
                    self.update_pose_rotation(dyaw=self.angular_step)
                
                elif key == 'l':  # Yaw right (Z-)
                    self.update_pose_rotation(dyaw=-self.angular_step)
                
                elif key == 'r':  # Reset to home
                    self.reset_to_home()
                
                elif key == 'p':  # Print pose
                    self.print_pose()
                
                elif key == '+' or key == '=':  # Increase step size
                    self.linear_step *= 1.5
                    self.angular_step *= 1.5
                    print(f"Step size increased: {self.linear_step*1000:.1f}mm / {self.angular_step*180/pi:.1f}°")
                
                elif key == '-' or key == '_':  # Decrease step size
                    self.linear_step /= 1.5
                    self.angular_step /= 1.5
                    print(f"Step size decreased: {self.linear_step*1000:.1f}mm / {self.angular_step*180/pi:.1f}°")
                
                elif key == ' ':  # Execute motion
                    self.send_goal()
                    # Allow some time for motion execution
                    for _ in range(10):
                        rclpy.spin_once(self, timeout_sec=0.1)
                
                # Spin once to process callbacks
                rclpy.spin_once(self, timeout_sec=0.01)
                
        except Exception as e:
            self.get_logger().error(f'Error in control loop: {e}')
        
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = KeyboardTeleop()
        node.run()
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
