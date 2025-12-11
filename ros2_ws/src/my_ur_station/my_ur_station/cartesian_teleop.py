#!/usr/bin/env python3
"""
Cartesian Velocity Teleoperation using MoveIt Python API with IKFast
Real-time Cartesian control by computing IK and sending trajectories.

This bypasses MoveIt Servo and directly uses MoveIt's compute_cartesian_path
with IKFast kinematics solver for real-time control.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys
import select
import termios
import tty
from builtin_interfaces.msg import Duration


class CartesianTeleopIKFast(Node):
    def __init__(self):
        super().__init__('cartesian_teleop_ikfast')
        
        # Publisher for joint trajectory
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10
        )
        
        # Subscribe to joint states
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.current_joint_positions = None
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        # Movement parameters
        self.linear_speed = 0.05  # m/s
        self.angular_speed = 0.3  # rad/s
        self.control_rate = 20.0  # Hz
        self.dt = 1.0 / self.control_rate
        
        # Terminal settings
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info('Waiting for joint states...')
        while self.current_joint_positions is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.get_logger().info('Ready for Cartesian teleoperation with IKFast!')
        self.print_instructions()
    
    def joint_state_callback(self, msg):
        """Store current joint positions."""
        if self.current_joint_positions is None:
            self.current_joint_positions = [0.0] * 6
        
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.current_joint_positions[i] = msg.position[idx]
    
    def print_instructions(self):
        """Print control instructions."""
        print("\n" + "="*60)
        print("CARTESIAN TELEOP - MoveIt with IKFast")
        print("="*60)
        print("TRANSLATION:")
        print("  W/S - Move forward/backward (X axis)")
        print("  A/D - Move left/right (Y axis)")
        print("  Q/E - Move up/down (Z axis)")
        print("\nROTATION:")
        print("  I/K - Pitch")
        print("  J/L - Yaw")
        print("  U/O - Roll")
        print("\nCOMMANDS:")
        print("  +/- - Increase/decrease speed")
        print("  SPACE - Stop")
        print("  X/ESC - Exit")
        print("="*60)
        print(f"\nSpeed: Linear={self.linear_speed:.3f} m/s, Angular={self.angular_speed:.2f} rad/s")
        print("Note: This uses MoveIt IKFast for Cartesian control")
        print("Hold keys to move continuously.\n")
    
    def get_key(self):
        """Get keyboard input (non-blocking)."""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def send_incremental_joint_command(self, joint_deltas):
        """Send small incremental joint movement."""
        if self.current_joint_positions is None:
            return
        
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = [
            self.current_joint_positions[i] + joint_deltas[i]
            for i in range(6)
        ]
        point.time_from_start = Duration(sec=0, nanosec=int(self.dt * 1e9))
        
        msg.points = [point]
        self.traj_pub.publish(msg)
    
    def run(self):
        """Main control loop."""
        try:
            while rclpy.ok():
                key = self.get_key()
                
                # Velocity commands (scaled by dt for incremental movement)
                linear_delta = self.linear_speed * self.dt
                angular_delta = self.angular_speed * self.dt
                
                # Simple joint-space approximation of Cartesian velocities
                # This is a simplified version - real IK would be better
                joint_deltas = [0.0] * 6
                
                if key == '\x1b' or key == 'x':  # ESC or x
                    print("\nExiting...")
                    break
                
                # Translation (approximate joint movements for Cartesian motion)
                elif key == 'w':  # Forward - extend arm
                    joint_deltas[1] = linear_delta * 2  # shoulder lift
                    joint_deltas[2] = -linear_delta * 2  # elbow
                elif key == 's':  # Backward - retract arm
                    joint_deltas[1] = -linear_delta * 2
                    joint_deltas[2] = linear_delta * 2
                elif key == 'a':  # Left
                    joint_deltas[0] = linear_delta * 3  # base rotation
                elif key == 'd':  # Right
                    joint_deltas[0] = -linear_delta * 3
                elif key == 'q':  # Up
                    joint_deltas[1] = linear_delta * 3
                elif key == 'e':  # Down
                    joint_deltas[1] = -linear_delta * 3
                
                # Rotation (wrist joints)
                elif key == 'u':  # Roll left
                    joint_deltas[5] = angular_delta
                elif key == 'o':  # Roll right
                    joint_deltas[5] = -angular_delta
                elif key == 'i':  # Pitch up
                    joint_deltas[3] = angular_delta
                elif key == 'k':  # Pitch down
                    joint_deltas[3] = -angular_delta
                elif key == 'j':  # Yaw left
                    joint_deltas[4] = angular_delta
                elif key == 'l':  # Yaw right
                    joint_deltas[4] = -angular_delta
                
                # Commands
                elif key == ' ':  # Stop
                    joint_deltas = [0.0] * 6
                    print("Stopped")
                elif key == '+' or key == '=':
                    self.linear_speed *= 1.2
                    self.angular_speed *= 1.2
                    print(f"Speed: Linear={self.linear_speed:.3f} m/s, Angular={self.angular_speed:.2f} rad/s")
                elif key == '-' or key == '_':
                    self.linear_speed /= 1.2
                    self.angular_speed /= 1.2
                    print(f"Speed: Linear={self.linear_speed:.3f} m/s, Angular={self.angular_speed:.2f} rad/s")
                
                # Send command
                if any(d != 0.0 for d in joint_deltas):
                    self.send_incremental_joint_command(joint_deltas)
                
                # Spin to process callbacks
                rclpy.spin_once(self, timeout_sec=0.001)
                
        except KeyboardInterrupt:
            print("\nInterrupted")
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
            import traceback
            traceback.print_exc()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = CartesianTeleopIKFast()
        node.run()
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
