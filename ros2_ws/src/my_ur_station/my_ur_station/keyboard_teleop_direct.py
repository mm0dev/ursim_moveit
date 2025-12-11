#!/usr/bin/env python3
"""
Real-time Keyboard Teleoperation for UR Robot
Direct joint velocity control for live teleoperation.

Controls:
  W/S - Rotate joint 1 (base)
  A/D - Rotate joint 2 (shoulder)  
  Q/E - Rotate joint 3 (elbow)
  I/K - Rotate joint 4 (wrist 1)
  J/L - Rotate joint 5 (wrist 2)
  U/O - Rotate joint 6 (wrist 3)
  SPACE - Stop all motion
  ESC/X - Exit

Hold keys to move continuously.
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import sys
import select
import termios
import tty
from builtin_interfaces.msg import Duration


class KeyboardTeleopDirect(Node):
    def __init__(self):
        super().__init__('keyboard_teleop_direct')
        
        # Publisher for joint trajectory commands
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10
        )
        
        # Subscribe to joint states to get current position
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
        self.joint_speed = 0.3  # rad/s
        self.time_step = 0.1    # seconds
        
        # Terminal settings
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info('Waiting for joint states...')
        while self.current_joint_positions is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.get_logger().info('Joint states received. Ready!')
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
        print("REAL-TIME KEYBOARD TELEOP - Direct Joint Control")
        print("="*60)
        print("JOINT CONTROL:")
        print("  W/S - Joint 1 (Base rotation)")
        print("  A/D - Joint 2 (Shoulder)")
        print("  Q/E - Joint 3 (Elbow)")
        print("  I/K - Joint 4 (Wrist 1)")
        print("  J/L - Joint 5 (Wrist 2)")
        print("  U/O - Joint 6 (Wrist 3)")
        print("\nCOMMANDS:")
        print("  +/- - Increase/decrease speed")
        print("  SPACE - Stop all motion")
        print("  X/ESC - Exit")
        print("="*60)
        print(f"\nSpeed: {self.joint_speed:.2f} rad/s")
        print("Ready! Hold keys to move in real-time.\n")
    
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
    
    def send_joint_command(self, joint_deltas):
        """Send joint trajectory command with delta movements."""
        if self.current_joint_positions is None:
            return
        
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = [
            self.current_joint_positions[i] + joint_deltas[i]
            for i in range(6)
        ]
        point.time_from_start = Duration(sec=0, nanosec=int(self.time_step * 1e9))
        
        msg.points = [point]
        self.traj_pub.publish(msg)
    
    def run(self):
        """Main control loop."""
        try:
            while rclpy.ok():
                key = self.get_key()
                
                # Initialize deltas to zero
                joint_deltas = [0.0] * 6
                delta = self.joint_speed * self.time_step
                
                if key == '\x1b' or key == 'x':  # ESC or x
                    print("\nExiting keyboard teleop...")
                    break
                
                # Joint 1 (Base)
                elif key == 'w':
                    joint_deltas[0] = delta
                elif key == 's':
                    joint_deltas[0] = -delta
                
                # Joint 2 (Shoulder)
                elif key == 'a':
                    joint_deltas[1] = delta
                elif key == 'd':
                    joint_deltas[1] = -delta
                
                # Joint 3 (Elbow)
                elif key == 'q':
                    joint_deltas[2] = delta
                elif key == 'e':
                    joint_deltas[2] = -delta
                
                # Joint 4 (Wrist 1)
                elif key == 'i':
                    joint_deltas[3] = delta
                elif key == 'k':
                    joint_deltas[3] = -delta
                
                # Joint 5 (Wrist 2)
                elif key == 'j':
                    joint_deltas[4] = delta
                elif key == 'l':
                    joint_deltas[4] = -delta
                
                # Joint 6 (Wrist 3)
                elif key == 'u':
                    joint_deltas[5] = delta
                elif key == 'o':
                    joint_deltas[5] = -delta
                
                # Commands
                elif key == ' ':  # Stop
                    # Send current position (no movement)
                    self.send_joint_command([0.0] * 6)
                    print("Stopped")
                    continue
                    
                elif key == '+' or key == '=':  # Increase speed
                    self.joint_speed *= 1.2
                    print(f"Speed increased: {self.joint_speed:.2f} rad/s")
                    continue
                    
                elif key == '-' or key == '_':  # Decrease speed
                    self.joint_speed /= 1.2
                    print(f"Speed decreased: {self.joint_speed:.2f} rad/s")
                    continue
                
                # Send command if any key was pressed
                if any(d != 0.0 for d in joint_deltas):
                    self.send_joint_command(joint_deltas)
                
                # Spin once to process callbacks
                rclpy.spin_once(self, timeout_sec=0.001)
                    
        except KeyboardInterrupt:
            print("\nInterrupted")
        except Exception as e:
            self.get_logger().error(f'Error in control loop: {e}')
            import traceback
            traceback.print_exc()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = KeyboardTeleopDirect()
        node.run()
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
