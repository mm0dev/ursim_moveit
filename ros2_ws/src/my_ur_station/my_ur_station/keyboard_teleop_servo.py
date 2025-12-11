#!/usr/bin/env python3
"""
Simple Up/Down Keyboard Control for MoveIt Servo
Only Q/E keys for up/down movement.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import Trigger
import sys
import os


class KeyboardTeleopServo(Node):
    def __init__(self):
        super().__init__('keyboard_teleop_servo')
        
        # Publisher for servo twist commands
        self.twist_pub = self.create_publisher(
            TwistStamped, 
            '/servo_node/delta_twist_cmds', 
            10
        )
        
        # Service clients for starting/stopping servo
        self.start_servo_client = self.create_client(Trigger, '/servo_node/start_servo')
        
        # Movement speed
        self.linear_speed = 0.1   # m/s for Z axis
        
        # Current velocity command
        self.current_twist = TwistStamped()
        self.current_twist.header.frame_id = 'base_link'
        
        self.get_logger().info('Keyboard Teleop Servo initialized')
        
        # Start servo
        self.start_servo()
        
        print("\n" + "="*60)
        print("UP/DOWN CONTROL - MoveIt Servo")
        print("="*60)
        print("  Q - Move UP (Z+)")
        print("  E - Move DOWN (Z-)")
        print("  X - Exit")
        print("="*60)
        print("Type Q, E, or X then press ENTER\n")
        
    def start_servo(self):
        """Start MoveIt Servo."""
        self.get_logger().info('Waiting for servo service...')
        if not self.start_servo_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('Servo service not available')
            return
            
        req = Trigger.Request()
        future = self.start_servo_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result() is not None:
            self.get_logger().info(f'Servo started: {future.result().message}')
    
    def send_twist(self, z_velocity):
        """Publish twist command."""
        self.current_twist.twist.linear.z = z_velocity
        self.current_twist.header.stamp = self.get_clock().now().to_msg()
        self.twist_pub.publish(self.current_twist)
    
    def run(self):
        """Main control loop using simple input()."""
        try:
            while rclpy.ok():
                # Use simple input() which works in all terminals
                try:
                    key = input("Command (q/e/x): ").lower().strip()
                except EOFError:
                    break
                
                if key == 'x':
                    print("Exiting...")
                    break
                
                elif key == 'q':
                    print("▲ Moving UP...")
                    # Send velocity for 1 second
                    for _ in range(10):
                        self.send_twist(self.linear_speed)
                        rclpy.spin_once(self, timeout_sec=0.1)
                    self.send_twist(0.0)  # Stop
                    print("Stopped")
                    
                elif key == 'e':
                    print("▼ Moving DOWN...")
                    # Send velocity for 1 second
                    for _ in range(10):
                        self.send_twist(-self.linear_speed)
                        rclpy.spin_once(self, timeout_sec=0.1)
                    self.send_twist(0.0)  # Stop
                    print("Stopped")
                else:
                    print(f"Unknown command: {key}")
                    
        except KeyboardInterrupt:
            print("\nInterrupted")
        finally:
            # Send zero velocity
            self.send_twist(0.0)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = KeyboardTeleopServo()
        node.run()
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
