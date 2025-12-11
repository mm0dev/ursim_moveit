#!/usr/bin/env python3
"""
Simple command-line tool to send twist commands to MoveIt Servo.
Properly updates timestamps for each message.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import sys


class ServoTwistCommander(Node):
    def __init__(self):
        super().__init__('servo_twist_commander')
        
        self.twist_pub = self.create_publisher(
            TwistStamped,
            '/servo_node/delta_twist_cmds',
            10
        )
        
        # Timer to publish at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_twist)
        
        # Twist message
        self.twist_msg = TwistStamped()
        self.twist_msg.header.frame_id = 'base_link'
        
    def set_twist(self, linear_x=0.0, linear_y=0.0, linear_z=0.0, 
                  angular_x=0.0, angular_y=0.0, angular_z=0.0):
        """Set the twist values."""
        self.twist_msg.twist.linear.x = linear_x
        self.twist_msg.twist.linear.y = linear_y
        self.twist_msg.twist.linear.z = linear_z
        self.twist_msg.twist.angular.x = angular_x
        self.twist_msg.twist.angular.y = angular_y
        self.twist_msg.twist.angular.z = angular_z
        
    def publish_twist(self):
        """Publish twist with updated timestamp."""
        self.twist_msg.header.stamp = self.get_clock().now().to_msg()
        self.twist_pub.publish(self.twist_msg)


def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) < 2:
        print("Usage:")
        print("  ros2 run my_ur_station servo_twist_cmd up      # Move up (Z+)")
        print("  ros2 run my_ur_station servo_twist_cmd down    # Move down (Z-)")
        print("  ros2 run my_ur_station servo_twist_cmd stop    # Stop movement")
        print("  ros2 run my_ur_station servo_twist_cmd <lx> <ly> <lz> <ax> <ay> <az>  # Custom twist")
        print("\nExamples:")
        print("  ros2 run my_ur_station servo_twist_cmd up")
        print("  ros2 run my_ur_station servo_twist_cmd 0 0 0.1 0 0 0")
        print("\nPress Ctrl+C to stop.")
        sys.exit(1)
    
    node = ServoTwistCommander()
    
    command = sys.argv[1].lower()
    
    if command == "up":
        node.set_twist(linear_z=0.1)
        print("Moving UP (Z+) at 0.1 m/s")
    elif command == "down":
        node.set_twist(linear_z=-0.1)
        print("Moving DOWN (Z-) at 0.1 m/s")
    elif command == "stop":
        node.set_twist()
        print("Sending STOP command")
    else:
        # Custom twist values
        try:
            if len(sys.argv) == 7:
                lx, ly, lz, ax, ay, az = map(float, sys.argv[1:7])
                node.set_twist(lx, ly, lz, ax, ay, az)
                print(f"Sending custom twist: linear=({lx}, {ly}, {lz}), angular=({ax}, {ay}, {az})")
            else:
                print("Error: Expected 6 values for custom twist (lx ly lz ax ay az)")
                sys.exit(1)
        except ValueError:
            print("Error: All twist values must be numbers")
            sys.exit(1)
    
    print("Publishing at 10 Hz. Press Ctrl+C to stop.")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        # Send zero velocity before shutting down
        node.set_twist()
        for _ in range(5):
            node.publish_twist()
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
