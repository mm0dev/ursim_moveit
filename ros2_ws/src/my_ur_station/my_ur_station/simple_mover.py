import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class URMover(Node):
    def __init__(self):
        super().__init__('ur_mover')
        self.publisher_ = self.create_publisher(JointTrajectory, '/scaled_joint_trajectory_controller/joint_trajectory', 10)
        self.timer = self.create_timer(4.0, self.move_callback)
        self.position_toggle = False
        self.get_logger().info('UR Mover Node Started!')

    def move_callback(self):
        msg = JointTrajectory()
        msg.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        point = JointTrajectoryPoint()
        
        if self.position_toggle:
            point.positions = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0] # Pose A
            self.get_logger().info('Moving to Pose A (Upright)...')
        else:
            point.positions = [0.5, -1.0, 1.0, -1.57, 0.0, 0.0] # Pose B
            self.get_logger().info('Moving to Pose B (Forward)...')
            
        point.time_from_start.sec = 2
        msg.points = [point]
        self.publisher_.publish(msg)
        self.position_toggle = not self.position_toggle

def main(args=None):
    rclpy.init(args=args)
    node = URMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()