import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Twist
import math

class HumanFollowingNode(Node):
    def __init__(self):
        super().__init__('human_following_node')
        self.subscription = self.create_subscription(PoseArray, 'detected_persons', self.pose_array_callback, 10)
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)  # Publish control commands
        self.is_following = False  # Flag to indicate whether the robot is following a human

    def pose_array_callback(self, msg):
        if msg.poses:  # Check if any person is detected
            self.is_following = True
            print("Following a human.")
        else:
            self.is_following = False
            print("No human detected.")

    def test_linear_velocity(self):
        if self.is_following:
            # Calculate desired linear velocity (replace this with your logic)
            desired_linear_velocity = 0.3  # Change this value as needed
            
            # Simulate publishing control commands
            control_command = Twist()
            control_command.linear.x = desired_linear_velocity
            self.velocity_publisher.publish(control_command)
            print("Published control commands.")

def main():
    rclpy.init()
    node = HumanFollowingNode()

    try:
        while rclpy.ok():
            node.test_linear_velocity()
            rclpy.spin_once(node, timeout_sec=1.0)  # Allow time for publishing and processing
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

