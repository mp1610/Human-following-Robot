import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Twist

class SimpleHumanFollowingNode(Node):
    def __init__(self):
        super().__init__('simple_human_following_node')
        self.subscription = self.create_subscription(PoseArray, 'detected_persons', self.pose_array_callback, 10)
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)  # Publish control commands

    def pose_array_callback(self, msg):
        if msg.poses:
            # Get the position of the first detected person
            target_position = msg.poses[0].position

            # Calculate linear velocity
            linear_velocity = 0.5  # Fixed linear velocity

            # Calculate angular velocity to keep the target in front
            angular_velocity = 0.0  # No angular velocity for a simple following algorithm
            
            # Publish control commands
            control_command = Twist()
            control_command.linear.x = linear_velocity
            control_command.angular.z = angular_velocity
            self.velocity_publisher.publish(control_command)

def main():
    rclpy.init()
    node = SimpleHumanFollowingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
