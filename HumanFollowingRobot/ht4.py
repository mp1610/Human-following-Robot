import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Twist
import math

class SimpleHumanFollowingNode(Node):
    def __init__(self):
        super().__init__('simple_human_following_node')
        self.subscription = self.create_subscription(PoseArray, 'detected_persons', self.pose_array_callback, 10)
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)  # Publish control commands
        self.is_following = False
        self.target_pose = None
        self.desired_distance = 0.5  # Desired following distance

    def pose_array_callback(self, msg):
        if msg.poses:
            self.target_pose = msg.poses[0]  # Follow the first detected person
            self.is_following = True
        else:
            self.is_following = False

    def follow_human(self):
        if self.is_following and self.target_pose:
            # Calculate linear velocity based on distance to human
            distance = math.sqrt(self.target_pose.position.x ** 2 + self.target_pose.position.y ** 2)
            linear_velocity = max(0.0, min(0.5, distance - self.desired_distance))  # Limit velocity to 0.5
            
            # Calculate angular velocity to maintain angle to human
            angle_to_human = math.atan2(self.target_pose.position.y, self.target_pose.position.x)
            angular_velocity = angle_to_human * 0.75
            
            # Publish control commands
            control_command = Twist()
            control_command.linear.x = linear_velocity
            control_command.angular.z = angular_velocity
            self.velocity_publisher.publish(control_command)

    def main(self):
        rate = self.create_rate(10)  # Set the publishing rate
        
        while rclpy.ok():
            self.follow_human()
            rate.sleep()

def main():
    rclpy.init()
    node = SimpleHumanFollowingNode()
    try:
        node.main()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

