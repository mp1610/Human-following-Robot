import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Twist
from std_msgs.msg import Bool
import math

class HumanFollowingNode(Node):
    def __init__(self):
        super().__init__('human_following_node')
        self.subscription = self.create_subscription(PoseArray, 'detected_persons', self.pose_array_callback, 10)
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)  # Publish control commands
        self.is_following = False  # Flag to indicate whether the robot is following a human
        self.target_pose = None

    def pose_array_callback(self, msg):
        if msg.poses:  # Check if any person is detected
            self.target_pose = msg.poses[0]  # Follow the first detected person
            self.is_following = True
        else:
            self.is_following = False

    def follow_human(self, human_pose):
        # Define constants for desired behaviors
        FAST_LINEAR_SPEED = 0.9  # Adjust this value based on desired speed when not too close
        SAFE_DISTANCE = 0.5  # Adjust this value based on desired safe following distance
        ANGULAR_SPEED_GAIN = 0.75  # Adjust this value to control angular velocity adjustment
        
        robot_pose = self.get_robot_pose()  # Get the robot's current pose
        distance = self.calculate_distance(robot_pose, human_pose)

        if distance > SAFE_DISTANCE:
            # Calculate desired linear velocity
            desired_linear_velocity = FAST_LINEAR_SPEED

            # Calculate desired angular velocity
            angle_to_human = math.atan2(human_pose.position.y - robot_pose.position.y, human_pose.position.x - robot_pose.position.x)
            desired_angular_velocity = angle_to_human * ANGULAR_SPEED_GAIN

        else:
            # Robot is too close, stop
            desired_linear_velocity = 0.0
            desired_angular_velocity = 0.0

        # Publish control commands
        self.publish_control_commands(desired_linear_velocity, desired_angular_velocity)

    def main(self):
        rate = self.create_rate(10)  # Set the publishing rate
        
        while rclpy.ok():
            if self.is_following and self.target_pose:
                self.follow_human(self.target_pose)  # Provide the target_pose to the function
            rate.sleep()

def main():
    rclpy.init()
    node = HumanFollowingNode()
    try:
        node.main()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
