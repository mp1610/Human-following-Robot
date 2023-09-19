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
        try:
            if msg.poses:  # Check if any person is detected
                self.target_pose = msg.poses[0]  # Follow the first detected person
                self.is_following = True
                self.get_logger().info("Following a human.")
                self.get_logger().info(f"Target Pose: {self.target_pose}")
                
                # Print the list of detected poses
                for i, pose in enumerate(msg.poses):
                    print(f"Detected Person {i + 1} - Position: ({pose.position.x:.2f}, {pose.position.y:.2f}), Orientation: ({pose.orientation.x:.2f}, {pose.orientation.y:.2f}, {pose.orientation.z:.2f}, {pose.orientation.w:.2f})")
                
            else:
                self.is_following = False
                self.get_logger().info("No human detected.")
        except Exception as e:
            self.get_logger().error(f"Error in pose_array_callback: {e}")

    def follow_human(self):
        try:
            if self.is_following and self.target_pose:
                # Calculate the distance between the robot and the human
                distance = math.sqrt(
                    (self.target_pose.position.x ** 2) + (self.target_pose.position.y ** 2)
                )

                # Calculate the desired linear velocity based on the distance
                desired_linear_velocity = max(0.0, min(0.5, distance - 0.5))

                # Calculate the desired angular velocity based on the target's position
                angle_to_human = math.atan2(self.target_pose.position.y, self.target_pose.position.x)
                desired_angular_velocity = angle_to_human * 0.75
                
                # Check and adjust robot's linear velocity if too close to human
                if distance < 0.2:
                    desired_linear_velocity = 0.0

                # Publish control commands
                control_command = Twist()
                control_command.linear.x = desired_linear_velocity
                control_command.angular.z = desired_angular_velocity
                self.velocity_publisher.publish(control_command)
        except Exception as e:
            self.get_logger().error(f"Error in follow_human: {e}")

    def main(self):
        rate = self.create_rate(10)  # Set the publishing rate
        
        while rclpy.ok():
            try:
                self.follow_human()  # Implement the human-following algorithm
                rate.sleep()
            except Exception as e:
                self.get_logger().error(f"Error in main loop: {e}")

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


