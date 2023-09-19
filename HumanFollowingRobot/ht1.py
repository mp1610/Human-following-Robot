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
        self.target_pose = None

    def pose_array_callback(self, msg):
        if msg.poses:  # Check if any person is detected
            self.target_pose = msg.poses[0]  # Follow the first detected person
            self.is_following = True
            print("Following a human.")
            print("Target Pose:", self.target_pose)
        else:
            self.is_following = False
            print("No human detected.")

    def follow_human(self):
        if self.is_following and self.target_pose:
            # Calculate the distance between the robot and the human
            distance = math.sqrt(
                (self.target_pose.position.x ** 2) + (self.target_pose.position.y ** 2)
            )
            print("Distance to human:", distance)
            
            # Calculate the desired linear velocity based on the distance
            desired_linear_velocity = max(0.0, min(0.5, distance - 0.5))
            print("Desired linear velocity:", desired_linear_velocity)
            
            # Calculate the desired angular velocity based on the target's position
            angle_to_human = math.atan2(self.target_pose.position.y, self.target_pose.position.x)
            desired_angular_velocity = angle_to_human * 0.5  # Adjust the scaling factor as needed
            print("Desired angular velocity:", desired_angular_velocity)
            
            # Adjust the robot's linear velocity based on the angle to the human
            if abs(angle_to_human) > math.pi / 4:  # If the human is at a significant angle
                desired_linear_velocity = 0.1  # Slow down the robot's forward movement
                print("Adjusting linear velocity due to angle:", desired_linear_velocity)
            
            # Publish control commands
            control_command = Twist()
            control_command.linear.x = desired_linear_velocity
            control_command.angular.z = desired_angular_velocity
            self.velocity_publisher.publish(control_command)
            print("Published control commands.")

    def main(self):
        rate = self.create_rate(10)  # Set the publishing rate
        
        while rclpy.ok():
            self.follow_human()  # Implement the human-following algorithm
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







