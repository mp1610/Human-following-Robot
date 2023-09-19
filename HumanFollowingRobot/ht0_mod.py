import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Twist

class HumanFollowingNode(Node):
    def __init__(self):
        super().__init__('human_following_node')
        self.subscription = self.create_subscription(PoseArray, 'detected_persons', self.pose_array_callback, 10)
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.is_following = False
        self.prev_person_position = None

    def pose_array_callback(self, msg):
        if msg.poses:
            self.prev_person_position = msg.poses[0].position
            self.is_following = True
        else:
            self.is_following = False

    def follow_person(self):
        if self.is_following and self.prev_person_position:
            control_command = Twist()
            
            # Calculate linear velocity based on distance to person
            distance_to_person = self.prev_person_position.x  # Assuming the x-coordinate is the distance
            linear_velocity = min(0.5, distance_to_person)
            control_command.linear.x = linear_velocity
            
            # Calculate angular velocity based on angle to person
            angle_to_person = math.atan2(self.prev_person_position.y, self.prev_person_position.x)
            angular_velocity = angle_to_person * 0.5  # Adjust the angular speed as needed
            control_command.angular.z = angular_velocity
            
            self.velocity_publisher.publish(control_command)

    def main(self):
        rate = self.create_rate(10)
        
        while rclpy.ok():
            self.follow_person()
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


