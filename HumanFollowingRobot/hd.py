import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge
import cv2
import torch
from mmdet.apis import init_detector, inference_detector

class HumanDetectionNode(Node):
    def __init__(self):
        super().__init__('human_detection_node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.pose_array_publisher = self.create_publisher(PoseArray, 'detected_persons', 10)  # Create PoseArray publisher
        self.model = init_detector('../mmdetection/configs/yolo/yolov3.py', '../mmdetection/work_dirs/yolo300/epoch_150.pth', device='cuda:0')

    def image_callback(self, msg):
        print("Received a new image!")
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        result = self.detect_persons(cv_image)
        pose_array = self.create_pose_array(result)
        self.publish_pose_array(pose_array)
        
        # Display the image with detected bounding boxes
        image_with_boxes = self.draw_boxes(cv_image, result)
        cv2.imshow("Image with Detected Persons", image_with_boxes)
        cv2.waitKey(1)  # Wait for a short time to update the window
        
        # Print the generated PoseArray
        print("Generated PoseArray:")
        print(pose_array)
        
    def draw_boxes(self, image, persons):
        image_with_boxes = image.copy()
        
        for person in persons:
            _, _, x1, y1, x2, y2 = person
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            cv2.rectangle(image_with_boxes, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Display the label inside the box
            font = cv2.FONT_HERSHEY_SIMPLEX
            label_text = 'Person'
            label_size = cv2.getTextSize(label_text, font, 0.5, 1)[0]
            cv2.putText(image_with_boxes, label_text, (x1, y1 - label_size[1]), font, 0.5, (0, 255, 0), 1)
            
        return image_with_boxes
    

    def detect_persons(self, image):
        result = inference_detector(self.model, image)
    
        pred_instances = result.pred_instances
        
        persons = []  # Initialize an empty list to store detected persons
    
        # Loop through each predicted instance and extract relevant information
        for i in range(len(pred_instances)):
            label = pred_instances.labels[i].item()  # Get the label (class index)
            score = pred_instances.scores[i].item()  # Get the confidence score
            bbox = pred_instances.bboxes[i].cpu().numpy()  # Convert the bounding box to a NumPy array
        
            # Append the extracted information as a tuple to the persons list
            persons.append((label, score, *bbox))
    
        return persons

    def create_pose_array(self, persons):
        pose_array = PoseArray()
        pose_array.header.frame_id = 'camera_frame'  # Update the frame ID as needed

        for person in persons:
            label, score, x1, y1, x2, y2 = person
            pose = Pose()
            pose.position.x = (x1 + x2) / 2
            pose.position.y = (y1 + y2) / 2
            pose.position.z = 0.0  # Assuming a 2D plane
            pose.orientation.w = 1.0  # Default orientation
            pose_array.poses.append(pose)

        return pose_array

    def publish_pose_array(self, pose_array):
        self.pose_array_publisher.publish(pose_array)  # Publish the PoseArray message

def main():
    rclpy.init()
    node = HumanDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
