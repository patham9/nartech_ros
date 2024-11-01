import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import threading
import time
import cv2

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        
        # Define QoS to keep only the latest image
        qos_profile = QoSProfile(depth=1)
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE
        
        # Initialize the CV bridge
        self.bridge = CvBridge()
        
        # Load YOLOv4 model
        self.net = cv2.dnn.readNet('yolov4-tiny.weights', 'yolov4-tiny.cfg')  # Adjust paths as needed
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        
        # Create a subscriber to the RGBD camera image topic
        self.create_subscription(Image, '/rgbd_camera/image', self.image_callback, qos_profile)
        
        # Create a publisher for the processed images
        self.image_pub = self.create_publisher(Image, '/yolo_output/image', qos_profile)
        self.processing = False
        self.lock = threading.Lock()

    def image_callback(self, msg):
        with self.lock:
            if self.processing:
                return
            self.processing = True
        start_time = time.time()
        print("NEW PROCESSING")
        self.processing = True
        # Convert ROS Image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Load class names (assuming you have a 'coco.names' file)
        with open('coco.names', 'r') as f:
            classes = [line.strip() for line in f.readlines()]

        # Apply YOLOv4
        height, width, _ = cv_image.shape
        blob = cv2.dnn.blobFromImage(cv_image, 0.00392, (608, 608), (0, 0, 0), True, crop=False)
        self.net.setInput(blob)
        layer_names = self.net.getLayerNames()
        
        # Get the indices of the output layers
        layer_indices = self.net.getUnconnectedOutLayers()
        output_layers = [layer_names[i - 1] for i in layer_indices]  # Adjusted indexing

        detections = self.net.forward(output_layers)

        # Process detections (you can customize this part)
        for out in detections:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.1:  # Adjust confidence threshold as needed
                    print("DET", detection)
                    # Bounding box coordinates
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)
                    # Draw bounding box (optional)
                    cv2.rectangle(cv_image, (center_x - w // 2, center_y - h // 2), 
                                  (center_x + w // 2, center_y + h // 2), (0, 255, 0), 2)
                    
                         # Prepare label with class name and confidence
                    label = f"{classes[class_id]}: {confidence:.2f}"
                    # Get the size of the label to position it
                    (w, h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                    # Draw a background rectangle for the label
                    #cv2.rectangle(cv_image, (center_x - w // 2, center_y - h // 2 - 20), 
                    #              (center_x + w // 2, center_y - h // 2), (0, 255, 0), cv2.FILLED)
                    # Put the label on the image
                    cv2.putText(cv_image, label, 
                                (center_x - w // 2, center_y - h // 2 - 5), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # Convert the processed image back to ROS format and publish
        output_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        self.image_pub.publish(output_msg)
        elapsed_time = time.time() - start_time
        print(f"DONE PROCESSING: {elapsed_time:.2f} seconds")
        with self.lock:
            self.processing = False

def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
