#!/usr/bin/env python3
import rclpy
import time
import numpy as np
import math
import cv2
import threading
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import TransformStamped, PoseStamped
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import tf2_geometry_msgs
import tf2_ros
from tf2_ros.buffer import Buffer
from geometry_msgs.msg import PointStamped, Point
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from std_msgs.msg import Header
from rclpy.duration import Duration
from rclpy.time import Time
from geometry_msgs.msg import Quaternion

class LowResGridMapPublisher(Node):
    def __init__(self):
        super().__init__('lowres_gridmap_publisher')
        self.M = {"wall": 100, "robot": 127, "chair": -126, "table": -126, "bottle": -125, "cup": -125, "can": -125, "person": -124}
        self.previous_detections_persistence = 100.0 #how long to persist it in seconds
        self.previous_detections = dict([]) #where objects of type has been seen last time
        # Define QoS to keep only the latest image for string commands:
        qos_profile_str = QoSProfile(depth=1)
        qos_profile_str.history = QoSHistoryPolicy.KEEP_LAST
        qos_profile_str.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos_profile_str.reliability = QoSReliabilityPolicy.RELIABLE
        # Define QoS to keep map
        qos_profile_map = QoSProfile(depth=1)
        qos_profile_map.history = QoSHistoryPolicy.KEEP_LAST
        qos_profile_map.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        # Define QoS to keep only the latest image
        qos_profile_yolo = QoSProfile(depth=1)
        qos_profile_yolo.history = QoSHistoryPolicy.KEEP_LAST
        qos_profile_yolo.durability = QoSDurabilityPolicy.VOLATILE
        # Parameters for resolution reduction
        self.downsample_factor = 28  # Change as needed
        # TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        # Subscribing to the occupancy grid map
        self.occ_grid_sub = self.create_subscription(OccupancyGrid, '/map', self.occ_grid_callback, qos_profile_map)
        # Publisher for the low-resolution grid map
        self.lowres_grid_pub = self.create_publisher(OccupancyGrid, '/lowres_map', qos_profile_map)
        # Subscriber to listen for movement commands
        self.naceop_sub = self.create_subscription(String, '/naceop', self.naceop_callback, qos_profile_str)
        # Publisher to indicate movement completion
        self.nacedone_pub = self.create_publisher(String, '/nacedone', qos_profile_str)
        # Initialize Action Client
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.navigation_goal = None
        self.navigation_retries = 0
        self.goal_handle = None
        # value that are set later after messages are received
        self.origin = None
        self.robot_lowres_x = None
        self.robot_lowres_y = None
        self.low_res_grid = None
        self.mapupdate = 0
        self.goalstart = 0
        self.new_width = 0
        self.cached_msg = None
        #YOLO:
        # Initialize CV bridge and other parameters
        self.bridge = CvBridge()
        self.processing = False
        self.detections = None
        self.lock = threading.Lock()
        # Subscribe to RGB camera images
        self.create_subscription(Image, '/rgbd_camera/image', self.image_callback, qos_profile_yolo)
        # Subscribe to depth images
        self.create_subscription(Image, '/rgbd_camera/depth_image', self.depth_callback, qos_profile_yolo)
        # Load YOLO model
        self.net = cv2.dnn.readNet('yolov4-tiny.weights', 'yolov4-tiny.cfg')
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        self.image_pub = self.create_publisher(Image, '/yolo_output/image', qos_profile_yolo)
        self.image_width = 320
        self.image_height = 240
        self.horizontal_fov = 1.25  # in radians
        # Calculate the focal lengths
        self.fx = self.image_width / (2 * np.tan(self.horizontal_fov / 2))
        self.fy = self.image_height / (2 * np.tan(self.horizontal_fov / 2))
        self.depth_image = None
        # Load class names (assuming you have a 'coco.names' file)
        with open('coco.names', 'r') as f:
            self.classes = [line.strip() for line in f.readlines()]
        # Create a periodic timer that calls the processing function every 1 second
        self.timer = self.create_timer(2.0, self.build_grid_periodic)

    # Helper function to compute low-res grid coordinates
    def get_lowres_position(self, world_x, world_y, original_origin, original_resolution):
        # Calculate low-resolution grid coordinates
        grid_x = int((world_x - original_origin.position.x) / original_resolution)
        grid_y = int((world_y - original_origin.position.y) / original_resolution)
        return grid_x, grid_y

    def occ_grid_callback(self, msg):
        self.get_logger().info("NEW OCC GRID")
        self.cached_msg = msg
        self.build_grid_periodic()

    def build_grid_periodic(self):
        if self.navigation_goal is not None:
            target_cell, _ = self.navigation_goal
            if self.check_collision(target_cell):
                if self.goal_handle is not None:
                    self.goal_handle.cancel_goal_async()
                    self.get_logger().info('Cancellation of goal due collision.')
        if self.cached_msg is None:
            self.get_logger().info("WAITING FOR OCC GRID")
            return
        msg = self.cached_msg
        start_time = time.time()
        self.get_logger().info("NEW PROCESSING: OCC GRID")
        # Get original grid map data
        original_width = msg.info.width
        original_height = msg.info.height
        original_resolution = msg.info.resolution
        original_origin = msg.info.origin
        original_data = msg.data
        # Compute new width and height
        self.new_width = original_width // self.downsample_factor
        self.new_height = original_height // self.downsample_factor
        self.new_resolution = original_resolution * self.downsample_factor
        # If original dimensions are not evenly divisible, adjust the boundaries
        if original_width % self.downsample_factor != 0:
            self.new_width += 1
        if original_height % self.downsample_factor != 0:
            self.new_height += 1
        # Initialize new grid map data with zeros (free)
        self.low_res_grid = [0] * (self.new_width * self.new_height)
        # Mark occupied cells based on downsampling
        for y in range(0, original_height, self.downsample_factor):
            for x in range(0, original_width, self.downsample_factor):
                new_x = x // self.downsample_factor
                new_y = y // self.downsample_factor
                new_idx = new_y * self.new_width + new_x
                cell_value = self.get_block_occupancy(original_data, x, y, original_width, self.downsample_factor)
                self.low_res_grid[new_idx] = cell_value
        # Get robot's current position
        self.robot_lowres_x, self.robot_lowres_y = self.get_robot_lowres_position(original_origin, original_resolution)
        # Ensure indices are within bounds
        if 0 <= self.robot_lowres_x < self.new_width and 0 <= self.robot_lowres_y < self.new_height:
            robot_idx = self.robot_lowres_y * self.new_width + self.robot_lowres_x
            self.low_res_grid[robot_idx] = 127  # Mark as occupied
            self.get_logger().info(f"Marked robot position at ({self.robot_lowres_x}, {self.robot_lowres_y}) as occupied.")
        else:
            self.get_logger().warn("Robot position is out of bounds in the downsampled map.")
        if not self.detections is None and self.depth_image is None:
            self.get_logger().warn("Got detections but no depth image")
        if not self.depth_image is None and self.detections is None:
            self.get_logger().warn("Got depth image but no detections")
        if self.depth_image is None and self.detections is None:
            self.get_logger().warn("Got no depth image and no detections")
        # Process detections
        if self.detections:
            for out in self.detections:
                for detection in out:
                    scores = detection[5:]
                    class_id = np.argmax(scores)
                    confidence = scores[class_id]
                    if confidence > 0.5:  # Confidence threshold
                        # Compute detection center in pixel coordinates
                        center_x = int(detection[0] * self.width)
                        center_y = int(detection[1] * self.height)
                        # Get depth value at the detection center
                        depth_value = self.depth_image[center_y, center_x]
                        category = self.classes[class_id]
                        if depth_value > 0 and category in self.M:  # Valid depth
                            # Use the timestamp from the synchronized images
                            stamp = self.last_image_stamp if self.last_image_stamp else self.get_clock().now()
                            # Create a point in camera space with the correct timestamp
                            self.get_logger().info(f"DEPTH DEBUG" + str(depth_value))
                            camera_point = PointStamped(
                                header=Header(stamp=stamp.to_msg(), frame_id='oakd_left_camera_frame'),
                                point = Point(
                                    x=depth_value,  # The depth value determines how far forward the point is
                                    y=-(center_x - (self.width / 2)) * depth_value / self.fx,  # Horizontal offset scaled by depth
                                    z=-(center_y - (self.height / 2)) * depth_value / self.fy   # Vertical offset scaled by depth
                                )
                            )
                            # Transform the point to the robot's frame
                            try:
                                transformed_point = self.tf_buffer.transform(camera_point, 'map', timeout=Duration(seconds=1.0))
                                # Map the transformed point to the grid coordinates
                                object_grid_x, object_grid_y = self.get_lowres_position(
                                    transformed_point.point.x, transformed_point.point.y,
                                    original_origin, self.new_resolution  # Use new_resolution for low-res grid
                                )
                                # Mark the detection in the low-resolution grid
                                if 0 <= object_grid_x < self.new_width and 0 <= object_grid_y < self.new_height:
                                    obj_idx = object_grid_y * self.new_width + object_grid_x
                                    self.previous_detections[category] = (obj_idx, time.time()) #persist last seen as object
                                    self.low_res_grid[obj_idx] = self.M[category]  # Mark as detected object
                                    self.get_logger().info(f"Marked detected object at ({object_grid_x}, {object_grid_y}) in grid.")
                                else:
                                    self.get_logger().warn("Detected object position is out of bounds in the downsampled map.")
                            except tf2_ros.LookupException:
                                self.get_logger().error("Transform not found.")
                            except tf2_ros.ConnectivityException:
                                self.get_logger().error("Connectivity exception while looking up transform.")
                            except tf2_ros.ExtrapolationException:
                                self.get_logger().error("Extrapolation exception while looking up transform.")
        for category in self.previous_detections:
            (obj_idx, t) = self.previous_detections[category]
            if time.time() - t < self.previous_detections_persistence:
                self.low_res_grid[obj_idx] = self.M[category]
        # Publish the low-resolution map
        self.publish_low_res_map(msg)
        elapsed_time = time.time() - start_time
        self.get_logger().info(f"DONE map: {elapsed_time:.2f} seconds")

    def depth_callback(self, msg):
        self.get_logger().info("Depth image received")
        with self.lock:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')  # Depth image
            self.last_depth_stamp = Time.from_msg(msg.header.stamp)

    def image_callback(self, msg):
        with self.lock:
            if self.depth_image is None:
                return  # No depth data available
            if self.processing:
                return
            self.processing = True
        self.last_image_stamp = Time.from_msg(msg.header.stamp)
        start_time = time.time()
        self.get_logger().info("Processing image for object detection")
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.height, self.width, _ = cv_image.shape
        blob = cv2.dnn.blobFromImage(cv_image, 0.00392, (608, 608), (0, 0, 0), True, crop=False)
        self.net.setInput(blob)
        layer_names = self.net.getLayerNames()
        output_layers = [layer_names[i - 1] for i in self.net.getUnconnectedOutLayers()]
        self.detections = self.net.forward(output_layers)
        # Process detections (you can customize this part)
        for out in self.detections:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.1:  # Adjust confidence threshold as needed
                    print("DET", detection)
                    # Bounding box coordinates
                    center_x = int(detection[0] * self.width)
                    center_y = int(detection[1] * self.height)
                    w = int(detection[2] * self.width)
                    h = int(detection[3] * self.height)
                    # Draw bounding box (optional)
                    cv2.rectangle(cv_image, (center_x - w // 2, center_y - h // 2), (center_x + w // 2, center_y + h // 2), (0, 255, 0), 2)
                         # Prepare label with class name and confidence
                    label = f"{self.classes[class_id]}: {confidence:.2f}"
                    # Get the size of the label to position it
                    (w, h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                    # Put the label on the image
                    cv2.putText(cv_image, label, (center_x - w // 2, center_y - h // 2 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        # Convert the processed image back to ROS format and publish
        output_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        self.image_pub.publish(output_msg)
        elapsed_time = time.time() - start_time
        self.get_logger().info(f"DONE detect: {elapsed_time:.2f} seconds")
        self.processing = False

    def get_robot_lowres_position(self, original_origin, original_resolution):
        try:
            # Wait for the transform for up to 1 second
            trans = self.tf_buffer.lookup_transform(
                'map',          # target frame
                'base_link',    # source frame
                rclpy.time.Time(),  # get the latest available
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            robot_x = trans.transform.translation.x
            robot_y = trans.transform.translation.y
            # Convert robot's position to grid coordinates
            map_origin_x = original_origin.position.x
            map_origin_y = original_origin.position.y
            # Calculate the robot's index in the original map
            robot_map_x = int((robot_x - map_origin_x) / original_resolution)
            robot_map_y = int((robot_y - map_origin_y) / original_resolution)
            # Adjust for downsampling
            robot_lowres_x = robot_map_x // self.downsample_factor
            robot_lowres_y = robot_map_y // self.downsample_factor
            return robot_lowres_x, robot_lowres_y
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"Transform exception: {str(e)}")
            return 0, 0  # Default position

    def publish_low_res_map(self, msg):
        lowres_msg = OccupancyGrid()
        lowres_msg.header = msg.header
        lowres_msg.header.frame_id = 'map'
        lowres_msg.header.stamp = self.get_clock().now().to_msg()
        lowres_msg.info.resolution = self.new_resolution
        lowres_msg.info.width = self.new_width
        lowres_msg.info.height = self.new_height
        lowres_msg.info.origin = msg.info.origin
        lowres_msg.data = self.low_res_grid
        self.origin = msg.info.origin
        if self.robot_lowres_x is not None:
            with open("/home/nartech/nartech_ws/src/nartech_ros/channels/grid.txt","w") as f:
                f.write(str(self.new_width) + "\n" + str(self.new_height) + "\n" + str(self.robot_lowres_x) + '\n' + str(self.robot_lowres_y) + "\n" + str(self.low_res_grid))
        self.lowres_grid_pub.publish(lowres_msg)

    def get_block_occupancy(self, data, x_start, y_start, width, factor):
        for y in range(y_start, y_start + factor):
            for x in range(x_start, x_start + factor):
                if y < (len(data) // width) and x < width and y * width + x < len(data):
                    # If any cell in the block is occupied, mark the downsampled cell as occupied
                    if data[y * width + x] == 100:  # Assuming 100 indicates occupancy
                        return 100
        return 0  # Mark as free if no occupied cells are found

    def naceop_callback(self, msg):
        self.goalstart = self.mapupdate #current mapupdate
        command = msg.data.lower()
        self.get_logger().info(f"Received command: {command}")
        if self.robot_lowres_x is None:
            return
        self.start_navigation(command)

    # Determine the target low-res position based on the command, and send it as navigation goal
    def start_navigation(self, command):
        target_cell = self.get_current_target_cell(command)
        if target_cell:
            self.navigation_goal = (target_cell, command)
            self.navigation_retries = 0
            self.send_navigation_goal(target_cell, command)

    def set_orientation_with_angle(self, angle_radians):
        half_angle = angle_radians / 2.0
        return Quaternion(x=0.0, y=0.0, z=math.sin(half_angle), w=math.cos(half_angle))

    def set_orientation(self, command):
        command = command.split(",")[-1] #for orientation it is the last move that matters
        if command == "right":
            angle = 0  # Facing up (0 radians)
        elif command == "left":
            angle = math.pi  # Facing down (π radians)
        elif command == "up":
            angle = math.pi / 2  # Facing left (π/2 radians)
        elif command == "down":
            angle = -math.pi / 2  # Facing right (-π/2 radians)
        else:
            raise ValueError("Invalid command. Must be 'up', 'down', 'left', or 'right'.")
        return self.set_orientation_with_angle(angle)

    def check_collision(self, target_cell):
        cell_x, cell_y = target_cell
        idx = cell_y * self.new_width + cell_x
        if idx >= len(self.low_res_grid) or self.low_res_grid[idx] != 0:
            self.get_logger().info(f"COLLISION!!!")
            return True
        return False

    def send_navigation_goal(self, target_cell, command):
        # Convert low-res cell to world coordinates based on resolution and origin
        origin_x, origin_y = self.origin.position.x, self.origin.position.y    # Example origin x, update with actual
        if self.check_collision(target_cell):
            self.publish_done(force_mapupdate = False) #goal was not accepted so nothing happened anyway
            return
        cell_x, cell_y = target_cell
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'  # Ensure the frame matches your map
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = origin_x + (cell_x * self.new_resolution) + self.new_resolution/2
        goal_pose.pose.position.y = origin_y + (cell_y * self.new_resolution) + self.new_resolution/2
        goal_pose.pose.orientation = self.set_orientation(command)  # Adjust as needed
        self.get_logger().info(f"Sending goal to ({goal_pose.pose.position.x}, {goal_pose.pose.position.y})")
        # Check if the action client is available
        if not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Action server not available!")
            self.publish_done(force_mapupdate = False) #goal was not accepted so nothing happened anyway
            return
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        # Send goal to the action server
        sent_goal = self.action_client.send_goal_async(goal_msg)
        sent_goal.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.goal_handle = None
            self.get_logger().info("Goal rejected")
            self.publish_done(force_mapupdate = False) #goal was not accepted so nothing happened anyway
            return
        self.get_logger().info("Goal accepted, waiting for result")
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal succeeded!")
        else:
            if self.navigation_retries < 10:
                self.send_navigation_goal(self.navigation_goal[0], self.navigation_goal[1])
                self.get_logger().info("Goal failed with status: {0}, retrying".format(result.status))
                self.navigation_retries += 1
                return
            else:
                if "," in self.navigation_goal[1]:
                    self.get_logger().info("Goal failed with status: {0}, exhausted retries, shortening command".format(result.status))
                    newcommand = ",".join(self.navigation_goal[1].split(",")[1:])
                    self.start_navigation(newcommand)
                    return
                else:
                    self.get_logger().info("Goal failed with status: {0}, exhausted retries and shortenings".format(result.status))
        self.publish_done(force_mapupdate = True) #in this the goal was reached or it was at least attempted

    def publish_done(self, force_mapupdate):
        force_mapupdate = False #not needed with fast map updating speed (leave functionality for now for testing)
        # Publish the 'done' message
        if force_mapupdate:
            waittime = 0.1
            while self.mapupdate == self.goalstart: #TODO better solution, but we want to force a map update
                if waittime == 0.1:
                    self.get_logger().info("Waiting for map update!")
                miniwait = 0.1
                time.sleep(miniwait)
                waittime += miniwait
                if waittime > 10.0: #don't wait more than 10s
                    self.get_logger().warn("Wait time exceeded, finishing nevertheless!")
                    break
        msg = String()
        msg.data = str(time.time())
        self.nacedone_pub.publish(msg)
        self.get_logger().info("Published 'done' to /nacedone")

    def get_current_target_cell(self, dirs):
        # Logic to get the robot's target position in the low-res grid
        current_x, current_y = self.robot_lowres_x, self.robot_lowres_y  # Replace with actual position logic
        for direction in dirs.split(","):
            if direction == "left":
                current_x, current_y = (current_x - 1, current_y)
            elif direction == "right":
                current_x, current_y = (current_x + 1, current_y)
            elif direction == "up":
                current_x, current_y = (current_x, current_y + 1)
            elif direction == "down":
                current_x, current_y = (current_x, current_y - 1)
        return (current_x, current_y)

def main(args=None):
    rclpy.init(args=args)
    node = LowResGridMapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
