# semanticslam.py
import tf2_geometry_msgs
import time
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, Point, PointStamped
from std_msgs.msg import Header
from rclpy.time import Time
import tf2_ros
import math
# Assume mettabridge provides space_tick (and any other helper functions/constants)
from mettabridge import space_tick

class SemanticSLAM:
    def __init__(self, node, tf_buffer, localization, object_detector):
        self.node = node
        self.tf_buffer = tf_buffer
        self.localization = localization
        self.object_detector = object_detector
        # Mapping from object category to occupancy value
        self.M = {
            "wall": 100, "robot": 127, "chair": -126, "table": -126,
            "bottle": -125, "cup": -125, "can": -125, "person": -124, "unknown": -1
        }
        self.previous_detections_persistence = 100000.0  # seconds
        self.previous_detections = {}
        # Downsampling parameters
        self.downsample_factor = 28
        # Set up a QoS profile for map topics.
        qos_profile_map = QoSProfile(depth=1)
        qos_profile_map.history = QoSHistoryPolicy.KEEP_LAST
        qos_profile_map.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.map_sub = self.node.create_subscription(
            OccupancyGrid, '/map', self.occ_grid_callback, qos_profile_map)
        self.lowres_grid_pub = self.node.create_publisher(
            OccupancyGrid, '/lowres_map', qos_profile_map)
        # Timer to periodically process and publish the low-res grid.
        self.timer = self.node.create_timer(2.0, self.build_grid_periodic)
        # State variables used during grid processing.
        self.cached_msg = None
        self.low_res_grid = None
        self.robot_lowres_x = None
        self.robot_lowres_y = None
        self.new_width = 0
        self.new_height = 0
        self.new_resolution = None
        self.origin = None
        self.mapupdate = 0
        self.goalstart = 0

    def occ_grid_callback(self, msg):
        self.node.get_logger().info("NEW OCC GRID")
        self.cached_msg = msg
        self.build_grid_periodic()

    def build_grid_periodic(self):
        if self.cached_msg is None:
            self.node.get_logger().info("WAITING FOR OCC GRID")
            return
        start_time = time.time()
        self.node.get_logger().info("NEW PROCESSING: OCC GRID")
        msg = self.cached_msg
        original_width = msg.info.width
        original_height = msg.info.height
        original_resolution = msg.info.resolution
        original_origin = msg.info.origin
        original_data = msg.data
        # Compute new grid dimensions and resolution.
        self.new_width = original_width // self.downsample_factor
        self.new_height = original_height // self.downsample_factor
        self.new_resolution = original_resolution * self.downsample_factor
        if original_width % self.downsample_factor != 0:
            self.new_width += 1
        if original_height % self.downsample_factor != 0:
            self.new_height += 1
        # Initialize a low-resolution grid (default free cells are 0).
        self.low_res_grid = [0] * (self.new_width * self.new_height)
        for y in range(0, original_height, self.downsample_factor):
            for x in range(0, original_width, self.downsample_factor):
                new_x = x // self.downsample_factor
                new_y = y // self.downsample_factor
                new_idx = new_y * self.new_width + new_x
                cell_value = self.get_block_occupancy(original_data, x, y, original_width, self.downsample_factor)
                self.low_res_grid[new_idx] = cell_value
        # Update robot position using the Localization module.
        self.robot_lowres_x, self.robot_lowres_y = self.localization.get_robot_lowres_position(
            original_origin, original_resolution, self.downsample_factor)
        if self.robot_lowres_x is not None and 0 <= self.robot_lowres_x < self.new_width and 0 <= self.robot_lowres_y < self.new_height:
            robot_idx = self.robot_lowres_y * self.new_width + self.robot_lowres_x
            self.low_res_grid[robot_idx] = 127  # Mark the robot cell.
            self.previous_detections["self"] = (time.time(), self.robot_lowres_x, self.robot_lowres_y,
                                                original_origin.position.x, original_origin.position.y)
            self.node.get_logger().info(f"Marked robot position at ({self.robot_lowres_x}, {self.robot_lowres_y}) as occupied.")
        else:
            self.node.get_logger().warn("Robot position is out of bounds in the downsampled map.")
        # Process detections from the YOLODetector module.
        object_detections = self.object_detector.detections
        depth_image = self.object_detector.depth_image
        if object_detections is not None:
            for out in object_detections:
                for detection in out:
                    scores = detection[5:]
                    class_id = int(np.argmax(scores))
                    confidence = scores[class_id]
                    if confidence > 0.5:  # Confidence threshold for semantic mapping
                        center_x = int(detection[0] * self.object_detector.width)
                        center_y = int(detection[1] * self.object_detector.height)
                        if depth_image is None:
                            self.node.get_logger().warn("Got detections but no depth image")
                            continue
                        depth_value = depth_image[center_y, center_x]
                        category = self.object_detector.classes[class_id]
                        if depth_value > 0 and category in self.M:
                            stamp = self.object_detector.last_image_stamp if self.object_detector.last_image_stamp else self.node.get_clock().now()
                            self.node.get_logger().info(f"DEPTH DEBUG: {depth_value}")
                            # Create a point in camera coordinates.
                            camera_point = PointStamped(
                                header=Header(stamp=stamp.to_msg(), frame_id='oakd_left_camera_frame'),
                                point=Point(
                                    x=depth_value,
                                    y=-(center_x - (self.object_detector.width / 2)) * depth_value / self.object_detector.fx,
                                    z=-(center_y - (self.object_detector.height / 2)) * depth_value / self.object_detector.fy
                                )
                            )
                            try:
                                # Transform the point into the map frame.
                                transformed_point = self.tf_buffer.transform(camera_point, 'map', timeout=Duration(seconds=1.0))
                                object_grid_x, object_grid_y = self.get_lowres_position(
                                    transformed_point.point.x, transformed_point.point.y,
                                    original_origin, self.new_resolution
                                )
                                if 0 <= object_grid_x < self.new_width and 0 <= object_grid_y < self.new_height:
                                    obj_idx = object_grid_y * self.new_width + object_grid_x
                                    self.previous_detections[category] = (time.time(), object_grid_x, object_grid_y,
                                                                          original_origin.position.x, original_origin.position.y)
                                    self.low_res_grid[obj_idx] = self.M[category]
                                    self.node.get_logger().info(f"Marked detected object at ({object_grid_x}, {object_grid_y}) in grid.")
                                else:
                                    self.node.get_logger().warn("Detected object position is out of bounds in the downsampled map.")
                            except Exception as e:
                                self.node.get_logger().error(f"Transform exception: {str(e)}")
        else:
            self.node.get_logger().warn("No detections received")
        # Update previously detected objects based on map shifts.
        for category in list(self.previous_detections.keys()):
            (t, object_grid_x, object_grid_y, old_origin_x, old_origin_y) = self.previous_detections[category]
            current_origin_x = msg.info.origin.position.x
            current_origin_y = msg.info.origin.position.y
            offset_x = int(round((old_origin_x - current_origin_x) / self.new_resolution))
            offset_y = int(round((old_origin_y - current_origin_y) / self.new_resolution))
            new_object_grid_x = object_grid_x + offset_x
            new_object_grid_y = object_grid_y + offset_y
            self.previous_detections[category] = (t, new_object_grid_x, new_object_grid_y, current_origin_x, current_origin_y)
            if 0 <= new_object_grid_x < self.new_width and 0 <= new_object_grid_y < self.new_height:
                obj_idx = new_object_grid_y * self.new_width + new_object_grid_x
                if time.time() - t < self.previous_detections_persistence and self.low_res_grid[obj_idx] != 127:
                    self.low_res_grid[obj_idx] = self.M[category]
        self.publish_low_res_map(msg)
        elapsed_time = time.time() - start_time
        self.node.get_logger().info(f"DONE map: {elapsed_time:.2f} seconds")
        # If using metta mode, call space_tick (this call is preserved from the original code)
        import sys
        if self.robot_lowres_x is not None and any(arg.endswith(".metta") for arg in sys.argv):
            space_tick(self.node)

    def get_lowres_position(self, world_x, world_y, original_origin, original_resolution):
        grid_x = int((world_x - original_origin.position.x) / original_resolution)
        grid_y = int((world_y - original_origin.position.y) / original_resolution)
        return grid_x, grid_y

    def get_block_occupancy(self, data, x_start, y_start, width, factor):
        occupied = False
        empty = False
        for y in range(y_start, y_start + factor):
            for x in range(x_start, x_start + factor):
                if y < (len(data) // width) and x < width and (y * width + x) < len(data):
                    if data[y * width + x] == 100:
                        occupied = True
                    elif data[y * width + x] == 0:
                        empty = True
        if occupied:
            return 100
        if empty:
            return 0
        return -1

    def publish_low_res_map(self, original_msg):
        from nav_msgs.msg import OccupancyGrid
        lowres_msg = OccupancyGrid()
        lowres_msg.header = original_msg.header
        lowres_msg.header.frame_id = 'map'
        lowres_msg.header.stamp = self.node.get_clock().now().to_msg()
        lowres_msg.info.resolution = self.new_resolution
        lowres_msg.info.width = self.new_width
        lowres_msg.info.height = self.new_height
        lowres_msg.info.origin = original_msg.info.origin
        lowres_msg.data = self.low_res_grid
        self.origin = original_msg.info.origin
        try:
            with open("/home/nartech/nartech_ws/src/nartech_ros/channels/grid.txt","w") as f:
                f.write(str(self.new_width) + "\n" +
                        str(self.new_height) + "\n" +
                        str(self.robot_lowres_x) + "\n" +
                        str(self.robot_lowres_y) + "\n" +
                        str(self.low_res_grid))
        except Exception as e:
            self.node.get_logger().error(f"Error writing grid file: {e}")
        self.lowres_grid_pub.publish(lowres_msg)
