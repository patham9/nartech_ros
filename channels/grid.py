#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
import tf2_ros
from tf2_ros.buffer import Buffer
from geometry_msgs.msg import TransformStamped

class LowResGridMapPublisher(Node):
    def __init__(self):
        super().__init__('lowres_gridmap_publisher')
        
        # Define QoS to keep only the latest image
        qos_profile = QoSProfile(depth=1)
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        
        # Parameters for resolution reduction
        self.downsample_factor = 20  # Change as needed

        # TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Subscribing to the occupancy grid map
        self.occ_grid_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.occ_grid_callback,
            qos_profile
        )

        # Publisher for the low-resolution grid map
        self.lowres_grid_pub = self.create_publisher(OccupancyGrid, '/lowres_map', qos_profile)

    def occ_grid_callback(self, msg):
        # Get original grid map data
        start_time = time.time()
        self.get_logger().info("NEW PROCESSING")
        
        original_width = msg.info.width
        original_height = msg.info.height
        original_resolution = msg.info.resolution
        original_origin = msg.info.origin
        original_data = msg.data

        # Compute new width and height
        new_width = original_width // self.downsample_factor
        new_height = original_height // self.downsample_factor
        new_resolution = original_resolution * self.downsample_factor
        
        # If original dimensions are not evenly divisible, adjust the boundaries
        if original_width % self.downsample_factor != 0:
            new_width += 1
        if original_height % self.downsample_factor != 0:
            new_height += 1

        # Initialize new grid map data with zeros (free)
        new_data = [0] * (new_width * new_height)

        # Mark occupied cells based on downsampling
        for y in range(0, original_height, self.downsample_factor):
            for x in range(0, original_width, self.downsample_factor):
                # Calculate the index for the downsampled map
                new_x = x // self.downsample_factor
                new_y = y // self.downsample_factor
                new_idx = new_y * new_width + new_x

                # Check if any cell in the block is occupied
                cell_value = self.get_block_occupancy(
                    original_data, x, y, original_width, self.downsample_factor
                )
                
                new_data[new_idx] = cell_value

        # Get robot's current position
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

            # Ensure indices are within bounds
            if 0 <= robot_lowres_x < new_width and 0 <= robot_lowres_y < new_height:
                robot_idx = robot_lowres_y * new_width + robot_lowres_x
                new_data[robot_idx] = 127  # Mark as occupied
                self.get_logger().info(f"Marked robot position at ({robot_lowres_x}, {robot_lowres_y}) as occupied.")
            else:
                self.get_logger().warn("Robot position is out of bounds in the downsampled map.")
        
        except tf2_ros.LookupException:
            self.get_logger().error("Transform from map to base_link not found.")
        except tf2_ros.ConnectivityException:
            self.get_logger().error("Connectivity exception while looking up transform.")
        except tf2_ros.ExtrapolationException:
            self.get_logger().error("Extrapolation exception while looking up transform.")

        # Publish the low-resolution map
        lowres_msg = OccupancyGrid()
        lowres_msg.header = msg.header
        lowres_msg.header.frame_id = 'map'
        lowres_msg.header.stamp = self.get_clock().now().to_msg()
        lowres_msg.info.resolution = new_resolution
        lowres_msg.info.width = new_width
        lowres_msg.info.height = new_height
        lowres_msg.info.origin = msg.info.origin
        lowres_msg.data = new_data
        
        self.lowres_grid_pub.publish(lowres_msg)
        
        elapsed_time = time.time() - start_time
        self.get_logger().info(f"DONE PROCESSING: {elapsed_time:.2f} seconds")

    def get_block_occupancy(self, data, x_start, y_start, width, factor):
        for y in range(y_start, y_start + factor):
            for x in range(x_start, x_start + factor):
                if y < (len(data) // width) and x < width and y * width + x < len(data):
                    # If any cell in the block is occupied, mark the downsampled cell as occupied
                    if data[y * width + x] == 100:  # Assuming 100 indicates occupancy
                        return 100
        return 0  # Mark as free if no occupied cells are found

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
