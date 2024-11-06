#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import TransformStamped, PoseStamped
from std_msgs.msg import String
import tf2_ros
from tf2_ros.buffer import Buffer
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav2_msgs.msg import Costmap

class LowResGridMapPublisher(Node):
    def __init__(self):
        super().__init__('lowres_gridmap_publisher')
        # Define QoS to keep only the latest image for string commands:
        qos_profile_str = QoSProfile(depth=1)
        qos_profile_str.history = QoSHistoryPolicy.KEEP_LAST
        qos_profile_str.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos_profile_str.reliability = QoSReliabilityPolicy.RELIABLE
        # Define QoS to keep only the latest image for maps:
        qos_profile = QoSProfile(depth=10)
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        # Parameters for resolution reduction
        self.downsample_factor = 20  # Change as needed
        # TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        # Subscribing to the occupancy grid map
        self.occ_grid_sub = self.create_subscription(OccupancyGrid, '/map', self.occ_grid_callback, qos_profile)
        # Publisher for the low-resolution grid map
        self.lowres_grid_pub = self.create_publisher(OccupancyGrid, '/lowres_map', qos_profile)
        # Subscriber to listen for movement commands
        self.naceop_sub = self.create_subscription(String, '/naceop', self.naceop_callback, qos_profile_str)
        # Publisher to indicate movement completion
        self.nacedone_pub = self.create_publisher(String, '/nacedone', qos_profile_str)
        # Initialize Action Client
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # value that are set later after messages are received
        self.origin = None
        self.resolution = None
        self.robot_lowres_x = None
        self.robot_lowres_y = None
        self.new_data = None
        self.mapupdate = 0
        self.goalstart = 0
        self.new_width = 0

    def occ_grid_callback(self, msg):
        # Get original grid map data
        self.mapupdate += 1
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
        self.new_data = new_data
        self.new_width = new_width
        # Mark occupied cells based on downsampling
        for y in range(0, original_height, self.downsample_factor):
            for x in range(0, original_width, self.downsample_factor):
                # Calculate the index for the downsampled map
                new_x = x // self.downsample_factor
                new_y = y // self.downsample_factor
                new_idx = new_y * new_width + new_x
                # Check if any cell in the block is occupied
                cell_value = self.get_block_occupancy(original_data, x, y, original_width, self.downsample_factor)
                new_data[new_idx] = cell_value
        # Get robot's current position
        robot_lowres_x = None
        robot_lowres_y = None
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
            self.robot_lowres_x = robot_lowres_x
            self.robot_lowres_y = robot_lowres_y
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
        self.resolution = new_resolution
        lowres_msg.info.width = new_width
        lowres_msg.info.height = new_height
        lowres_msg.info.origin = msg.info.origin
        self.origin = msg.info.origin
        lowres_msg.data = new_data
        if robot_lowres_x is not None:
            with open("/home/nartech/nartech_ws/src/nartech_ros/channels/grid.txt","w") as f:
                f.write(str(new_width) + "\n" + str(new_height) + "\n" + str(robot_lowres_x) + '\n' + str(robot_lowres_y) + "\n" + str(new_data))
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
        # Determine the target low-res position based on the command
        target_cell = self.get_current_target_cell(command)
        if target_cell:
            self.send_navigation_goal(target_cell)

    def send_navigation_goal(self, target_cell):
        cell_x, cell_y = target_cell
        # Convert low-res cell to world coordinates based on resolution and origin
        resolution = self.resolution
        origin_x, origin_y = self.origin.position.x, self.origin.position.y    # Example origin x, update with actual
        #origin_y = 0.0    # Example origin y, update with actual
        idx = cell_y * self.new_width + cell_x
        if self.new_data[idx] != 0:
            self.get_logger().info(f"COLLISION!!!")
            self.publish_done(force_mapupdate = False) #goal was not accepted so nothing happened anyway
            return
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'  # Ensure the frame matches your map
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = origin_x + (cell_x * resolution) + resolution/2
        goal_pose.pose.position.y = origin_y + (cell_y * resolution) + resolution/2
        goal_pose.pose.orientation.w = 1.0  # Adjust as needed
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
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            self.publish_done(force_mapupdate = False) #goal was not accepted so nothing happened anyway
            return
        self.get_logger().info("Goal accepted, waiting for result")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal succeeded!")
        else:
            self.get_logger().info("Goal failed with status: {0}".format(result.status))
        self.publish_done(force_mapupdate = True) #in this the goal was reached or it was at least attempted

    def publish_done(self, force_mapupdate):
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

    def get_current_target_cell(self, direction):
        # Logic to get the robot's target position in the low-res grid
        current_x, current_y = self.robot_lowres_x, self.robot_lowres_y  # Replace with actual position logic.
        if direction == "left":
            return (current_x - 1, current_y)
        elif direction == "right":
            return (current_x + 1, current_y)
        elif direction == "up":
            return (current_x, current_y + 1)
        elif direction == "down":
            return (current_x, current_y - 1)
        return None


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
