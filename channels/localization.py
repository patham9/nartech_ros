# robot.py
import math
import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
import tf2_ros
from geometry_msgs.msg import Quaternion

class Localization:
    def __init__(self, node, tf_buffer):
        self.node = node
        self.tf_buffer = tf_buffer

    def get_robot_lowres_position(self, original_origin, original_resolution, downsample_factor):
        try:
            trans = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0)
            )
            robot_x = trans.transform.translation.x
            robot_y = trans.transform.translation.y
            map_origin_x = original_origin.position.x
            map_origin_y = original_origin.position.y
            robot_map_x = int((robot_x - map_origin_x) / original_resolution)
            robot_map_y = int((robot_y - map_origin_y) / original_resolution)
            robot_lowres_x = robot_map_x // downsample_factor
            robot_lowres_y = robot_map_y // downsample_factor
            return robot_lowres_x, robot_lowres_y
        except Exception as e:
            self.node.get_logger().error(f"Transform exception: {str(e)}")
            return None, None  # Default position if transform fails

    def set_orientation_with_angle(self, angle_radians):
        if angle_radians is None:
            return Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        half_angle = angle_radians / 2.0
        return Quaternion(x=0.0, y=0.0, z=math.sin(half_angle), w=math.cos(half_angle))

    def set_orientation(self, command):
        # Use the last comma‐separated part of the command for orientation.
        command = command.split(",")[-1]
        if command == "right":
            angle = 0  # Facing up (0 radians)
        elif command == "left":
            angle = math.pi  # Facing down (π radians)
        elif command == "up":
            angle = math.pi / 2  # Facing left (π/2 radians)
        elif command == "down":
            angle = -math.pi / 2  # Facing right (-π/2 radians)
        else:
            return self.set_orientation_with_angle(None)
        return self.set_orientation_with_angle(angle)
