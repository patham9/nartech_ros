# nav.py
import time
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
# Assume mettabridge defines these constants and functions:
from mettabridge import NAV_STATE_SET, NAV_STATE_BUSY, NAV_STATE_SUCCESS, NAV_STATE_FAIL

class Navigation:
    def __init__(self, node: Node, semantic_slam, localization):
        self.node = node
        self.semantic_slam = semantic_slam
        self.localization = localization
        self.navigation_goal = None
        self.navigation_retries = 0
        self.goal_handle = None
        qos_profile_str = rclpy.qos.QoSProfile(depth=1)
        qos_profile_str.history = rclpy.qos.QoSHistoryPolicy.KEEP_LAST
        qos_profile_str.durability = rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos_profile_str.reliability = rclpy.qos.QoSReliabilityPolicy.RELIABLE
        self.naceop_sub = self.node.create_subscription(
            String, '/naceop', self.naceop_callback, qos_profile_str)
        self.nacedone_pub = self.node.create_publisher(String, '/nacedone', qos_profile_str)
        self.action_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')

    def naceop_callback(self, msg):
        self.semantic_slam.goalstart = self.semantic_slam.mapupdate  # capture current map update state
        command = msg.data.lower()
        self.node.get_logger().info(f"Received command: {command}")
        self.start_navigation_by_moves(command)

    def start_navigation_by_moves(self, command):
        if self.semantic_slam.robot_lowres_x is None:
            return
        target_cell = self.get_current_target_cell(command)
        if target_cell:
            self.start_navigation_to_coordinate(target_cell, command)

    def start_navigation_to_coordinate(self, target_cell, command=""):
        NAV_STATE_SET(NAV_STATE_BUSY)
        if self.semantic_slam.robot_lowres_x is None:
            return
        self.navigation_goal = (target_cell, command)
        self.navigation_retries = 0
        self.send_navigation_goal(target_cell, command)

    def send_navigation_goal(self, target_cell, command):
        origin_x, origin_y = self.semantic_slam.origin.position.x, self.semantic_slam.origin.position.y
        import sys
        if not any(arg.endswith(".metta") for arg in sys.argv) and self.check_collision(target_cell):
            if "," in self.navigation_goal[1]:
                self.node.get_logger().info("COLLISION, shortening command")
                newcommand = ",".join(self.navigation_goal[1].split(",")[1:])
                self.start_navigation_by_moves(newcommand)
                return
            else:
                self.node.get_logger().info("COLLISION, aborting")
                self.publish_done(force_mapupdate=False)
                NAV_STATE_SET(NAV_STATE_FAIL)
                return
        cell_x, cell_y = target_cell
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_pose.pose.position.x = origin_x + (cell_x * self.semantic_slam.new_resolution) + self.semantic_slam.new_resolution / 2
        goal_pose.pose.position.y = origin_y + (cell_y * self.semantic_slam.new_resolution) + self.semantic_slam.new_resolution / 2
        goal_pose.pose.orientation = self.localization.set_orientation(command)
        self.node.get_logger().info(f"Sending goal to ({goal_pose.pose.position.x}, {goal_pose.pose.position.y})")
        if not self.action_client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().error("Action server not available!")
            self.publish_done(force_mapupdate=False)
            return
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        sent_goal = self.action_client.send_goal_async(goal_msg)
        sent_goal.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.goal_handle = None
            self.node.get_logger().info("Goal rejected")
            self.publish_done(force_mapupdate=False)
            return
        self.node.get_logger().info("Goal accepted, waiting for result")
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        result = future.result()
        nav_state = NAV_STATE_SUCCESS
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().info("Goal succeeded!")
        else:
            if self.navigation_retries < 10 and "metta" not in __import__("sys").argv:
                self.node.get_logger().info("Goal failed with status: {0}, retrying".format(result.status))
                self.navigation_retries += 1
                self.send_navigation_goal(self.navigation_goal[0], self.navigation_goal[1])
                return
            else:
                if "," in self.navigation_goal[1]:
                    self.node.get_logger().info("Goal failed with status: {0}, exhausted retries, shortening command".format(result.status))
                    newcommand = ",".join(self.navigation_goal[1].split(",")[1:])
                    self.start_navigation_by_moves(newcommand)
                    return
                else:
                    self.node.get_logger().info("Goal failed with status: {0}, exhausted retries and shortenings".format(result.status))
        self.publish_done(force_mapupdate=True)
        NAV_STATE_SET(nav_state)

    def publish_done(self, force_mapupdate):
        force_mapupdate = False  # as in the original code, map updating is fast so we disable forced waiting
        if force_mapupdate:
            waittime = 0.1
            while self.semantic_slam.mapupdate == self.semantic_slam.goalstart:
                if waittime == 0.1:
                    self.node.get_logger().info("Waiting for map update!")
                miniwait = 0.1
                time.sleep(miniwait)
                waittime += miniwait
                if waittime > 10.0:
                    self.node.get_logger().warn("Wait time exceeded, finishing nevertheless!")
                    break
        msg = String()
        msg.data = str(time.time())
        self.nacedone_pub.publish(msg)
        self.node.get_logger().info("Published 'done' to /nacedone")

    def get_current_target_cell(self, dirs):
        current_x, current_y = self.semantic_slam.robot_lowres_x, self.semantic_slam.robot_lowres_y
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

    def check_collision(self, target_cell):
        cell_x, cell_y = target_cell
        idx = cell_y * self.semantic_slam.new_width + cell_x
        if idx < len(self.semantic_slam.low_res_grid) and self.semantic_slam.low_res_grid[idx] not in [0, -1, 127]:
            self.node.get_logger().info("COLLISION!!!")
            return True
        return False
