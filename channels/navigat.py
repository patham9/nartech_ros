# nav.py
import sys
import time
import math
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from action_msgs.msg import GoalStatus
# Assume mettabridge defines these constants and functions:
from mettabridge import NAV_STATE_SET, NAV_STATE_GET, NAV_STATE_BUSY, NAV_STATE_SUCCESS, NAV_STATE_FAIL

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
        self.naceop_sub = self.node.create_subscription(String, '/naceop', self.naceop_callback, qos_profile_str)
        self.nacedone_pub = self.node.create_publisher(String, '/nacedone', qos_profile_str)
        self.action_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')
        self.cmd_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.mettacontrolled = False
        self._pending_cancel = False
        for arg in sys.argv:
            if arg == "metta" or arg.endswith(".metta"): #we let explicit MeTTa code control the robot
                self.mettacontrolled = True

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
            self.start_navigation_to_coordinate(target_cell, "", command)

    def start_navigation_to_coordinate(self, target_cell, objectlabel, command=""):
        self.state = NAV_STATE_SET(NAV_STATE_BUSY)
        #retrieve current (potentially updated) position estimate
        self.objectlabel = objectlabel
        self.target_point = None
        #if objectlabel and objectlabel in self.node.semantic_slam.previous_detections:
        #    (t, object_grid_x, object_grid_y, origin_x, origin_y, spoint_map, spoint_base_link, imagecoords_depth) = self.semantic_slam.previous_detections[objectlabel]
        #    self.target_point = spoint_map
        if self.semantic_slam.robot_lowres_x is None:
            return
        self.navigation_goal = (target_cell, command)
        self.navigation_retries = 0
        self.send_navigation_goal(target_cell, command)

    def send_navigation_goal(self, target_cell, command):
        origin_x, origin_y = self.semantic_slam.origin.position.x, self.semantic_slam.origin.position.y
        if not self.mettacontrolled and self.check_collision(target_cell):
            if "," in self.navigation_goal[1]:
                self.node.get_logger().info("COLLISION, shortening command")
                newcommand = ",".join(self.navigation_goal[1].split(",")[1:])
                self.start_navigation_by_moves(newcommand)
                return
            else:
                self.node.get_logger().info("COLLISION, aborting")
                self.publish_done(force_mapupdate=False)
                self.state = NAV_STATE_SET(NAV_STATE_FAIL)
                return
        cell_x, cell_y = (None, None)
        if target_cell is not None:
            cell_x, cell_y = target_cell
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.node.get_clock().now().to_msg()
        if target_cell is not None:
            goal_pose.pose.position.x = origin_x + (cell_x * self.semantic_slam.new_resolution) + self.semantic_slam.new_resolution / 2
            goal_pose.pose.position.y = origin_y + (cell_y * self.semantic_slam.new_resolution) + self.semantic_slam.new_resolution / 2
        if self.target_point is not None:
            # Assume current_pose is your robot’s current position in map frame
            current_x = self.semantic_slam.trans.transform.translation.x
            current_y = self.semantic_slam.trans.transform.translation.y
            # Vector from current position to target
            dx = self.target_point.point.x - current_x
            dy = self.target_point.point.y - current_y
            distance = math.hypot(dx, dy)
            ARM_REACH_DISTANCE = 1.0 #TODO
            # Stop 0.4 m before the target
            if distance > ARM_REACH_DISTANCE:
                scale = (distance - ARM_REACH_DISTANCE) / distance
                # Compute yaw angle toward target
                yaw = math.atan2(dy, dx)
                # Create quaternion for yaw-only rotation
                q = Quaternion()
                q.w = math.cos(yaw / 2.0)
                q.x = 0.0
                q.y = 0.0
                q.z = math.sin(yaw / 2.0)
                # Set orientation
                goal_pose.pose.orientation = q
                goal_pose.pose.position.x = current_x + dx * scale
                goal_pose.pose.position.y = current_y + dy * scale
        else:
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
         # ── one-shot 20 s watchdog ────────────────────────────────────────
        if getattr(self, "_nav_timeout_timer", None):
            self._nav_timeout_timer.cancel()        # clear any previous timer
        def _nav_timeout_cb():
            self.node.get_logger().warn("Navigation timeout – cancelling goal")
            self.cancel_goals()                     # abort Nav2 goals
            self._nav_timeout_timer.cancel()        # make it one-shot
        self._nav_timeout_timer = self.node.create_timer(30.0, _nav_timeout_cb)
        # ──────────────────────────────────────────────────────────────────

    def _goal_response_callback(self, future):
        self.goal_handle = future.result()
        if self._pending_cancel:
            self._pending_cancel = False
            self.node.get_logger().info("Navigation: late cancel")
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
            self.publish_done(force_mapupdate=False)
            self.state = NAV_STATE_SET(NAV_STATE_FAIL)
            return
        if not self.goal_handle.accepted:
            self.goal_handle = None
            self.node.get_logger().info("Goal rejected")
            self.publish_done(force_mapupdate=False)
            self.state = NAV_STATE_SET(NAV_STATE_FAIL)
            return
        self.node.get_logger().info("Goal accepted, waiting for result")
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        result = future.result()
        nav_state = NAV_STATE_SUCCESS
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().info("Goal succeeded!")
            #if self.objectlabel: #orient to object
            #    self.start_navigation_to_coordinate(self.navigation_goal[0], self.objectlabel, command="")
        else:
            if self.navigation_retries < 10: # and not self.mettacontrolled:
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
                    nav_state = NAV_STATE_FAIL
                    self.node.get_logger().info("Goal failed with status: {0}, exhausted retries and shortenings".format(result.status))
        self.publish_done(force_mapupdate=True)
        self.goal_handle = None
        if self._pending_cancel:
            self.node.get_logger().info("Suppressing NAV_STATE update due to pending cancel (e.g. from slip)")
            self._pending_cancel = False  # clear it now
        else:
            self.state = NAV_STATE_SET(nav_state)

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

    def cancel_goals(self):
        self._pending_cancel = True
        self.navigation_retries = 10
         # start “brake” timer – 20 Hz zero /cmd_vel
        self._brake_timer = self.node.create_timer(0.05, lambda: self.cmd_pub.publish(Twist()))
        if not self.action_client.server_is_ready():
            # Action server is not up (simulation paused, early startup, etc.)
            self.node.get_logger().warn("Navigation: Nav2 action server not ready, nothing to cancel")
            if self._brake_timer:
                self._brake_timer.cancel()
                self._brake_timer = None
                self._pending_cancel = False
            self.state = NAV_STATE_SET(NAV_STATE_FAIL)
            return
        # Ask Nav2 to drop every outstanding goal
        if self.goal_handle is None:
            self.node.get_logger().info("Navigation: no active Nav2 goal to cancel")
            if self._brake_timer:
                self._brake_timer.cancel()
                self._brake_timer = None
            self._pending_cancel = False
            self.state = NAV_STATE_SET(NAV_STATE_FAIL)
            return
        cancel_future = self.goal_handle.cancel_goal_async()
        def _on_cancel_done(fut):
            try:
                fut.result()
                self.node.get_logger().info("Navigation: cancelled Nav2 goal")
            except Exception as e:
                self.node.get_logger().warn(f"Navigation: cancel_goal_async failed: {e!r}")
            finally:
                if self._brake_timer:
                    self._brake_timer.cancel()
                    self._brake_timer = None
                self.state = NAV_STATE_SET(NAV_STATE_FAIL)
        cancel_future.add_done_callback(_on_cancel_done)
