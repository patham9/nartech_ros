#!/usr/bin/env python3
# main.py
import rclpy
from rclpy.node import Node
import tf2_ros
import sys
sys.path.append('./channels/')
from objectdetection import ObjectDetector
from localization import Localization
from semanticslam import SemanticSLAM
from navigat import Navigation
#from armcontroller import ArmController
from rclpy.parameter import Parameter
from rclpy.duration import Duration

class MainNode(Node):
    def __init__(self):
        super().__init__('NARTECH_node', parameter_overrides=[Parameter('use_sim_time', value=True)])
        # Create a single TF2 buffer and listener to be shared across modules.
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)
        # Instantiate the helper modules.
        self.pick = lambda x: None #overridden
        self.drop = lambda x: None #overridden
        self.object_detector = ObjectDetector(self, self.tf_buffer)
        self.localization = Localization(self, self.tf_buffer)
        self.semantic_slam = SemanticSLAM(self, self.tf_buffer, self.localization, self.object_detector)
        self.navigation = Navigation(self, self.semantic_slam, self.localization)
        self.start_navigation_to_coordinate = self.navigation.start_navigation_to_coordinate
        #self.arm_controller = ArmController(self, self.semantic_slam, self.navigation)
        #self.pick = self.arm_controller.pick
        #self.drop = self.arm_controller.drop

def main(args=None):
    rclpy.init(args=args)
    node = MainNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
