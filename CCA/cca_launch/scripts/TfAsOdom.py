#!/usr/bin/env python3
import sys

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rcl_interfaces.msg import ParameterDescriptor
from tf2_ros import TransformStamped

import rclpy.timer
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry

class tf_app(Node):

    def __init__(self):
        #node init
        super().__init__("tf_app")
        self.get_logger().info("This the topic transformer has started...")
        self.declare_parameter('fixed_frame', 'map', ParameterDescriptor(description='Map frame link name.'))
        self.fixed_frame = self.get_parameter('fixed_frame').get_parameter_value().string_value
        self.current_position_stream = self.create_subscription(Odometry,"odom", self.read_current_position, 10)
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL,history=HistoryPolicy.KEEP_LAST,depth=1)
        self.current_transformed_position_stream = self.create_publisher(Odometry, "tf_odom",qos_profile)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
    
    def read_current_position(self, Odom: Odometry):
        try:
            drift = TransformStamped()
            drift = self.tf_buffer.lookup_transform(self.fixed_frame, Odom.child_frame_id, rclpy.time.Time())
        except TransformException as expn:
            self.get_logger().error(f"Could not get transform: {expn}")
            return  #abort
        
        pose = Odom
        pose.header.frame_id = self.fixed_frame #admin
        pose.pose.pose.position.x = drift.transform.translation.x #translation
        pose.pose.pose.position.y = drift.transform.translation.y
        pose.pose.pose.position.z = drift.transform.translation.z
        pose.pose.pose.orientation = drift.transform.rotation
        self.current_transformed_position_stream.publish(pose)

def main(args=None):
    rclpy.init(args=args)
    node = tf_app()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("")
        sys.exit(0)