#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.task import Future
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rclpy.time
from cca_interfaces.srv import GlobalPlan, PathFilter
from cca_interfaces.msg import RefSeq, SplineRef
from cca_interfaces.action import Drive
from cca_utilities.spline_works import Spline, Spline_make
from math import modf

class Voyager(Node):

    def __init__(self):
        super().__init__("Voyager")
        self.get_logger().info("The voyager has started. Initialising...")
        self.declare_parameter('vehicle_mission_speed', 1.0, ParameterDescriptor(description='Vehicle path traversal speed.'))
        self.declare_parameter('vehicle_mission_reference_tolerance', -1.0, ParameterDescriptor(description='Maximum allowable deviation of spline reference from global plan points.'))
        self.declare_parameter('map_height', 0.0, ParameterDescriptor(description='height of relevant obstacles.'))
        self.miss_spd = self.get_parameter('vehicle_mission_speed').get_parameter_value().double_value
        self.miss_tol = self.get_parameter('vehicle_mission_reference_tolerance').get_parameter_value().double_value
        self.map_z = self.get_parameter('map_height').get_parameter_value().double_value
        self.goal_sub = self.create_subscription(PoseStamped,'goal_pose',self.get_goal,10)
        self.pose_sub = self.create_subscription(Odometry,'initial_pose',self.get_pose,10)
        self.ref_sub = self.create_subscription(RefSeq,'mission', self.request_mission,10)
        self.plan_srv = self.create_client(GlobalPlan, 'Global/get_plan')
        self.ref_srv = self.create_client(PathFilter, 'Path_filter/filter')
        self.ref_pub = self.create_publisher(RefSeq,'plan',10)
        self.ref_act = ActionClient(self, Drive, 'Controller_Server/Go_to_waypoint')
        self.start = Odometry()
        self.global_future: Future = None
        self.ref_future: Future = None

    def get_pose(self, odom_msg: Odometry):
        self.start = odom_msg

    def get_goal(self, goal_msg: PoseStamped):
        req_plan = GlobalPlan.Request()
        req_plan.start = self.start.pose.pose.position
        req_plan.start.z = self.map_z
        req_plan.end = goal_msg.pose.position
        req_plan.end.z = self.map_z
        while not self.plan_srv.wait_for_service(timeout_sec=5.0):
            self.get_logger().info(f'service {self.plan_srv.srv_name} not available, waiting...')
        if self.global_future is not None and not self.global_future.done():
            self.global_future.cancel()  # what to do if a goal is requested before the previous completes. The callback will be called with Future.result == None.
        self.global_future = self.plan_srv.call_async(req_plan)
        self.global_future.add_done_callback(self.get_ref)
        self.get_logger().info("The global plan was requested...")
    
    def get_ref(self, future: Future):
        if future.result() is None:
            self.get_logger().info("The global plan request was cancelled.")
        self.get_logger().info("The global plan was received.")
        req_ref = PathFilter.Request()
        req_ref.path = future.result().path
        req_ref.tolerance = self.miss_tol
        while not self.ref_srv.wait_for_service(timeout_sec=5.0):
            self.get_logger().info(f'service {self.ref_srv.srv_name} not available, waiting...')
        if self.ref_future is not None and not self.ref_future.done():
            self.ref_future.cancel()  # what to do if a goal is requested before the previous completes. The callback will be called with Future.result == None.
        self.ref_future = self.ref_srv.call_async(req_ref)
        self.ref_future.add_done_callback(self.pass_ref)
        self.get_logger().info("The global ref was requested...")

    def pass_ref(self, future: Future):
        if future.result() is None:
            self.get_logger().info("The global ref request was cancelled.")
        self.get_logger().info("The global ref was received.")
        msn = RefSeq()
        for i in range(len(future.result().ctrl_sq)):
            s = SplineRef()
            s.path = future.result().ctrl_sq[i]
            s.speed = self.miss_spd
            msn.seq += [s]
        self.request_mission(msn)

    def request_mission(self, mission: RefSeq):
        action_msg = Drive.Goal()
        action_msg.path_reference = mission
        for i in range(len(action_msg.path_reference.seq)):
            dt = Spline(Spline_make.hermite_mtx, Spline_make.deserialize(action_msg.path_reference.seq[i].path.mtx_data,action_msg.path_reference.seq[i].path.ROWS,action_msg.path_reference.seq[i].path.cols)).len/action_msg.path_reference.seq[i].speed
            if i > 0:
                action_msg.path_reference.seq[i].start_time = action_msg.path_reference.seq[i-1].stop_time
            else:
                action_msg.path_reference.seq[i].start_time = self.get_clock().now().to_msg()
            nano, sec = modf(rclpy.time.Time.from_msg(action_msg.path_reference.seq[i].start_time).nanoseconds/1e9 + dt)
            action_msg.path_reference.seq[i].stop_time = rclpy.time.Time(seconds=int(sec),nanoseconds=int(nano * 1e9)).to_msg()
        self.ref_pub.publish(action_msg.path_reference)
        while not self.ref_act.wait_for_server(timeout_sec=5.0):
            self.get_logger().info(f'Drive server {self.ref_act._action_name} not available, waiting...')
        msg_future = self.ref_act.send_goal_async(action_msg)
        msg_future.add_done_callback(self.start_mission)
        self.get_logger().info("The mission was requested...")
    
    def start_mission(self, future: Future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Mission rejected.')
            self.ref_pub.publish(RefSeq())
            return
        self.get_logger().info('Mission accepted.')
        act_future = goal_handle.get_result_async()
        act_future.add_done_callback(self.end_mission)

    def end_mission(self, future: Future):
        if future.result().result.complete:
            self.get_logger().info('Mission SUCCESS.')
        else:
            self.get_logger().info('Mission FAILED.')
        self.ref_pub.publish(RefSeq())


def main(args=None):
    rclpy.init(args=args)
    node = Voyager()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("")
        sys.exit(0)