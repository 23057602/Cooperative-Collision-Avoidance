#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.task import Future
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
import rclpy.time
from cca_interfaces.srv import GlobalPlan, PathFilter, StopPrediction
from cca_interfaces.msg import RefSeq, SplineRef
from cca_interfaces.action import Drive
from cca_utilities.spline_works import Spline, Spline_make
from math import modf, sin, cos, atan2

class Voyager(Node):

    def __init__(self):
        super().__init__("Voyager")
        self.get_logger().info("The virtual voyager has started. Initialising...")
        # Get typical voyager parameters
        self.declare_parameter('vehicle_mission_speed', 1.0, ParameterDescriptor(description='Vehicle path traversal speed.'))
        self.declare_parameter('vehicle_mission_reference_tolerance', -1.0, ParameterDescriptor(description='Maximum allowable deviation of spline reference from global plan points.'))
        self.declare_parameter('map_height', 0.0, ParameterDescriptor(description='height of relevant obstacles.'))
        self.miss_spd = self.get_parameter('vehicle_mission_speed').get_parameter_value().double_value
        self.miss_tol = self.get_parameter('vehicle_mission_reference_tolerance').get_parameter_value().double_value
        self.map_z = self.get_parameter('map_height').get_parameter_value().double_value
        # Get virtualizing parameters
        self.declare_parameter('fixed_frame', 'map', ParameterDescriptor(description='Fixed frame tf.'))
        self.declare_parameter('robot_frame', 'base_link', ParameterDescriptor(description='robot frame tf.'))
        self.declare_parameter('controller_frequency', 1.0, ParameterDescriptor(description='controller frequency.'))
        self.fixed_frame = self.get_parameter('fixed_frame').get_parameter_value().string_value
        self.robot_frame = self.get_parameter('robot_frame').get_parameter_value().string_value
        self.step_size = 1.0/self.get_parameter('controller_frequency').get_parameter_value().double_value
        # Subscriptions
        self.goal_sub = self.create_subscription(PoseStamped,'goal_pose',self.get_goal,10)
        self.pose_sub = self.create_subscription(Odometry,'initial_pose',self.get_pose,10)
        self.ref_sub = self.create_subscription(RefSeq,'mission', self.request_mission,10)
        self.twt_sub = self.create_subscription(Twist,'cmd_vel',self.get_cmd,10)
        # Services
        self.plan_srv = self.create_client(GlobalPlan, 'Global/get_plan')
        self.ref_srv = self.create_client(PathFilter, 'Path_filter/filter')
        self.pred_srv = self.create_client(StopPrediction, 'toggle_publishing')
        # Publishers
        self.ref_pub = self.create_publisher(RefSeq,'plan',10)
        self.odo_pub = self.create_publisher(Odometry,'pose',10)
        # Action Server to controller
        self.ref_act = ActionClient(self, Drive, 'Controller_Server/Go_to_waypoint')
        # Global parameters
        self.start = Odometry()
        self.start.header.frame_id = self.fixed_frame
        self.start.child_frame_id = self.robot_frame
        self.global_future: Future = None
        self.ref_future: Future = None

    # Kinematic forwarding
    def get_pose(self, odom_msg: Odometry):
        self.start = odom_msg
    
    def get_cmd(self, twt: Twist):
        yaw_meas = atan2(2 * (self.start.pose.pose.orientation.w * self.start.pose.pose.orientation.z + self.start.pose.pose.orientation.x * self.start.pose.pose.orientation.y), 1 - 2 * (self.start.pose.pose.orientation.y ** 2 + self.start.pose.pose.orientation.z ** 2))
        yaw_meas += twt.angular.z * self.step_size
        self.start.header.stamp = self.get_clock().now().to_msg()
        self.start.pose.pose.position.x += (twt.linear.x * self.step_size * cos(yaw_meas))
        self.start.pose.pose.position.y += (twt.linear.x * self.step_size * sin(yaw_meas))
        self.start.pose.pose.orientation.w = cos(0.5 * yaw_meas)
        self.start.pose.pose.orientation.z = sin(0.5 * yaw_meas)
        self.odo_pub.publish(self.start)

    # Global Planning
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

    # En Route/Mission start
    def request_mission(self, mission: RefSeq):
        action_msg = Drive.Goal()
        action_msg.path_reference = mission
        if len(action_msg.path_reference.seq) == 0:
            self.get_logger().info("The mission was empty.")
            return
        for i in range(len(action_msg.path_reference.seq)):
            dt = Spline(Spline_make.hermite_mtx, Spline_make.deserialize(action_msg.path_reference.seq[i].path.mtx_data,action_msg.path_reference.seq[i].path.ROWS,action_msg.path_reference.seq[i].path.cols)).len/action_msg.path_reference.seq[i].speed
            if i > 0:
                action_msg.path_reference.seq[i].start_time = action_msg.path_reference.seq[i-1].stop_time
            else:
                action_msg.path_reference.seq[i].start_time = self.get_clock().now().to_msg()
            nano, sec = modf(rclpy.time.Time.from_msg(action_msg.path_reference.seq[i].start_time).nanoseconds/1e9 + dt)
            action_msg.path_reference.seq[i].stop_time = rclpy.time.Time(seconds=int(sec),nanoseconds=int(nano * 1e9)).to_msg()
        self.ref_pub.publish(action_msg.path_reference)
        while not self.pred_srv.wait_for_service(timeout_sec=5.0):
            self.get_logger().info(f'Prediction toggle server {self.pred_srv.srv_name} not available, waiting...')
        while not self.ref_act.wait_for_server(timeout_sec=5.0):
            self.get_logger().info(f'Drive server {self.ref_act._action_name} not available, waiting...')
        self.get_cmd(Twist())
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