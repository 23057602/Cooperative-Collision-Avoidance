#!/usr/bin/env python3
import sys
from numpy import pi, sign, clip, cos, arccos, sin, arctan, arctan2, sum, tan
from math import isnan

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse,CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rcl_interfaces.msg import ParameterDescriptor
from tf2_ros import TransformStamped

import rclpy.time
import time
from cca_interfaces.action import Drive
from cca_interfaces.msg import DriveState, OverRefSeq
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from cca_utilities.spline_works import Spline, Spline_make

class Conductor(Node):

    def __init__(self):
        #node init
        super().__init__("Conductor_Server")
        self.get_logger().info("The conductor has started...")
        self.declare_parameter('controller_frequency', 20.0, ParameterDescriptor(description='Controller frequency.'))
        self.declare_parameter('vehicle_base', 1.0, ParameterDescriptor(description='Distance between the front and back wheels.'))
        self.declare_parameter('vehicle_steering_limit', pi/3, ParameterDescriptor(description='Maximum steering angle in radians.'))
        self.declare_parameter('controller_error_gain', 0.2, ParameterDescriptor(description='Stanley controller cross-track sensitivity gain.'))
        self.declare_parameter('controller_speed_gain', 0.01, ParameterDescriptor(description='Stanley controller speed sensitivity gain.'))
        self.declare_parameter('controller_follow_gain', 1.0, ParameterDescriptor(description='In/Along-track controller reference lead/lag sensitivity gain.'))
        self.declare_parameter('evasive_fast_speed', 1.0, ParameterDescriptor(description='In/Along-track controller maximum catch up speed.'))
        self.declare_parameter('evasive_slow_speed', 0.01, ParameterDescriptor(description='In/Along-track controller minimum lag speed.'))
        self.frequency = self.get_parameter('controller_frequency').get_parameter_value().double_value
        self.wheel_base = self.get_parameter('vehicle_base').get_parameter_value().double_value
        self.turn_clip = self.get_parameter('vehicle_steering_limit').get_parameter_value().double_value
        self.ctrl_K = self.get_parameter('controller_error_gain').get_parameter_value().double_value
        self.ctrl_Kslow = self.get_parameter('controller_speed_gain').get_parameter_value().double_value
        self.ctrl_Ks = self.get_parameter('controller_follow_gain').get_parameter_value().double_value
        self.ctrl_v_hi = self.get_parameter('evasive_fast_speed').get_parameter_value().double_value
        self.ctrl_v_lo = self.get_parameter('evasive_slow_speed').get_parameter_value().double_value
        self.server = ActionServer(self, Drive,"Controller_Server/Go_to_waypoint",goal_callback=self.receiveGoal,execute_callback=self.actionMonitor, cancel_callback=self.cancelGoal, callback_group=ReentrantCallbackGroup())
        self.current_position_stream = self.create_subscription(Odometry,"pose", self.read_current_position, 10)
        self.override_stream = self.create_subscription(OverRefSeq,"ovr", self.get_local_plan, 10)
        self.control_command_stream = self.create_publisher(Twist, "cmd_vel", 10)
        self.control_state_stream = self.create_publisher(DriveState, "ctrl_st", 10) #for real-time visibility to ros bag or external logging
        self.timer = self.create_timer(1.0/self.frequency, self.stanley)
        self.timer.cancel()
        self.state_log = []
        #measurement init
        self.pose = Pose()
        self.fixed = ""
        self.base = ""
        #path init
        self.prog = 0.0
        self.g_ref_ptr = [-1]
        self.l_ref_ptr = [-1]
        self.g_ref_list = []
        self.l_ref_list = []

    def receiveGoal(self, Goal: ServerGoalHandle):
        #check availability
        if(self.g_ref_ptr[0] >= 0):
            self.get_logger().error("A goal was rejected. Server busy.")
            return GoalResponse.REJECT
        if(len(Goal.path_reference.seq) < 1):
            self.get_logger().error("At least 1 path and speed needed needed.")
            return GoalResponse.REJECT
        #Initialize variables
        self.g_ref_list = [[Spline(Spline_make.hermite_mtx, Spline_make.deserialize(Goal.path_reference.seq[i].path.mtx_data,Goal.path_reference.seq[i].path.ROWS,Goal.path_reference.seq[i].path.cols)), Goal.path_reference.seq[i].speed, Goal.path_reference.seq[i].start_time, Goal.path_reference.seq[i].stop_time] for i in range(len(Goal.path_reference.seq))][::-1]
        self.g_ref_ptr[0] = len(self.g_ref_list) - 1
        self.prog = 0.0
        self.state_log = []
        self.get_logger().info("A goal was accepted.")
        self.timer.reset()
        return GoalResponse.ACCEPT

    def cancelGoal(self, goal_handle: ServerGoalHandle):
        #Log cancellation
        self.g_ref_ptr[0] = -2
        self.get_logger().info('Received goal was canceled.')
        return CancelResponse.ACCEPT

    def actionMonitor(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Beginning departure.")
        fdbk = Drive.Feedback()
        while(self.g_ref_ptr[0] > -1):
            fdbk.progress = self.prog
            if (fdbk.ref_index != self.g_ref_ptr[0]) or (fdbk.local_index != self.l_ref_ptr[0]):
                fdbk.ref_index = self.g_ref_ptr[0]
                fdbk.local_index = self.l_ref_ptr[0]
                goal_handle.publish_feedback(fdbk)
        if not self.timer.is_canceled():
            self.timer.cancel() #stop stanley controller callback
        goal_handle.succeed()
        result = Drive.Result()
        result.complete = self.g_ref_ptr[0] == -1
        result.exec_log = self.state_log
        if result.complete:
            self.get_logger().info("An arrival notification was sent.")
        else:
            self.get_logger().info("An error notification was sent.")
        self.state_log = []
        return result
    
    def read_current_position(self, Odom: Odometry):
        self.pose = Odom.pose.pose
        self.fixed = Odom.header.frame_id
        self.base = Odom.child_frame_id
    
    def get_local_plan(self, plan: OverRefSeq):
        if(len(plan.ref_stack.seq) < 1):
            self.get_logger().error("Local planner abort condition received. Halting.")
            self.g_ref_ptr[0] = -2
            return
        #Initialize variables
        self.get_logger().info("Local planner override received. Restacking...")
        self.l_ref_list = [[Spline(Spline_make.hermite_mtx, Spline_make.deserialize(plan.ref_stack.seq[i].path.mtx_data,plan.ref_stack.seq[i].path.ROWS,plan.ref_stack.seq[i].path.cols)),plan.ref_stack.seq[i].speed,plan.ref_stack.seq[i].start_time,plan.ref_stack.seq[i].stop_time] for i in range(len(plan.ref_stack.seq))][::-1]
        self.l_ref_ptr = [len(self.l_ref_list) - 1]
        self.get_logger().info("Running override...")

    def stanley(self):
        if(self.g_ref_ptr[0] < 0):
            return
        
        ref_ptr = self.l_ref_ptr
        ref_list = self.l_ref_list
        if(self.l_ref_ptr[0] < 0):
            ref_ptr = self.g_ref_ptr
            ref_list = self.g_ref_list

        x_meas = self.pose.position.x
        y_meas = self.pose.position.y
        yaw_meas = arctan2(2 * (self.pose.orientation.w * self.pose.orientation.z + self.pose.orientation.x * self.pose.orientation.y), 1 - 2 * (self.pose.orientation.y ** 2 + self.pose.orientation.z ** 2))

        #compute state variables
        first = 0
        last = len(self.g_ref_list)
        while(first+1 < last):
            if(self.get_clock().now().nanoseconds >= rclpy.time.Time.from_msg(self.g_ref_list[int((first + last)/2)][2]).nanoseconds):
                first = (first + last)/2
            else:
                last = (first + last)/2
        c_ref_ptr = int((first + last)/2)
        
        ref_len = self.g_ref_list[c_ref_ptr][0].len * clip((self.get_clock().now().nanoseconds - rclpy.time.Time.from_msg(self.g_ref_list[c_ref_ptr][2]).nanoseconds)/(rclpy.time.Time.from_msg(self.g_ref_list[c_ref_ptr][3]).nanoseconds - rclpy.time.Time.from_msg(self.g_ref_list[c_ref_ptr][2]).nanoseconds), 0.0, 1.0)
        tg = self.g_ref_list[self.g_ref_ptr[0]][0].getClosestT([x_meas, y_meas])
        curr_len = self.g_ref_list[self.g_ref_ptr[0]][0].getLength(tg)
        cross_len = (2*(c_ref_ptr > self.g_ref_ptr[0]) - 1) * sum([r[0].len for r in self.g_ref_list[min(c_ref_ptr,self.g_ref_ptr[0]):max(c_ref_ptr,self.g_ref_ptr[0])]])
        v = clip(ref_list[ref_ptr[0]][1] + self.ctrl_Ks*(ref_len - curr_len + cross_len), self.ctrl_v_lo, self.ctrl_v_hi)#in-track control law
        
        t = ref_list[ref_ptr[0]][0].getClosestT([x_meas, y_meas])
        track_pos = ref_list[ref_ptr[0]][0](t)
        track_tan = ref_list[ref_ptr[0]][0].getTangent(t)
        track_heading = arctan2(track_tan[1], track_tan[0])
        ctrl_Herr = sign(cos(yaw_meas)*sin(track_heading) - sin(yaw_meas)*cos(track_heading))*arccos(cos(yaw_meas)*cos(track_heading) + sin(yaw_meas)*sin(track_heading))
        ctrl_CTerr = (x_meas - track_pos[0])*sin(track_heading) - (y_meas - track_pos[1])*cos(track_heading)
        ctrl_Lw = clip(ctrl_Herr + arctan(self.ctrl_K*ctrl_CTerr/(v + self.ctrl_Kslow)),-1*self.turn_clip,self.turn_clip)#cross track control law
        
        #Publish
        vhl_cmd = Twist()
        if(tg >= 1.0):
            self.get_logger().info("Global reference path " + str(self.g_ref_ptr[0]) + " has been completed.")
            self.g_ref_ptr[0]-=1
        if(t >= 1.0):#check if reference complete
            if(ref_ptr is self.l_ref_ptr):
                ref_ptr[0] -= 1
            self.get_logger().info("Loading reference path " + str(self.g_ref_ptr[0]) + "/" + str(self.l_ref_ptr[0]) + " of " + str(len(self.g_ref_list) - 1 ) + "/" + str(len(self.l_ref_list) - 1) + "...")
            if(self.g_ref_ptr[0] > -1):
                return
        else: #continue reference
            vhl_cmd.linear.x = v
            vhl_cmd.angular.z = v*tan(ctrl_Lw)/self.wheel_base #steering to angular rate convertion
        self.control_command_stream.publish(vhl_cmd)
        
        #record data
        self.prog = t
        state = DriveState()
        state.header.frame_id = self.fixed
        state.header.stamp = self.get_clock().now().to_msg()
        state.child_frame_id = self.base
        state.heading_error = ctrl_Herr
        state.cross_track_error = ctrl_CTerr
        state.in_track_error = ref_len - curr_len + cross_len
        state.pose = self.pose
        state.cmd_vel = vhl_cmd
        #state.path = ref_list[ref_ptr[0]][0]
        self.control_state_stream.publish(state)

def main(args=None):
    rclpy.init(args=args)
    node = Conductor()
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