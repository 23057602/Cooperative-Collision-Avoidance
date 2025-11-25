#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from cca_interfaces.srv import PathFilter
from cca_interfaces.msg import CubicSpline
from cca_utilities.spline_works import Spline_make, Spline

class Path_filter(Node):

    def __init__(self):
        super().__init__("Path_filter")
        self.get_logger().info("The Path Filter has started. Initialising...")
        self.srv = self.create_service(PathFilter, 'Path_filter/filter', self.filter)

    def filter(self, request: PathFilter.Request, response: PathFilter.Response):
        self.get_logger().info("A " + str(len(request.path)) + " point request was received.")
        if len(request.path) < 4 :
            self.get_logger().error("Rejected: insufficient data (4+ req).")
            return response
        
        self.get_logger().info("Filtering...")
        points_splined = 0
        tol = request.tolerance
        if request.tolerance <= 0.0:
            tol = float('inf')
        point_column = [[p.x, p.y, p.z] for p in request.path]
        while (points_splined + 1 < len(request.path)):
            points_to_spline = Spline_make.search_tol_arg(point_column,Spline_make.hermite_mtx,tol)
            s_ctrl = Spline_make.fromPoints(point_column[0:points_to_spline],Spline_make.hermite_mtx)
            current_path = Spline(Spline_make.hermite_mtx, s_ctrl) #find spline
            if (len(response.ctrl_sq) > 0):
                clip_start = Spline_make.deserialize(response.ctrl_sq[-1].mtx_data,response.ctrl_sq[-1].ROWS,response.ctrl_sq[-1].cols)[2] #previous end point
            else:
                clip_start = point_column[0] #current start point
            nxt_ctrl = Spline_make.herm_seg(s_ctrl,current_path.getClosestT(clip_start),current_path.getClosestT(point_column[points_to_spline-1])) #clip the ends
            msg = CubicSpline()
            msg.cols = len(nxt_ctrl[0])
            msg.mtx_data = Spline_make.serialize(nxt_ctrl)
            response.ctrl_sq += [msg]
            points_splined += (points_to_spline-1) #overlap points for alignment
            point_column = point_column[min(points_to_spline-1,len(point_column)-4):] #keep remaining points
        self.get_logger().info("Filtered.")
        return response

def main(args=None):
    rclpy.init(args=args)
    node = Path_filter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("")
        sys.exit(0)