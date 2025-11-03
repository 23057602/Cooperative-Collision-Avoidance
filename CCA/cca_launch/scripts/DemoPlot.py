#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
import numpy as np
from matplotlib import pyplot as plt
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from cca_interfaces.msg import RefSeq, DriveForecast, OverRefSeq
from cca_utilities.spline_works import Spline, Spline_make

class DemoPlot(Node):

    def __init__(self):
        super().__init__("Demo_Plot")
        self.get_logger().info("The demo plot node has started. Initialising...")
        plt.ion()
        plt.show()
        self.tim = self.create_timer(0.1, self.plotting)
        self.subs = dict()
        self.plots = dict()
        self.declare_parameter('agent_count', 0, ParameterDescriptor(description='The number of agent plans accessible for monitoring.'))
        i = self.get_parameter('agent_count').get_parameter_value().integer_value
        for i in range(i):
            self.subs['plan_' + str(i)] = self.create_subscription(RefSeq,'agent_' + str(i) + '/global_plan', lambda msg, c=i: self.plots.update({'Global plan reference ' + str(c): {'color': 'green', 'data' : self.getRefData(msg), 'fmt' : '--', 'ms' : 0, 'lw': 2}}) if len(msg.seq) != 0 else None,10)
            self.subs['loc_plan_' + str(i)] = self.create_subscription(OverRefSeq,'agent_' + str(i) + '/local_plan', lambda msg, c=i: self.plots.update({'Local plan path reference ' + str(c): {'color': 'purple', 'data' : self.getRefData(msg.ref_stack), 'fmt' : '--', 'ms' : 0, 'lw': 2}}) if len(msg.ref_stack.seq) != 0 else None,10)
        self.pred_sub = self.create_subscription(DriveForecast,'predicted_trajectory', self.get_pred,10)
        self.map_sub = self.create_subscription(PointCloud2,'map', self.get_map,10)
        self.rdmp_sub = self.create_subscription(PointCloud2,'roadmap', self.get_roadmap,10)

    def plotting(self):
        xsx = [plt.xlim(),plt.ylim()]
        plt.cla()
        plt.xlabel('x (m)')
        plt.ylabel('y (m)')
        x = []
        y = []
        for label, graph in self.plots.items():
            plt.plot(graph['data'][:,0],graph['data'][:,1],graph['fmt'],color=graph['color'],label=label,ms=graph['ms'],linewidth=graph['lw'])
            if label != 'Roadmap vertices':
                x += graph['data'][:,0].tolist()
                y += graph['data'][:,1].tolist()
        if len(self.plots) > 1:
            plt.xlim(min(x) - 1.0,max(x) + 1.0)
            plt.ylim(min(y) - 1.0,max(y) + 1.0)
            tb = plt.gcf().canvas.manager.toolbar
            tb.update()
            tb.push_current()
            plt.xlim(xsx[0])
            plt.ylim(xsx[1])
            plt.legend()
            ax = plt.gca()
            ax.set_box_aspect((xsx[1][1]-xsx[1][0])/(xsx[0][1]-xsx[0][0]))
        plt.draw()
        plt.pause(0.05)
    
    def get_pred(self, pr: DriveForecast):
        pts = []
        for i in range(len(pr.trajectory)):
            pts += [[pr.trajectory[i].pose.pose.position.x, pr.trajectory[i].pose.pose.position.y]]
        self.plots['Trajectory prediction of ' + pr.vehicle_id] = {'color': 'red', 'data' : np.array(pts), 'fmt' : '-', 'ms' : 0, 'lw': 2}
        if ('Trajectory of ' + pr.vehicle_id) in self.plots:
            self.plots['Trajectory of ' + pr.vehicle_id]['data'] = np.array(self.plots['Trajectory of ' + pr.vehicle_id]['data'].tolist() + [pts[0]])
        else:
            self.plots['Trajectory of ' + pr.vehicle_id] = {'color': 'blue', 'data' : np.array([pts[0]]), 'fmt' : '-', 'ms' : 0, 'lw': 2}
        self.plots['Exclusion border of ' + pr.vehicle_id] = {'color': 'black', 'data' : (pr.conflict_radius * np.array([np.cos(np.arange(0,201*np.pi/100,np.pi/100)),np.sin(np.arange(0,201*np.pi/100,np.pi/100))]) + np.array([[pts[0][0]],[pts[0][1]]])).T, 'fmt' : '-', 'ms' : 0, 'lw': 2}
    
    def get_map(self, m: PointCloud2):
        return self.plots.update({'Map point cloud': {'color': 'grey', 'data' : np.array([list(p) for p in pc2.read_points(m,field_names=('x','y'),skip_nans=True)]), 'fmt' : 'o', 'ms' : 1, 'lw': 5}}) if m.width * m.height != 0 else None
    
    def get_roadmap(self, m: PointCloud2):  
        return self.plots.update({'Roadmap vertices':{'color': 'orange', 'data' : np.array([list(p) for p in pc2.read_points(m,field_names=('x','y'),skip_nans=True)]), 'fmt' : 'o', 'ms' : 2, 'lw': 2}}) if m.width * m.height != 0 else None
    
    def clearMyPlot(self):
        for k in [e for e in self.plots.keys()]:
            if (k != 'Map point cloud') and (k != 'Roadmap vertices'):
                self.plots.pop(k)
        
    def getRefData(self, r: RefSeq):
        ref_buf = np.array([])
        if len(r.seq) != 0:
            for i in range(len(r.seq)):
                curve = Spline(Spline_make.hermite_mtx,Spline_make.deserialize(r.seq[i].path.mtx_data,r.seq[i].path.ROWS,r.seq[i].path.cols))
                pts = []
                for j in range(1001):
                    pts += [curve(0.001*j)]
                ref_buf = np.array((ref_buf.tolist() + pts))
        return ref_buf

def main(args=None):
    rclpy.init(args=args)
    node = DemoPlot()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("")
        sys.exit(0)