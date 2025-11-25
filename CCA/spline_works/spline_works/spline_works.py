#!/usr/bin/env python3
import numpy as np
from numpy.polynomial import Polynomial
import scipy.integrate as integrate
from scipy.optimize import minimize

#https://stackoverflow.com/questions/2742610/closest-point-on-a-cubic-bezier-curve/57315396#57315396
class Spline:
    def __init__(self, matrix: list[list[float]], controls: list[list[float]]):
        self.C = np.array(matrix)
        self.M = np.array(controls)
        self._CM_f = [Polynomial(coeffs) for coeffs in (self.C @ self.M).T]
        self._dCM_dt_f = [p_i.deriv() for p_i in self._CM_f]
        self._norm_v_sq = np.sum([dp_i**2 for dp_i in self._dCM_dt_f])
        self.len = self.getLength(1.0)
    
    def __call__(self, t) -> list[float]:
        return np.array([p_i(t) for p_i in self._CM_f])
    
    def getDistFunc(self, point: list[float]):
        return np.sum([(pi_xi[0] - pi_xi[1])**2 for pi_xi in zip(self._CM_f,point)])
    
    def getClosestT(self, point: list[float]) -> float:
        d = self.getDistFunc(point)
        d2d_dt2 = d.deriv(2)
        solutions = [r.real for r in d.deriv().roots() if ((r.imag == 0.0) and (d2d_dt2(r) >= 0) and (r.real > 0.0) and (r.real < 1.0))] + [0.0, 1.0]
        return solutions[np.argmin([float(d(s)) for s in solutions])]
    
    def getTangent(self, t) -> list[float]:
        return np.array([dp_i(t) for dp_i in self._dCM_dt_f]).tolist()
    
    def getLength(self, t) -> float:
        return integrate.fixed_quad(lambda x: np.sqrt(self._norm_v_sq(x)), 0.0, t)[0]

#this class manages multiple sequentially specified splines as one curve with whole number knot values
class Splines:
    def __init__(self, spline_list: list[Spline]):
        self.curves = spline_list
    
    def __call__(self, t: float):
        return self.curves[min(int(t), len(self.curves) - 1)](t - min(int(t), len(self.curves) - 1))
    
    def getTangent(self, t: float):
        return self.curves[min(int(t), len(self.curves) - 1)].getTangent(t - min(int(t), len(self.curves) - 1))
    
    def getClosestT(self, point: list[float]) -> float:
        curveTs = [self.curves[i].getClosestT(point) + i for i in range(len(self.curves))]
        curveDs = [self.curves[min(int(t), len(self.curves) - 1)].getDistFunc(point)(t - min(int(t), len(self.curves) - 1)) for t in curveTs]
        return curveTs[np.argmin(curveDs)]
    
    def size(self):
        return len(self.curves)

#this class contains methods for creating control point matrices of common shapes from more intuitive parameters
class Spline_make:
    hermite_mtx = [[1, 0, 0, 0], [0, 1, 0, 0], [-3, -2, 3, -1], [2, 1, -2, 1]] #hermite characteristic matrix for control points [p0, tan0 ,p1 ,tan1]
    
    def herm_line(p0: list[float], p1: list[float])->list[list[float]]:
        v = np.array(p1) - np.array(p0)
        return [p0, v.tolist(), p1, v.tolist()]
    
    def herm_arc2d(pos: list[float], yaw: float, radius: float, angle: float, quad: list[float] = [1, 1])->list[list[float]]:
        #https://stackoverflow.com/questions/30277646/svg-convert-arcs-to-cubic-bezier and
        #https://pomax.github.io/bezierinfo/#circles_cubic
        k = 4*radius*np.tan(angle/4.0)
        p1 = radius*np.array([quad[0]*(np.sin(angle + yaw) - np.sin(yaw)), quad[1]*(np.cos(yaw) - np.cos(angle + yaw))]) + np.array(pos)
        return [pos, [quad[0]*k*np.cos(yaw), quad[1]*k*np.sin(yaw)], p1.tolist(), [quad[0]*k*np.cos(angle + yaw), quad[1]*k*np.sin(angle + yaw)]]
    
    def _fromPointsLSE(point_col: list[list[float]], sp_char_mtx: list[list[float]]):
        T = np.array([[1, val, val**2, val**3] for val in np.array([*range(0,len(point_col))])/(len(point_col)-1)])
        return (np.linalg.inv(np.array(sp_char_mtx)) @ np.linalg.inv(T.T @ T) @ T.T @ np.array(point_col)).tolist()

    def _D_sq(param_Vec, point_Mtx):
        P = [Polynomial(coeffs) for coeffs in np.reshape(np.array(param_Vec[len(point_Mtx):]),(4,int((len(param_Vec)-len(point_Mtx))/4)), order='F').T]
        D2 = np.sum(np.square([P_j[0](np.array(param_Vec[:len(point_Mtx)])) - P_j[1] for P_j in zip(P,np.array(point_Mtx).T)]))
        return D2

    def _D_sq_Jac(param_Vec, point_Mtx):
        P = [Polynomial(coeffs) for coeffs in np.reshape(np.array(param_Vec[len(point_Mtx):]),(4,int((len(param_Vec)-len(point_Mtx))/4)), order='F').T]
        dD_dt = np.sum([2 * P_j[0].deriv()(np.array(param_Vec[:len(point_Mtx)])) * (P_j[0](np.array(param_Vec[:len(point_Mtx)])) - P_j[1]) for P_j in zip(P,np.array(point_Mtx).T)],axis=0)
        dD_da = np.reshape([[2 * np.sum(dx2p * np.power(np.array(param_Vec[:len(point_Mtx)]),k)) for k in range(4)] for dx2p in np.array([P_j[0](np.array(param_Vec[:len(point_Mtx)])) - P_j[1] for P_j in zip(P,np.array(point_Mtx).T)])],-1)
        dD_dX = dD_dt.tolist() + dD_da.tolist()
        return dD_dX
    
    # Finds the control points of the spline best fitted to the points. Available methods: LSE (evenly spread in parameter space), BFGS (good for fitting up to 300 points), SLSQP (good for fitting up to 200 points).
    def fromPoints(point_col: list[list[float]], sp_char_mtx: list[list[float]], method=None, tol=1e-9):
        init_est = Spline_make._fromPointsLSE(point_col,sp_char_mtx)
        X = [i/(len(point_col)-1) for i in range(len(point_col))] + np.reshape(np.array(sp_char_mtx) @ np.array(init_est), -1, order='F').tolist()
        if method == 'BFGS':
            res = minimize(Spline_make._D_sq, X, args=(point_col), method='BFGS', jac=Spline_make._D_sq_Jac, options={'disp': False})
            return (np.linalg.inv(np.array(sp_char_mtx)) @ np.reshape(np.array(res.x[len(point_col):]),(4,int((len(res.x)-len(point_col))/4)), order='F')).tolist()
        elif method == 'SLSQP':
            cons_ieq = {'type': 'ineq', 'fun' : lambda x, pts=len(point_col): np.array([x[i] - x[i-1] for i in range(1,pts)] + [x[i] * (1.0 - x[i]) for i in range(0,pts)])}
            res = minimize(Spline_make._D_sq, X, args=(point_col), method='SLSQP', jac=Spline_make._D_sq_Jac, constraints=[cons_ieq], options={'ftol': tol, 'disp': False})
            return (np.linalg.inv(np.array(sp_char_mtx)) @ np.reshape(np.array(res.x[len(point_col):]),(4,int((len(res.x)-len(point_col))/4)), order='F')).tolist()
        else:
            return init_est
    
    def point_error(pnt_lst: list[list[float]], spl):
        return [np.sqrt(np.sum(np.square(np.array(spl(spl.getClosestT(pnt))) - np.array(pnt)))) for pnt in pnt_lst]

    def search_tol_arg(pnt_lst: list[list[float]], sp_char_mtx: list[list[float]], tol: float):
        total_control = Spline_make.fromPoints(pnt_lst, sp_char_mtx)
        path = Spline(sp_char_mtx, total_control)
        if np.max(Spline_make.point_error(pnt_lst, path)) < tol:
            return len(pnt_lst)
        lower_bound = 0
        upper_bound = len(pnt_lst)
        last_success = 4 # minimum points allowed for a cubic spline
        while (upper_bound > lower_bound+1) and (int((upper_bound + lower_bound)/2)>4):
            segment_controls = Spline_make.fromPoints(pnt_lst[0:int((upper_bound + lower_bound)/2)], sp_char_mtx)
            segment = Spline(sp_char_mtx, segment_controls)
            if np.max(Spline_make.point_error(pnt_lst[0:int((upper_bound + lower_bound)/2)], segment)) < tol:
                last_success = int((upper_bound + lower_bound)/2)
                lower_bound = last_success
            else:
                upper_bound = int((upper_bound + lower_bound)/2)
        return last_success

    def herm_seg(c_ptx: list[list[float]], t0: float, t1: float)->list[list[float]]:
        poly = (np.array(Spline_make.hermite_mtx) @ np.array(c_ptx)).T.tolist()
        sec_c = []
        for p in poly:
            q_c = [p[0] + p[1]*t0 + p[2]*t0*t0 + p[3]*t0*t0*t0]
            q_c += [p[1]*(t1-t0) + 2.0*p[2]*t0*(t1-t0) + 3.0*p[3]*(t1-t0)*t0*t0]
            q_c += [p[3]*(t1-t0)*(t1-t0)*(t1-t0) + p[2]*(t1-t0)*(t1-t0) + 3.0*p[3]*(t1-t0)*(t1-t0)*t0 + q_c[0] + q_c[1]]
            q_c += [p[3]*(t1-t0)*(t1-t0)*(t1-t0) - 2.0*q_c[0] - q_c[1] + 2.0*q_c[2]]
            sec_c += [q_c]
        return np.array(sec_c).T.tolist()
    
    def serialize(m: list[list[float]])->list[float]:
        return np.reshape(np.array(m), -1, order='F').tolist()
    
    def deserialize(v: list[float], rows: int, cols: int)->list[list[float]]:
        return np.reshape(np.array(v),(rows,cols), order='F').tolist()

if __name__ == "__main__":
    print("Spline Works")
