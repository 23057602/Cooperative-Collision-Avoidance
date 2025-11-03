#pragma once
#include <Eigen/Dense>
#include <utility>
#include <vector>
#include <cmath>
#include <gsl/gsl_roots.h>

class Spline
{
    private:
        Eigen::Matrix4d C;
        Eigen::Matrix4Xd M;
        Eigen::Matrix4Xd C_M;
        Eigen::Matrix3Xd C_M_Prime;
        Eigen::VectorXd sq_norm_v_data;
        gsl_function norm_v;
        double len;
        
        static Eigen::MatrixXd colDiff(Eigen::MatrixXd columns0);
        static Eigen::RowVectorXd polyEval(double t, Eigen::MatrixXd poly);
        static double norm_v_f(double x,void * p);
    public:
        Spline(Eigen::Matrix4d characteristic, Eigen::Matrix4Xd control);
        ~Spline();
        Eigen::RowVectorXd operator()(double t);
        std::size_t size();
        Eigen::VectorXd getDistFunc(Eigen::RowVectorXd point);
        double getClosestT(Eigen::RowVectorXd point);
        Eigen::RowVectorXd getTangent(double t);
        double getLength(double t, std::size_t limit = 50, double err = 1.49e-8, int order = 5);
        double getLength();
        Eigen::Matrix4Xd getControl();
};

class Splines
{
    private:
        std::vector<Spline> curve_list;
    public:
        Splines(std::vector<Spline> list);
        ~Splines();
        Eigen::RowVectorXd operator()(double t);
        std::size_t size();
        double getClosestT(Eigen::RowVectorXd point);
        Eigen::RowVectorXd getTangent(double t);
        Eigen::RowVectorXd polyEval(double t, Eigen::VectorXd poly);
};

namespace Spline_make
{
    extern Eigen::Matrix4d hermite_mtx;
    Eigen::Matrix4Xd herm_line(Eigen::RowVectorXd start, Eigen::RowVectorXd end);
    Eigen::Matrix<double, 4, 2> herm_arc2d(Eigen::RowVector2d start, double yaw, double radius, double angle);
    Eigen::Matrix4Xd fromPoints(Eigen::MatrixXd point_column, Eigen::Matrix4d char_mtx);
    Eigen::VectorXd point_error(Eigen::MatrixXd point_column, Spline spl);
    std::size_t search_tol_arg(Eigen::MatrixXd point_column, Eigen::Matrix4d char_mtx, double tol);
    Eigen::Matrix4Xd herm_seg(Eigen::Matrix4Xd c_ptx, double t0, double t1);
    std::vector<double> serialize(Eigen::MatrixXd m);
    Eigen::MatrixXd deserialize(std::vector<double> v, Eigen::Index rows, Eigen::Index cols);
};
