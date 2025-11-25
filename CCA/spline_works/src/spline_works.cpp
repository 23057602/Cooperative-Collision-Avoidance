#include <Eigen/Dense>
#include <utility>
#include <vector>
#include <list>
#include <cmath>
#include <stdexcept>
#include <spline_works/spline_works.hpp>
#include <iostream>
#include <ctime>
#include <gsl/gsl_poly.h>
#include <gsl/gsl_integration.h>
#include <complex>

Spline::Spline(Eigen::Matrix4d characteristic, Eigen::Matrix4Xd control): C(characteristic), M(control){
    this->C_M = characteristic * control;//get polynomial coefficients
    this->C_M_Prime = Spline::colDiff(characteristic * control);
    this->sq_norm_v_data = Eigen::VectorXd({{0.0,0.0,0.0,0.0,0.0}});
    for(long int i = 0; i < this->C_M_Prime.cols(); i++){
        for(long int j = 0; j < 3; j++){
            for(long int k = 0; k < 3; k++){this->sq_norm_v_data(j+k) += this->C_M_Prime(j,i) * this->C_M_Prime(k,i);}
        }
    }
    this->norm_v.params = this->sq_norm_v_data.data();
    this->norm_v.function = Spline::norm_v_f;
    this->len = this->getLength(1.0);
}

Spline::~Spline(){}

Eigen::RowVectorXd Spline::operator()(double t)//This function returns the row vector representing the position of the curve at t.
{return Spline::polyEval(t, this->C_M);}

std::size_t Spline::size(){return 1;}

Eigen::VectorXd Spline::getDistFunc(Eigen::RowVectorXd point)//This function returns the coefficients of the polynomial that give the squared distance from the point passed.
{
    this->C_M.row(0) -= point;//shift curve
    Eigen::VectorXd d_c {{0.0,0.0,0.0,0.0,0.0,0.0,0.0}};
    for(long int i = 0; i < this->C_M.cols(); i++){
        for(long int j = 0; j < 4; j++){
            for(long int k = 0; k < 4; k++){d_c(j+k) += this->C_M(j,i) * this->C_M(k,i);}
        }
    }
    this->C_M.row(0) += point;//shift curve
    return d_c;
}

Eigen::MatrixXd Spline::colDiff(Eigen::MatrixXd columns0)//This function implements the power rule of differentiation on the columns passed.
{
    for (long int i = 2; i < columns0.rows(); i++){columns0.row(i) *= i;}
    return columns0(Eigen::seq(1, Eigen::last), Eigen::all);
}

Eigen::RowVectorXd Spline::polyEval(double t, Eigen::MatrixXd poly)//This function returns a row vector representing the values of the column-wise polynomials at t.
{
    for(long int i = 0; i < poly.cols(); i++){poly(0,i) = gsl_poly_eval(poly.data() + i*poly.rows(),poly.rows(),t);}
    return poly.row(0).eval();
}

double Spline::norm_v_f(double x, void *p){return std::sqrt(std::max(gsl_poly_eval((double *) p, 5, x),0.0));}

double Spline::getClosestT(Eigen::RowVectorXd point)
{
    Eigen::VectorXd D = this->getDistFunc(point);
    Eigen::VectorXd D_rate {{D(1),2*D(2),3*D(3),4*D(4),5*D(5),6*D(6)}};
    Eigen::VectorXd D_curve {{D_rate(1),2*D_rate(2),3*D_rate(3),4*D_rate(4),5*D_rate(5)}};
    int i = D_rate.rows();
    while(D_rate(i-1) == 0.0){i-=1;}
    
    //root solving
    double c_pts[16];//[real,img,real,img,...]buffer of complex critical points
    gsl_poly_complex_workspace * w = gsl_poly_complex_workspace_alloc(i);
    gsl_poly_complex_solve(D_rate.data(), i, w, c_pts);
    gsl_poly_complex_workspace_free(w);

    //find optimal
    std::list<std::pair<double, double>> solution_candidates({std::make_pair(D.sum(), 1.0), std::make_pair(D(0), 0.0)});//init with endpoints
    for (int i = 0; i < D_rate.rows()-1; i++){
        if (c_pts[2*i+1] == 0.0){
            if ((c_pts[2*i] > 0.0) && (c_pts[2*i] < 1.0)){
                if (Spline::polyEval(c_pts[2*i], D_curve)(0) > 0.0){
                    solution_candidates.push_back(std::make_pair(Spline::polyEval(c_pts[2*i], D)(0), c_pts[2*i]));
                }
            }
        }
    }
    solution_candidates.sort();
    return solution_candidates.front().second;
}

Eigen::RowVectorXd Spline::getTangent(double t)
{return Spline::polyEval(t, this->C_M_Prime);}

double Spline::getLength(double t, std::size_t limit, double err, int order)
{
    double l;
    double e;
    this->norm_v.params = this->sq_norm_v_data.data();
    gsl_integration_workspace * w = gsl_integration_workspace_alloc(limit);
    gsl_integration_qag(&(this->norm_v), 0.0, t, err, err, limit, order, w, &l, &e);
    gsl_integration_workspace_free(w);
    return l;
}

double Spline::getLength(){return this->len;}

Eigen::Matrix4Xd Spline::getControl()
{
    return this->M;
}

Splines::Splines(std::vector<Spline> list): curve_list(list){}

Splines::~Splines(){}

Eigen::RowVectorXd Splines::operator()(double t)
{return this->curve_list[std::min((std::size_t) t, this->curve_list.size() - 1)](t - std::min((std::size_t) t, this->curve_list.size() - 1));}

std::size_t Splines::size()
{return this->curve_list.size();}

double Splines::getClosestT(Eigen::RowVectorXd point)
{
    std::list<std::pair<double, double>> candidate_solutions;
    for (std::size_t i = 0; i < this->curve_list.size(); i++){
        auto dist_poly = this->curve_list[i].getDistFunc(point);
        auto candidate = this->getClosestT(point);
        candidate_solutions.push_back(std::make_pair(this->polyEval(candidate, dist_poly)(0,0), candidate + i));
    }
    candidate_solutions.sort();
    return candidate_solutions.front().second;
}

Eigen::RowVectorXd Splines::getTangent(double t)
{return this->curve_list[std::min((std::size_t) t, this->curve_list.size() - 1)].getTangent(t - std::min((std::size_t) t, this->curve_list.size() - 1));}

Eigen::RowVectorXd Splines::polyEval(double t, Eigen::VectorXd poly)//This function returns a row vector representing the values of the column-wise polynomials at t.
{return Eigen::pow(t, Eigen::ArrayXd::LinSpaced(poly.rows(), 0, poly.rows() - 1)).matrix().transpose() * poly;}

Eigen::Matrix4d Spline_make::hermite_mtx {{1, 0, 0, 0}, {0, 1, 0, 0}, {-3, -2, 3, -1}, {2, 1, -2, 1}};

Eigen::Matrix4Xd Spline_make::herm_line(Eigen::RowVectorXd start, Eigen::RowVectorXd end)
{
    Eigen::Matrix4Xd control(4, start.cols());
    Eigen::RowVectorXd tan = end - start;
    control.block(0, 0, 1, control.cols()) = start;
    control.block(1, 0, 1, control.cols()) = tan;
    control.block(2, 0, 1, control.cols()) = end;
    control.block(3, 0, 1, control.cols()) = tan;
    return control;
}

Eigen::Matrix<double, 4, 2> Spline_make::herm_arc2d(Eigen::RowVector2d start, double yaw, double radius, double angle)
{
    int sgn = (radius >= 0.0) - (radius < 0.0);
    radius *= sgn;
    double k = 4*radius*std::tan(angle/4);
    Eigen::Matrix<double, 4, 2> control = Eigen::Matrix<double, 4, 2>();
    control.block(0, 0, 1, 2) = start;
    control(1, 0) = k * std::cos(yaw);
    control(1, 1) = k * std::sin(yaw);
    control(2, 0) = radius * (std::sin(angle + sgn * yaw) - sgn * std::sin(yaw)) + start(0, 0);
    control(2, 1) = radius * (-1 * sgn * std::cos(angle + sgn * yaw) + sgn * std::cos(yaw)) + start(0, 1);
    control(3, 0) = k * std::cos(yaw + sgn * angle);
    control(3, 1) = k * std::sin(yaw + sgn * angle);
    return control;
}

Eigen::Matrix4Xd Spline_make::fromPoints(Eigen::MatrixXd point_column, Eigen::Matrix4d char_mtx)
{
    if(point_column.rows() < 4){throw std::invalid_argument( "Insufficient data, at least 4 required." );}
    
    Eigen::MatrixXd T(point_column.rows(), 4);
    for (Eigen::Index i = 0; i < point_column.rows(); i++){//sets rows as [1, val, val**2, val**3]
        T(i,0) = 1;
        T(i,1) = ((double) i)/(point_column.rows()-1);
        T(i,2) = T(i,1) * T(i,1);
        T(i,3) = T(i,2) * T(i,1);
    }
    
    bool invertible;
    Eigen::Matrix4d char_mtx_inv;
    char_mtx.computeInverseWithCheck(char_mtx_inv, invertible);
    if(!invertible){throw std::invalid_argument( "char_mtx is not invertible." );}

    Eigen::Matrix4d T_t_T_inv;
    Eigen::Matrix4d(T.transpose() * T).computeInverseWithCheck(T_t_T_inv, invertible);
    if(!invertible){throw std::invalid_argument( "Data rank is less than 4." );}

    return char_mtx_inv * T_t_T_inv * T.transpose() * point_column;
}

Eigen::VectorXd Spline_make::point_error(Eigen::MatrixXd point_column, Spline spl)
{
    Eigen::VectorXd errors(point_column.rows());
    for (Eigen::Index i = 0; i < point_column.rows(); i++){
        errors(i) = std::sqrt((spl(spl.getClosestT(point_column.block(i,0,1,point_column.cols()))) - point_column.block(i,0,1,point_column.cols())).array().square().sum());
    }
    return errors;
}

std::size_t Spline_make::search_tol_arg(Eigen::MatrixXd point_column, Eigen::Matrix4d char_mtx, double tol)
{
    Eigen::Matrix4Xd segment_controls = Spline_make::fromPoints(point_column, char_mtx);
    Spline segment(char_mtx, segment_controls);
    if (Spline_make::point_error(point_column, segment).maxCoeff() < tol){return point_column.rows();}
    std::size_t lower_bound = 0;
    std::size_t upper_bound = point_column.rows();
    std::size_t last_success = 4;
    while ((upper_bound > lower_bound+1) && ((upper_bound + lower_bound)/2 > 4)){
        segment_controls = Spline_make::fromPoints(point_column.block(0,0,(upper_bound + lower_bound)/2,point_column.cols()), char_mtx);
        segment = Spline(char_mtx, segment_controls);
        if (Spline_make::point_error(point_column.block(0,0,(upper_bound + lower_bound)/2,point_column.cols()), segment).maxCoeff() < tol){
            last_success = (upper_bound + lower_bound)/2;
            lower_bound = last_success;
        }else{
            upper_bound = (upper_bound + lower_bound)/2;
        }
    }
    return last_success;
}

Eigen::Matrix4Xd Spline_make::herm_seg(Eigen::Matrix4Xd c_ptx, double t0, double t1)
{
    Spline P(Spline_make::hermite_mtx, c_ptx);
    Eigen::Matrix4Xd M_q(c_ptx);
    M_q.block(0,0,1,c_ptx.cols()) = P(t0);
    M_q.block(1,0,1,c_ptx.cols()) = (t1 - t0) * P.getTangent(t0);
    M_q.block(2,0,1,c_ptx.cols()) = P(t1);
    M_q.block(3,0,1,c_ptx.cols()) = (t1 - t0) * (t1 - t0) * (t1 - t0) * (c_ptx.block(3,0,1,c_ptx.cols()) + c_ptx.block(1,0,1,c_ptx.cols()) - 2.0 * (c_ptx.block(2,0,1,c_ptx.cols()) - c_ptx.block(0,0,1,c_ptx.cols()))) + 2.0 * (M_q.block(2,0,1,c_ptx.cols()) - M_q.block(0,0,1,c_ptx.cols())) - M_q.block(1,0,1,c_ptx.cols());
    return M_q;
}

std::vector<double> Spline_make::serialize(Eigen::MatrixXd m){return std::vector<double>(m.data(), m.data() + (m.rows() * m.cols()));}

Eigen::MatrixXd Spline_make::deserialize(std::vector<double> v, Eigen::Index rows, Eigen::Index cols){
    Eigen::MatrixXd m(rows,cols);
    std::copy_n(v.data(),v.size(),m.data());
    return m;
}
