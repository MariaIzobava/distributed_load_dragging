#ifndef DYNAMICS_FACTORS_WITH_ANGLE_HPP
#define DYNAMICS_FACTORS_WITH_ANGLE_HPP

#include <string>

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/numericalDerivative.h>

#include "common.hpp"

using namespace std;
using namespace gtsam;



class RobotLoadWithHeightAndOrientationFactor : public NoiseModelFactor2<Vector6, Vector6> {
private:
    double cable_length_; // The nominal length of the cable
    double weight_;       // Weight for this factor

    double smooth_max_zero_(double x, double epsilon = 1e-6) const {
        return 0.5 * (x + std::sqrt(x*x + epsilon*epsilon));
    }

    Vector2 getAttachementPoint(const Vector6& xl_k) const {
        Vector2 p(2);
        p << xl_k(0) - 0.2 * cos(xl_k(2)), xl_k(1) - 0.2 * sin(xl_k(2));
        return p;
    }

public:
    RobotLoadWithHeightAndOrientationFactor(Key robotPosKey, Key loadPosKey, double cableLength,
                              double weight, const SharedNoiseModel& model)
        : NoiseModelFactor2<Vector6, Vector6>(model, robotPosKey, loadPosKey),
          cable_length_(cableLength), weight_(weight) {}

    Vector evaluateError(const Vector6& p_r, const Vector6& p_l,
                         gtsam::OptionalMatrixType H1,
                         gtsam::OptionalMatrixType H2) const override {

        Vector2 ap = getAttachementPoint(p_l);

        Vector3 diff(3);
        diff << p_r(0) - ap(0), p_r(1) - ap(1), p_r(2) - 0.2;
        double distance = diff.norm(); // Euclidean distance ||p_r - p_l||

        // The error value is `weight_ * smooth_max_zero(distance - cable_length_)`.
        double error_val = weight_ * smooth_max_zero_(cable_length_ - distance);

        if (H1 || H2) {
            double inner_term = cable_length_ - distance;
            double deriv_smooth_max_wrt_inner = 0.5 * (1.0 + inner_term / std::sqrt(inner_term*inner_term + 1e-6*1e-6));

            // The derivative of distance `||p_r - p_l||` with respect to `p_r` is `(p_r - p_l) / ||p_r - p_l||`,
            // which is the unit vector `e` pointing from load to robot.
            Vector3 partial_deriv_distance_pr = -diff / distance;
            // The derivative with respect to `p_l` is `-(p_r - p_l) / ||p_r - p_l||`, or `-e`.
            Vector3 partial_deriv_distance_pl = diff / distance;
            double partial_deriv_distance_pt = -0.2 * (diff(1) * cos(p_l(2)) - diff(0) * sin(p_l(2))) / distance;

            if (H1) { // Jacobian with respect to p_r
                // Chain rule: d(error)/dp_r = weight * d(smooth_max_zero)/d(inner) * d(inner)/d(distance) * d(distance)/dp_r
                // d(inner)/d(distance) is 1.0.
                (*H1) = (gtsam::Matrix(1, 6) << weight_ * deriv_smooth_max_wrt_inner * partial_deriv_distance_pr(0), weight_ * deriv_smooth_max_wrt_inner * partial_deriv_distance_pr(1), weight_ * deriv_smooth_max_wrt_inner * partial_deriv_distance_pr(2), 0, 0, 0).finished();
            }
            if (H2) { // Jacobian with respect to p_l
                (*H2) = (gtsam::Matrix(1, 6) << weight_ * deriv_smooth_max_wrt_inner * partial_deriv_distance_pl(0), weight_ * deriv_smooth_max_wrt_inner * partial_deriv_distance_pl(1), weight_ * deriv_smooth_max_wrt_inner * partial_deriv_distance_pt, 0, 0, 0).finished();
            }
        }
        return (Vector(1) << error_val).finished();
    }

};

class RobotLoadWithOrientationFactor : public NoiseModelFactor2<Vector4, Vector6> {
private:
    double cable_length_; // The nominal length of the cable
    double weight_;       // Weight for this factor
    double k_;

    double smooth_max_zero_(double x, double epsilon = 1e-6) const {
        return 0.5 * (x + std::sqrt(x*x + epsilon*epsilon));
    }

    Vector2 getAttachementPoint(const Vector6& xl_k) const {
        Vector2 p(2);
        p << xl_k(0) - 0.2 * cos(xl_k(2)), xl_k(1) - 0.2 * sin(xl_k(2));
        return p;
    }

    Vector2 getAttachementPoint2(const Vector6& xl_k) const {
        Vector2 p(2);
        p << xl_k(0) - 0.2 * sin(xl_k(2)), xl_k(1) + 0.2 * cos(xl_k(2));
        return p;
    }

public:
    RobotLoadWithOrientationFactor(Key robotPosKey, Key loadPosKey, double cableLength,
                              double weight, const SharedNoiseModel& model, int k = 1)
        : NoiseModelFactor2<Vector4, Vector6>(model, robotPosKey, loadPosKey),
          cable_length_(cableLength), weight_(weight), k_(k) {}

    Vector evaluateError(const Vector4& p_r, const Vector6& p_l,
                         gtsam::OptionalMatrixType H1,
                         gtsam::OptionalMatrixType H2) const override {

        Vector2 ap = getAttachementPoint(p_l);

        if (k_ == 2) {
            ap = getAttachementPoint2(p_l);
        }

        Vector2 diff(2);
        diff << p_r(0) - ap(0), p_r(1) - ap(1);
        double distance = diff.norm(); // Euclidean distance ||p_r - p_l||

        // The error value is `weight_ * smooth_max_zero(distance - cable_length_)`.
        double error_val = weight_ * smooth_max_zero_(cable_length_ - distance);

        if (H1 || H2) {
            double inner_term = cable_length_ - distance;
            double deriv_smooth_max_wrt_inner = 0.5 * (1.0 + inner_term / std::sqrt(inner_term*inner_term + 1e-6*1e-6));

            // The derivative of distance `||p_r - p_l||` with respect to `p_r` is `(p_r - p_l) / ||p_r - p_l||`,
            // which is the unit vector `e` pointing from load to robot.
            Vector2 partial_deriv_distance_pr = -diff / distance;
            // The derivative with respect to `p_l` is `-(p_r - p_l) / ||p_r - p_l||`, or `-e`.
            Vector2 partial_deriv_distance_pl = diff / distance;
            double partial_deriv_distance_pt = -0.2 * (diff(1) * cos(p_l(2)) - diff(0) * sin(p_l(2))) / distance;

            if (k_ == 2) {
                partial_deriv_distance_pt = -0.2 * (diff(0) * cos(p_l(2)) + diff(1) * sin(p_l(2))) / distance;
            }

            if (H1) { // Jacobian with respect to p_r
                // Chain rule: d(error)/dp_r = weight * d(smooth_max_zero)/d(inner) * d(inner)/d(distance) * d(distance)/dp_r
                // d(inner)/d(distance) is 1.0.
                (*H1) = (gtsam::Matrix(1, 4) << weight_ * deriv_smooth_max_wrt_inner * partial_deriv_distance_pr(0), weight_ * deriv_smooth_max_wrt_inner * partial_deriv_distance_pr(1), 0, 0).finished();
            }
            if (H2) { // Jacobian with respect to p_l
                (*H2) = (gtsam::Matrix(1, 6) << weight_ * deriv_smooth_max_wrt_inner * partial_deriv_distance_pl(0), weight_ * deriv_smooth_max_wrt_inner * partial_deriv_distance_pl(1), weight_ * deriv_smooth_max_wrt_inner * partial_deriv_distance_pt, 0, 0, 0).finished();
            }
        }
        return (Vector(1) << error_val).finished();
    }

};



class RobotDynamicsWithLoadOrientationFactor: public NoiseModelFactor5<Vector4, Vector6, Vector2, Vector1, Vector4> {
    double dt_;
    double robot_mass_;
    bool analitical_derivative_;
    double k_;

public:
    RobotDynamicsWithLoadOrientationFactor(Key key_xr_k, Key key_xl_k, Key key_u_k, Key key_tension_k, Key key_xr_k_plus_1,
                        double dt, double robot_mass, bool analitical_derivative, const SharedNoiseModel& model, int k = 1) :
        NoiseModelFactor5<Vector4, Vector6, Vector2, Vector1, Vector4>(model, key_xr_k, key_xl_k, key_u_k, key_tension_k, key_xr_k_plus_1),
        dt_(dt), robot_mass_(robot_mass), k_(k), analitical_derivative_(analitical_derivative) {}

    Vector evaluateErrorOnly(const Vector4& xr_k,
                         const Vector6& xl_k,
                         const Vector2& u_k, 
                         const Vector1& tension_k,
                         const Vector4& xr_k_plus_1) const  {
    
        Vector2 pos_k = xr_k.head<2>();
        Vector2 vel_k = xr_k.tail<2>();

        CableVectorHelper h(xr_k, xl_k, k_);

        Vector2 next_pos = pos_k + vel_k * dt_;
        Vector2 next_vel = vel_k + dt_ * (u_k - tension_k(0) * h.e_norm) / robot_mass_;
        
        Vector4 predicted_xr_k_plus_1(4);
        predicted_xr_k_plus_1 << next_pos, next_vel;

        return (Vector(4) << xr_k_plus_1 - predicted_xr_k_plus_1).finished();
    }

    Vector evaluateError(const Vector4& xr_k,
                         const Vector6& xl_k,
                         const Vector2& u_k, 
                         const Vector1& tension_k,
                         const Vector4& xr_k_plus_1,
                         gtsam::OptionalMatrixType H1,
                         gtsam::OptionalMatrixType H2,
                         gtsam::OptionalMatrixType H3,
                         gtsam::OptionalMatrixType H4,
                         gtsam::OptionalMatrixType H5) const override {

        CableVectorHelper h(xr_k, xl_k, k_);
        double A_CNST = dt_ / robot_mass_ * tension_k(0);

        if (H1) {
            if (analitical_derivative_) {
                *H1 = (gtsam::Matrix(4, 4) << 
                    -1,  0, -dt_, 0,
                    0, -1,  0,  -dt_,
                    A_CNST * h.ex_dxr,  A_CNST * h.ex_dyr, -1,   0,
                    A_CNST * h.ey_dxr,  A_CNST * h.ey_dyr,  0,  -1).finished();
            }else {
                *H1 = numericalDerivative51<Vector, Vector4, Vector6, Vector2, Vector1, Vector4 > (
                    [this](const Vector4& a,
                        const Vector6& b, 
                            const Vector2& c,
                            const Vector1& d,
                            const Vector4& e) {
                                return this->evaluateErrorOnly(a, b, c, d, e);
                            }, xr_k, xl_k,  u_k, tension_k, xr_k_plus_1, 1e-5
                );
            }

        }
        if (H2) {
            if (analitical_derivative_) {
                *H2 = (gtsam::Matrix(4, 6) << 
                    0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,
                    A_CNST * h.ex_dxl,  A_CNST * h.ex_dyl,  A_CNST * h.ex_dtheta, 0, 0, 0,
                    A_CNST * h.ey_dxl,  A_CNST * h.ey_dyl,  A_CNST * h.ey_dtheta, 0, 0, 0).finished();
            }else {
                *H2 = numericalDerivative52<Vector, Vector4, Vector6, Vector2, Vector1, Vector4 > (
                    [this](const Vector4& a,
                        const Vector6& b, 
                            const Vector2& c,
                            const Vector1& d,
                            const Vector4& e) {
                                return this->evaluateErrorOnly(a, b, c, d, e);
                            }, xr_k, xl_k,  u_k, tension_k, xr_k_plus_1, 1e-5
                );
            }
        }

        if (H3) {
            if (analitical_derivative_) {
                *H3 = (gtsam::Matrix(4, 2) << 
                    0, 0, 
                    0, 0,  
                    -(dt_/robot_mass_), 0, 
                    0, -(dt_/robot_mass_)).finished();
            }else {
                *H3 = numericalDerivative53<Vector, Vector4, Vector6, Vector2, Vector1, Vector4 > (
                    [this](const Vector4& a,
                        const Vector6& b, 
                            const Vector2& c,
                            const Vector1& d,
                            const Vector4& e) {
                                return this->evaluateErrorOnly(a, b, c, d, e);
                            }, xr_k, xl_k,  u_k, tension_k, xr_k_plus_1, 1e-5
                );
            }
        }
        if (H4) {
            if (analitical_derivative_) {
                *H4 = (gtsam::Matrix(4, 1) << 
                    0,
                    0,
                    (dt_ * h.e_norm(0) / robot_mass_), 
                    (dt_ * h.e_norm(1) / robot_mass_)).finished();
            } else {
                *H4 = numericalDerivative54<Vector, Vector4, Vector6, Vector2, Vector1, Vector4 > (
                    [this](const Vector4& a,
                        const Vector6& b, 
                            const Vector2& c,
                            const Vector1& d,
                            const Vector4& e) {
                                return this->evaluateErrorOnly(a, b, c, d, e);
                            }, xr_k, xl_k,  u_k, tension_k, xr_k_plus_1, 1e-5
                );
            }
        }
        if (H5) {
            *H5 = gtsam::Matrix4::Identity();
        }

        return evaluateErrorOnly(xr_k, xl_k,  u_k, tension_k, xr_k_plus_1);
    }
};


class LoadDynamicsWithLoadOrientationFactor: public NoiseModelFactor4<Vector6, Vector4, Vector1, Vector6> {
    double dt_;
    double load_mass_;
    double mu_;
    double mu2_;
    double g_;
    double inertia_;
    bool analitical_derivative_;
    double eps_ = 1000000.0;

public:
    LoadDynamicsWithLoadOrientationFactor(Key key_xl_k, Key key_xr_k, Key key_tension_k, Key key_xl_k_plus_1,
                        double dt, double load_mass, double mu, double mu2, double g, double inertia, bool analitical_derivative, const SharedNoiseModel& model) :
        NoiseModelFactor4<Vector6, Vector4, Vector1, Vector6>(model, key_xl_k, key_xr_k, key_tension_k, key_xl_k_plus_1),
        dt_(dt), load_mass_(load_mass), mu_(mu), mu2_(mu2), g_(g), inertia_(inertia), analitical_derivative_(analitical_derivative) {}

    Vector evaluateErrorOnly(const Vector6& xl_k, 
                         const Vector4& xr_k,
                         const Vector1& tension_k,
                         const Vector6& xl_k_plus_1) const {
        
        Vector2 lin_vel_k(2);
        lin_vel_k << xl_k(3), xl_k(4);
        double v_norm = lin_vel_k.norm();
        Vector2 normed_vel_k = Vector2::Zero();
        if (v_norm > 1e-12) {
            normed_vel_k << lin_vel_k / v_norm;
        }

        Vector3 pos_k = xl_k.head<3>();
        Vector3 vel_k = xl_k.tail<3>();
        Vector3 next_pos = pos_k + vel_k * dt_;

        CableVectorHelper h(xr_k, xl_k);

        Vector2 next_lin_vel = lin_vel_k + dt_ * (tension_k(0) * h.e_norm - mu_ * load_mass_ * g_ * normed_vel_k) / load_mass_;

        double r1 = -0.2 * cos(xl_k(2));
        double r2 = -0.2 * sin(xl_k(2));

        Vector1 next_ang_vel(1);
        next_ang_vel(0) = xl_k(5) + dt_ / inertia_ * (r1 * tension_k(0) * h.e_norm(1) - r2 * tension_k(0) * h.e_norm(0) - mu2_ * load_mass_ * g_ * tanh(eps_ * xl_k(5))); 

        Vector6 predicted_xl_k_plus_1(6);
        predicted_xl_k_plus_1 << next_pos, next_lin_vel, next_ang_vel;
        Vector6 state_diff = xl_k_plus_1 - predicted_xl_k_plus_1;

        return state_diff;

    }

    Vector evaluateError(const Vector6& xl_k, 
                         const Vector4& xr_k,
                         const Vector1& tension_k,
                         const Vector6& xl_k_plus_1,
                         gtsam::OptionalMatrixType H1,
                         gtsam::OptionalMatrixType H2,
                         gtsam::OptionalMatrixType H3,
                         gtsam::OptionalMatrixType H4) const override {
        
        Vector2 lin_vel_k(2);
        lin_vel_k << xl_k(3), xl_k(4);
        double v_norm = lin_vel_k.norm();
        Vector2 normed_vel_k = Vector2::Zero();
        double vx_dvx = 0.0;
        double vx_dvy = 0.0;
        double vy_dvx = 0.0;
        double vy_dvy = 0.0;
        if (v_norm > 1e-12) {
            normed_vel_k << lin_vel_k / v_norm;
            vx_dvx = 1.0 / v_norm - lin_vel_k(0) * lin_vel_k(0) / (v_norm * v_norm * v_norm);
            vx_dvy = -1.0 * lin_vel_k(0) * lin_vel_k(1) / (v_norm * v_norm * v_norm);

            vy_dvx = -1.0 * lin_vel_k(0) * lin_vel_k(1) / (v_norm * v_norm * v_norm);        
            vy_dvy = 1.0 / v_norm - lin_vel_k(1) * lin_vel_k(1) / (v_norm * v_norm * v_norm);
        }

        CableVectorHelper h(xr_k, xl_k);

        double r1 = -0.2 * cos(xl_k(2));
        double r2 = -0.2 * sin(xl_k(2));
        double r1_dtheta = 0.2 * sin(xl_k(2));
        double r2_dtheta = -0.2 * cos(xl_k(2));
        double sech_sq_theta = eps_ * (1.0 / cosh(eps_ * xl_k(5))) * (1.0 / cosh(eps_ * xl_k(5)));
        double A_CNST = dt_ / load_mass_ * tension_k(0);

        if (H1) {
            if (analitical_derivative_) {
                *H1 = (gtsam::Matrix(6, 6) << 
                    -1,  0,  0, -dt_,  0,   0,
                    0, -1,  0,   0, -dt_,  0,
                    0,  0, -1,   0,   0, -dt_,
                    -A_CNST * h.ex_dxl,  -A_CNST * h.ex_dyl,  -A_CNST * h.ex_dtheta,  -1 + dt_ * mu_ * g_ * vx_dvx,       dt_ * mu_ * g_ * vx_dvy,  0,
                    -A_CNST * h.ey_dxl,  -A_CNST * h.ey_dyl,  -A_CNST * h.ey_dtheta,       dt_ * mu_ * g_ * vy_dvx,  -1 + dt_ * mu_ * g_ * vy_dvy,  0,
                    // The last row of the matrix: each element on its own row for visibility
                    -dt_ / inertia_ * tension_k(0) * (r1 * h.ey_dxl - r2 * h.ex_dxl), 
                    -dt_ / inertia_ * tension_k(0) * (r1 * h.ey_dyl - r2 * h.ex_dyl),  
                    -dt_ / inertia_ * tension_k(0) * (r1_dtheta * h.e_norm(1) + r1 * h.ey_dtheta - r2_dtheta * h.e_norm(0) - r2 * h.ex_dtheta), 
                    0, 
                    0, 
                    -1 + dt_ / inertia_ * (mu2_ * load_mass_ * g_ * sech_sq_theta)
                    ).finished();

            } else {
                *H1 = numericalDerivative41<Vector,Vector6, Vector4, Vector1, Vector6 > (
                    [this](const Vector6& a, 
                            const Vector4& b,
                            const Vector1& c,
                            const Vector6& d) {
                                return this->evaluateErrorOnly(a, b, c, d);
                            }, xl_k,  xr_k, tension_k, xl_k_plus_1, 1e-5
                );
            }
            
        }
        if (H2) {
            if (analitical_derivative_) {
                *H2 = (gtsam::Matrix(6, 4) << 
                    0, 0,  0,  0,
                    0, 0,  0,  0,
                    0, 0,  0,  0,
                    -A_CNST * h.ex_dxr,  -A_CNST * h.ex_dyr, 0, 0,
                    -A_CNST * h.ey_dxr,  -A_CNST * h.ey_dyr,  0, 0,
                    // The last row of the matrix: each element on its own row for visibility
                    -dt_ / inertia_ * tension_k(0) * (r1 * h.ey_dxr - r2 * h.ex_dxr), 
                    -dt_ / inertia_ * tension_k(0) * (r1 * h.ey_dyr - r2 * h.ex_dyr), 
                    0, 
                    0
                    ).finished();

            } else {
                *H2 = numericalDerivative42<Vector,Vector6, Vector4, Vector1, Vector6 > (
                    [this](const Vector6& a, 
                            const Vector4& b,
                            const Vector1& c,
                            const Vector6& d) {
                                return this->evaluateErrorOnly(a, b, c, d);
                            }, xl_k,  xr_k, tension_k, xl_k_plus_1, 1e-5
                );
            }
        }
        if (H3) {
            if (analitical_derivative_) {
                *H3 = (gtsam::Matrix(6, 1) << 
                    0,
                    0,
                    0,
                    -(dt_ * h.e_norm(0) / load_mass_),
                    -(dt_ * h.e_norm(1) / load_mass_),
                    -dt_ / inertia_ * (r1 * h.e_norm(1) - r2 * h.e_norm(0))
                    ).finished();
            } else {
                *H3 = numericalDerivative43<Vector,Vector6, Vector4, Vector1, Vector6 > (
                    [this](const Vector6& a, 
                            const Vector4& b,
                            const Vector1& c,
                            const Vector6& d) {
                                return this->evaluateErrorOnly(a, b, c, d);
                            }, xl_k,  xr_k, tension_k, xl_k_plus_1, 1e-5
                );
            }
        }
        if (H4) {
            *H4 = gtsam::Matrix6::Identity();
        }

        return evaluateErrorOnly(xl_k,  xr_k, tension_k, xl_k_plus_1);
    }
};


class LoadDynamicsTwoRobotsWithLoadOrientationFactor: public NoiseModelFactor6<Vector6, Vector4, Vector1, Vector4, Vector1, Vector6> {
    double dt_;
    double load_mass_;
    double mu_;
    double mu2_;
    double inertia_;
    double g_;
    double analitical_derivative_;
    double eps_ = 1000000.0;
    int atp2_;

public:
    LoadDynamicsTwoRobotsWithLoadOrientationFactor(
        Key key_xl_k, 
        Key key_xr1_k, 
        Key key_tension1_k, 
        Key key_xr2_k, 
        Key key_tension2_k, 
        Key key_xl_k_plus_1,   
        double dt, 
        double load_mass, 
        double mu, 
        double mu2, 
        double g, 
        double inertia, 
        bool analitical_derivative, 
        int atp2,
        const SharedNoiseModel& model) :
        NoiseModelFactor6<Vector6, Vector4, Vector1, Vector4, Vector1, Vector6>(model, key_xl_k, key_xr1_k, key_tension1_k, key_xr2_k, key_tension2_k, key_xl_k_plus_1),
        dt_(dt), load_mass_(load_mass), mu_(mu), mu2_(mu), inertia_(inertia), g_(g), analitical_derivative_(analitical_derivative), atp2_(atp2) {}


    Vector evaluateErrorOnly(const Vector6& xl_k, 
                         const Vector4& xr1_k,
                         const Vector1& tension1_k,
                         const Vector4& xr2_k,
                         const Vector1& tension2_k,
                         const Vector6& xl_k_plus_1) const {
        Vector2 lin_vel_k(2);
        lin_vel_k << xl_k(3), xl_k(4);
        double v_norm = lin_vel_k.norm();
        Vector2 normed_vel_k = Vector2::Zero();
        if (v_norm > 1e-12) {
            normed_vel_k << lin_vel_k / v_norm;
        }

        Vector3 pos_k = xl_k.head<3>();
        Vector3 vel_k = xl_k.tail<3>();
        Vector3 next_pos = pos_k + vel_k * dt_;

        CableVectorHelper h1(xr1_k, xl_k);
        CableVectorHelper h2(xr2_k, xl_k, atp2_);

        Vector2 next_lin_vel = lin_vel_k + dt_ / load_mass_ * (tension1_k(0) * h1.e_norm + tension2_k(0) * h2.e_norm - mu_ * load_mass_ * g_ * normed_vel_k);

        double r1 = -0.2 * cos(xl_k(2));
        double r2 = -0.2 * sin(xl_k(2));

        double r21 = -0.2 * sin(xl_k(2));
        double r22 = 0.2 * cos(xl_k(2));
        if (atp2_ == 1) {
            r21 = r1;
            r22 = r2;
        }

        Vector1 next_ang_vel(1);
        next_ang_vel(0) = xl_k(5) + dt_ / inertia_ * (r1 * tension1_k(0) * h1.e_norm(1) - r2 * tension1_k(0) * h1.e_norm(0) + r21 * tension2_k(0) * h2.e_norm(1) - r22 * tension2_k(0) * h2.e_norm(0) - mu2_ * load_mass_ * g_ * tanh(eps_ * xl_k(5))); 

        Vector6 predicted_xl_k_plus_1(6);
        predicted_xl_k_plus_1 << next_pos, next_lin_vel, next_ang_vel;

        return (Vector(6) << xl_k_plus_1 - predicted_xl_k_plus_1).finished();
    }


    Vector evaluateError(const Vector6& xl_k, 
                         const Vector4& xr1_k,
                         const Vector1& tension1_k,
                         const Vector4& xr2_k,
                         const Vector1& tension2_k,
                         const Vector6& xl_k_plus_1,
                         gtsam::OptionalMatrixType H1,
                         gtsam::OptionalMatrixType H2,
                         gtsam::OptionalMatrixType H3,
                         gtsam::OptionalMatrixType H4,
                         gtsam::OptionalMatrixType H5,
                         gtsam::OptionalMatrixType H6) const override {

        Vector2 lin_vel_k(2);
        lin_vel_k << xl_k(3), xl_k(4);
        double v_norm = lin_vel_k.norm();
        Vector2 normed_vel_k = Vector2::Zero();
        double vx_dvx = 0.0;
        double vx_dvy = 0.0;
        double vy_dvx = 0.0;
        double vy_dvy = 0.0;
        if (v_norm > 1e-12) {
            normed_vel_k << lin_vel_k / v_norm;
            vx_dvx = 1.0 / v_norm - lin_vel_k(0) * lin_vel_k(0) / (v_norm * v_norm * v_norm);
            vx_dvy = -1.0 * lin_vel_k(0) * lin_vel_k(1) / (v_norm * v_norm * v_norm);

            vy_dvx = -1.0 * lin_vel_k(0) * lin_vel_k(1) / (v_norm * v_norm * v_norm);        
            vy_dvy = 1.0 / v_norm - lin_vel_k(1) * lin_vel_k(1) / (v_norm * v_norm * v_norm);
        }

        CableVectorHelper h1(xr1_k, xl_k);
        CableVectorHelper h2(xr2_k, xl_k, atp2_);

        double r1 = -0.2 * cos(xl_k(2));
        double r2 = -0.2 * sin(xl_k(2));
        double r1_dtheta = 0.2 * sin(xl_k(2));
        double r2_dtheta = -0.2 * cos(xl_k(2));

        double r21 = -0.2 * sin(xl_k(2));
        double r22 = 0.2 * cos(xl_k(2));
        double r21_dtheta = -0.2 * cos(xl_k(2));
        double r22_dtheta = -0.2 * sin(xl_k(2));
        if (atp2_ == 1) {
            r21 = r1;
            r22 = r2;
            r21_dtheta = r1_dtheta;
            r22_dtheta = r2_dtheta;
        }
        double sech_sq_theta = eps_ * (1.0 / cosh(eps_ * xl_k(5))) * (1.0 / cosh(eps_ * xl_k(5)));
        double A_CNST1 = dt_ / load_mass_ * tension1_k(0);
        double A_CNST2 = dt_ / load_mass_ * tension2_k(0);

        if (H1) {
            if (!analitical_derivative_) {
                *H1 = numericalDerivative61<Vector,Vector6, Vector4, Vector1, Vector4, Vector1, Vector6 > (
                        [this](const Vector6& a, 
                                const Vector4& b,
                                const Vector1& c,
                                const Vector4& d,
                                const Vector1& e,
                                const Vector6& f) {
                                    return this->evaluateErrorOnly(a, b, c, d, e, f);
                                }, xl_k, xr1_k, tension1_k, xr2_k, tension2_k, xl_k_plus_1, 1e-5
                    );
            } else {
                *H1 = (gtsam::Matrix(6, 6) << 
                    -1,  0,  0, -dt_,  0,   0,
                    0, -1,  0,   0, -dt_,  0,
                    0,  0, -1,   0,   0, -dt_,
                    -A_CNST1 * h1.ex_dxl -A_CNST2 * h2.ex_dxl,  -A_CNST1 * h1.ex_dyl -A_CNST2 * h2.ex_dyl,  -A_CNST1 * h1.ex_dtheta -A_CNST2 * h2.ex_dtheta,  -1 + dt_ * mu_ * g_ * vx_dvx,       dt_ * mu_ * g_ * vx_dvy,  0,
                    -A_CNST1 * h1.ey_dxl -A_CNST2 * h2.ey_dxl,  -A_CNST1 * h1.ey_dyl -A_CNST2 * h2.ey_dyl,  -A_CNST1 * h1.ey_dtheta -A_CNST2 * h2.ey_dtheta,       dt_ * mu_ * g_ * vy_dvx,  -1 + dt_ * mu_ * g_ * vy_dvy,  0,
                    // The last row of the matrix: each element on its own row for visibility
                    -dt_ / inertia_ * tension1_k(0) * (r1 * h1.ey_dxl - r2 * h1.ex_dxl) -dt_ / inertia_ * tension2_k(0) * (r21 * h2.ey_dxl - r22 * h2.ex_dxl), 
                    -dt_ / inertia_ * tension1_k(0) * (r1 * h1.ey_dyl - r2 * h1.ex_dyl) -dt_ / inertia_ * tension2_k(0) * (r21 * h2.ey_dyl - r22 * h2.ex_dyl),  
                    -dt_ / inertia_ * tension1_k(0) * (r1_dtheta * h1.e_norm(1) + r1 * h1.ey_dtheta - r2_dtheta * h1.e_norm(0) - r2 * h1.ex_dtheta) -dt_ / inertia_ * tension2_k(0) * (r21_dtheta * h2.e_norm(1) + r21 * h2.ey_dtheta - r22_dtheta * h2.e_norm(0) - r22 * h2.ex_dtheta), 
                    0, 
                    0, 
                    -1 + dt_ / inertia_ * (mu2_ * load_mass_ * g_ * sech_sq_theta)
                    ).finished();
            }
        }
        if (H2) {
            if (!analitical_derivative_) {
                *H2 = numericalDerivative62<Vector,Vector6, Vector4, Vector1, Vector4, Vector1, Vector6 > (
                        [this](const Vector6& a, 
                                const Vector4& b,
                                const Vector1& c,
                                const Vector4& d,
                                const Vector1& e,
                                const Vector6& f) {
                                    return this->evaluateErrorOnly(a, b, c, d, e, f);
                                }, xl_k, xr1_k, tension1_k, xr2_k, tension2_k, xl_k_plus_1, 1e-5
                    );
            } else {
                *H2 = (gtsam::Matrix(6, 4) << 
                    0, 0,  0,  0,
                    0, 0,  0,  0,
                    0, 0,  0,  0,
                    -A_CNST1 * h1.ex_dxr,  -A_CNST1 * h1.ex_dyr, 0, 0,
                    -A_CNST1 * h1.ey_dxr,  -A_CNST1 * h1.ey_dyr,  0, 0,
                    // The last row of the matrix: each element on its own row for visibility
                    -dt_ / inertia_ * tension1_k(0) * (r1 * h1.ey_dxr - r2 * h1.ex_dxr), 
                    -dt_ / inertia_ * tension1_k(0) * (r1 * h1.ey_dyr - r2 * h1.ex_dyr), 
                    0, 
                    0
                    ).finished();
            }
        }
        if (H3) {
            if (!analitical_derivative_) {
                *H3 = numericalDerivative63<Vector,Vector6, Vector4, Vector1, Vector4, Vector1, Vector6 > (
                        [this](const Vector6& a, 
                                const Vector4& b,
                                const Vector1& c,
                                const Vector4& d,
                                const Vector1& e,
                                const Vector6& f) {
                                    return this->evaluateErrorOnly(a, b, c, d, e, f);
                                }, xl_k, xr1_k, tension1_k, xr2_k, tension2_k, xl_k_plus_1, 1e-5
                    );
            } else {
                *H3 = (gtsam::Matrix(6, 1) << 
                    0,
                    0,
                    0,
                    -(dt_ * h1.e_norm(0) / load_mass_),
                    -(dt_ * h1.e_norm(1) / load_mass_),
                    -dt_ / inertia_ * (r1 * h1.e_norm(1) - r2 * h1.e_norm(0))
                    ).finished();
            }
        }
        if (H4) {
            if (!analitical_derivative_) {
                *H4 = numericalDerivative64<Vector,Vector6, Vector4, Vector1, Vector4, Vector1, Vector6 > (
                        [this](const Vector6& a, 
                                const Vector4& b,
                                const Vector1& c,
                                const Vector4& d,
                                const Vector1& e,
                                const Vector6& f) {
                                    return this->evaluateErrorOnly(a, b, c, d, e, f);
                                }, xl_k, xr1_k, tension1_k, xr2_k, tension2_k, xl_k_plus_1, 1e-5
                    );
            } else {
                *H4 = (gtsam::Matrix(6, 4) << 
                    0, 0,  0,  0,
                    0, 0,  0,  0,
                    0, 0,  0,  0,
                    -A_CNST2 * h2.ex_dxr,  -A_CNST2 * h2.ex_dyr, 0, 0,
                    -A_CNST2 * h2.ey_dxr,  -A_CNST2 * h2.ey_dyr,  0, 0,
                    // The last row of the matrix: each element on its own row for visibility
                    -dt_ / inertia_ * tension2_k(0) * (r21 * h2.ey_dxr - r22 * h2.ex_dxr), 
                    -dt_ / inertia_ * tension2_k(0) * (r21 * h2.ey_dyr - r22 * h2.ex_dyr), 
                    0, 
                    0
                    ).finished();
            }
        }
        if (H5) {
            if (!analitical_derivative_) {
                *H5 = numericalDerivative65<Vector,Vector6, Vector4, Vector1, Vector4, Vector1, Vector6 > (
                        [this](const Vector6& a, 
                                const Vector4& b,
                                const Vector1& c,
                                const Vector4& d,
                                const Vector1& e,
                                const Vector6& f) {
                                    return this->evaluateErrorOnly(a, b, c, d, e, f);
                                }, xl_k, xr1_k, tension1_k, xr2_k, tension2_k, xl_k_plus_1, 1e-5
                    );
            } else {
                *H5 = (gtsam::Matrix(6, 1) << 
                    0,
                    0,
                    0,
                    -(dt_ * h2.e_norm(0) / load_mass_),
                    -(dt_ * h2.e_norm(1) / load_mass_),
                    -dt_ / inertia_ * (r21 * h2.e_norm(1) - r22 * h2.e_norm(0))
                    ).finished();
            }
        }
        if (H6) {
            *H6 = gtsam::Matrix6::Identity();
        }

        return evaluateErrorOnly(xl_k,  xr1_k, tension1_k,  xr2_k, tension2_k, xl_k_plus_1);
    }
};


class LoadDynamicsThreeRobotsWithLoadOrientationFactor: public NoiseModelFactorN<Vector6, Vector4, Vector1, Vector4, Vector1, Vector4, Vector1, Vector6> {
    double dt_;
    double load_mass_;
    double mu_;
    double mu2_;
    double inertia_;
    double g_;
    double eps_ = 1000000.0;
    int atp2_;

public:
    LoadDynamicsThreeRobotsWithLoadOrientationFactor(
        Key key_xl_k, 
        Key key_xr1_k, 
        Key key_tension1_k, 
        Key key_xr2_k, 
        Key key_tension2_k, 
        Key key_xr3_k, 
        Key key_tension3_k, 
        Key key_xl_k_plus_1,   
        double dt, 
        double load_mass, 
        double mu, 
        double mu2, 
        double g, 
        double inertia, 
        const SharedNoiseModel& model) :
        NoiseModelFactorN<Vector6, Vector4, Vector1, Vector4, Vector1, Vector4, Vector1, Vector6>(
            model, 
            key_xl_k, 
            key_xr1_k, key_tension1_k, 
            key_xr2_k, key_tension2_k, 
            key_xr3_k, key_tension3_k, 
            key_xl_k_plus_1),
        dt_(dt), load_mass_(load_mass), mu_(mu), mu2_(mu), inertia_(inertia), g_(g) {}


    Vector evaluateErrorOnly(const Vector6& xl_k, 
                         const Vector4& xr1_k,
                         const Vector1& tension1_k,
                         const Vector4& xr2_k,
                         const Vector1& tension2_k,
                         const Vector4& xr3_k,
                         const Vector1& tension3_k,
                         const Vector6& xl_k_plus_1) const {
        Vector2 lin_vel_k(2);
        lin_vel_k << xl_k(3), xl_k(4);
        double v_norm = lin_vel_k.norm();
        Vector2 normed_vel_k = Vector2::Zero();
        if (v_norm > 1e-12) {
            normed_vel_k << lin_vel_k / v_norm;
        }

        Vector3 pos_k = xl_k.head<3>();
        Vector3 vel_k = xl_k.tail<3>();
        Vector3 next_pos = pos_k + vel_k * dt_;

        CableVectorHelper h1(xr1_k, xl_k);
        CableVectorHelper h2(xr2_k, xl_k);
        CableVectorHelper h3(xr3_k, xl_k);

        Vector2 next_lin_vel = lin_vel_k + dt_ / load_mass_ * (tension1_k(0) * h1.e_norm + tension2_k(0) * h2.e_norm + tension3_k(0) * h3.e_norm - mu_ * load_mass_ * g_ * normed_vel_k);

        double r1 = -0.2 * cos(xl_k(2));
        double r2 = -0.2 * sin(xl_k(2));

        Vector1 next_ang_vel(1);
        next_ang_vel(0) = xl_k(5) + dt_ / inertia_ * (
            r1 * tension1_k(0) * h1.e_norm(1) - r2 * tension1_k(0) * h1.e_norm(0) + 
            r1 * tension2_k(0) * h2.e_norm(1) - r2 * tension2_k(0) * h2.e_norm(0) +
            r1 * tension3_k(0) * h3.e_norm(1) - r2 * tension3_k(0) * h3.e_norm(0) - mu2_ * load_mass_ * g_ * tanh(eps_ * xl_k(5))); 

        Vector6 predicted_xl_k_plus_1(6);
        predicted_xl_k_plus_1 << next_pos, next_lin_vel, next_ang_vel;

        return (Vector(6) << xl_k_plus_1 - predicted_xl_k_plus_1).finished();
    }


    Vector evaluateError(const Vector6& xl_k, 
                         const Vector4& xr1_k,
                         const Vector1& tension1_k,
                         const Vector4& xr2_k,
                         const Vector1& tension2_k,
                         const Vector4& xr3_k,
                         const Vector1& tension3_k,
                         const Vector6& xl_k_plus_1,
                         gtsam::OptionalMatrixType H1,
                         gtsam::OptionalMatrixType H2,
                         gtsam::OptionalMatrixType H3,
                         gtsam::OptionalMatrixType H4,
                         gtsam::OptionalMatrixType H5,
                         gtsam::OptionalMatrixType H6,
                         gtsam::OptionalMatrixType H7,
                         gtsam::OptionalMatrixType H8) const override {

        Vector2 lin_vel_k(2);
        lin_vel_k << xl_k(3), xl_k(4);
        double v_norm = lin_vel_k.norm();
        Vector2 normed_vel_k = Vector2::Zero();
        double vx_dvx = 0.0;
        double vx_dvy = 0.0;
        double vy_dvx = 0.0;
        double vy_dvy = 0.0;
        if (v_norm > 1e-12) {
            normed_vel_k << lin_vel_k / v_norm;
            vx_dvx = 1.0 / v_norm - lin_vel_k(0) * lin_vel_k(0) / (v_norm * v_norm * v_norm);
            vx_dvy = -1.0 * lin_vel_k(0) * lin_vel_k(1) / (v_norm * v_norm * v_norm);

            vy_dvx = -1.0 * lin_vel_k(0) * lin_vel_k(1) / (v_norm * v_norm * v_norm);        
            vy_dvy = 1.0 / v_norm - lin_vel_k(1) * lin_vel_k(1) / (v_norm * v_norm * v_norm);
        }

        CableVectorHelper h1(xr1_k, xl_k);
        CableVectorHelper h2(xr2_k, xl_k);
        CableVectorHelper h3(xr3_k, xl_k);

        double r1 = -0.2 * cos(xl_k(2));
        double r2 = -0.2 * sin(xl_k(2));
        double r1_dtheta = 0.2 * sin(xl_k(2));
        double r2_dtheta = -0.2 * cos(xl_k(2));

        double sech_sq_theta = eps_ * (1.0 / cosh(eps_ * xl_k(5))) * (1.0 / cosh(eps_ * xl_k(5)));
        double A_CNST1 = dt_ / load_mass_ * tension1_k(0);
        double A_CNST2 = dt_ / load_mass_ * tension2_k(0);
        double A_CNST3 = dt_ / load_mass_ * tension3_k(0);

        if (H1) {
                *H1 = (gtsam::Matrix(6, 6) << 
                    -1,  0,  0, -dt_,  0,   0,
                    0, -1,  0,   0, -dt_,  0,
                    0,  0, -1,   0,   0, -dt_,
                    -A_CNST1 * h1.ex_dxl -A_CNST2 * h2.ex_dxl -A_CNST3 * h3.ex_dxl,  -A_CNST1 * h1.ex_dyl -A_CNST2 * h2.ex_dyl -A_CNST3 * h3.ex_dyl,  -A_CNST1 * h1.ex_dtheta -A_CNST2 * h2.ex_dtheta -A_CNST3 * h3.ex_dtheta,  -1 + dt_ * mu_ * g_ * vx_dvx,       dt_ * mu_ * g_ * vx_dvy,  0,
                    -A_CNST1 * h1.ey_dxl -A_CNST2 * h2.ey_dxl -A_CNST3 * h3.ey_dxl,  -A_CNST1 * h1.ey_dyl -A_CNST2 * h2.ey_dyl -A_CNST3 * h3.ey_dyl,  -A_CNST1 * h1.ey_dtheta -A_CNST2 * h2.ey_dtheta -A_CNST3 * h3.ey_dtheta,       dt_ * mu_ * g_ * vy_dvx,  -1 + dt_ * mu_ * g_ * vy_dvy,  0,
                    // The last row of the matrix: each element on its own row for visibility
                    -dt_ / inertia_ * tension1_k(0) * (r1 * h1.ey_dxl - r2 * h1.ex_dxl) -dt_ / inertia_ * tension2_k(0) * (r1 * h2.ey_dxl - r2 * h2.ex_dxl) -dt_ / inertia_ * tension3_k(0) * (r1 * h3.ey_dxl - r2 * h3.ex_dxl), 
                    -dt_ / inertia_ * tension1_k(0) * (r1 * h1.ey_dyl - r2 * h1.ex_dyl) -dt_ / inertia_ * tension2_k(0) * (r1 * h2.ey_dyl - r2 * h2.ex_dyl) -dt_ / inertia_ * tension3_k(0) * (r1 * h3.ey_dyl - r2 * h3.ex_dyl),  
                    -dt_ / inertia_ * tension1_k(0) * (r1_dtheta * h1.e_norm(1) + r1 * h1.ey_dtheta - r2_dtheta * h1.e_norm(0) - r2 * h1.ex_dtheta) 
                    -dt_ / inertia_ * tension2_k(0) * (r1_dtheta * h2.e_norm(1) + r1 * h2.ey_dtheta - r2_dtheta * h2.e_norm(0) - r2 * h2.ex_dtheta)
                    -dt_ / inertia_ * tension3_k(0) * (r1_dtheta * h3.e_norm(1) + r1 * h3.ey_dtheta - r2_dtheta * h3.e_norm(0) - r2 * h3.ex_dtheta), 
                    0, 
                    0, 
                    -1 + dt_ / inertia_ * (mu2_ * load_mass_ * g_ * sech_sq_theta)
                    ).finished();
        }
        if (H2) {
                *H2 = (gtsam::Matrix(6, 4) << 
                    0, 0,  0,  0,
                    0, 0,  0,  0,
                    0, 0,  0,  0,
                    -A_CNST1 * h1.ex_dxr,  -A_CNST1 * h1.ex_dyr, 0, 0,
                    -A_CNST1 * h1.ey_dxr,  -A_CNST1 * h1.ey_dyr,  0, 0,
                    // The last row of the matrix: each element on its own row for visibility
                    -dt_ / inertia_ * tension1_k(0) * (r1 * h1.ey_dxr - r2 * h1.ex_dxr), 
                    -dt_ / inertia_ * tension1_k(0) * (r1 * h1.ey_dyr - r2 * h1.ex_dyr), 
                    0, 
                    0
                    ).finished();
        }
        if (H3) {
            
                *H3 = (gtsam::Matrix(6, 1) << 
                    0,
                    0,
                    0,
                    -(dt_ * h1.e_norm(0) / load_mass_),
                    -(dt_ * h1.e_norm(1) / load_mass_),
                    -dt_ / inertia_ * (r1 * h1.e_norm(1) - r2 * h1.e_norm(0))
                    ).finished();
        }
        if (H4) {
            
                *H4 = (gtsam::Matrix(6, 4) << 
                    0, 0,  0,  0,
                    0, 0,  0,  0,
                    0, 0,  0,  0,
                    -A_CNST2 * h2.ex_dxr,  -A_CNST2 * h2.ex_dyr, 0, 0,
                    -A_CNST2 * h2.ey_dxr,  -A_CNST2 * h2.ey_dyr,  0, 0,
                    // The last row of the matrix: each element on its own row for visibility
                    -dt_ / inertia_ * tension2_k(0) * (r1 * h2.ey_dxr - r2 * h2.ex_dxr), 
                    -dt_ / inertia_ * tension2_k(0) * (r1 * h2.ey_dyr - r2 * h2.ex_dyr), 
                    0, 
                    0
                    ).finished();
        }
        if (H5) {
                *H5 = (gtsam::Matrix(6, 1) << 
                    0,
                    0,
                    0,
                    -(dt_ * h2.e_norm(0) / load_mass_),
                    -(dt_ * h2.e_norm(1) / load_mass_),
                    -dt_ / inertia_ * (r1 * h2.e_norm(1) - r2 * h2.e_norm(0))
                    ).finished();
        }
         if (H6) {
            
                *H6 = (gtsam::Matrix(6, 4) << 
                    0, 0,  0,  0,
                    0, 0,  0,  0,
                    0, 0,  0,  0,
                    -A_CNST3 * h3.ex_dxr,  -A_CNST3 * h3.ex_dyr, 0, 0,
                    -A_CNST3 * h3.ey_dxr,  -A_CNST3 * h3.ey_dyr,  0, 0,
                    // The last row of the matrix: each element on its own row for visibility
                    -dt_ / inertia_ * tension3_k(0) * (r1 * h3.ey_dxr - r2 * h3.ex_dxr), 
                    -dt_ / inertia_ * tension3_k(0) * (r1 * h3.ey_dyr - r2 * h3.ex_dyr), 
                    0, 
                    0
                    ).finished();
        }
        if (H7) {
                *H7 = (gtsam::Matrix(6, 1) << 
                    0,
                    0,
                    0,
                    -(dt_ * h3.e_norm(0) / load_mass_),
                    -(dt_ * h3.e_norm(1) / load_mass_),
                    -dt_ / inertia_ * (r1 * h3.e_norm(1) - r2 * h3.e_norm(0))
                    ).finished();
        }
        if (H8) {
            *H8 = gtsam::Matrix6::Identity();
        }

        return evaluateErrorOnly(xl_k,  xr1_k, tension1_k,  xr2_k, tension2_k,  xr3_k, tension3_k, xl_k_plus_1);
    }
};


class LoadDynamicsFourRobotsWithLoadOrientationFactor: public NoiseModelFactorN<Vector6, Vector4, Vector1, Vector4, Vector1, Vector4, Vector1, Vector4, Vector1, Vector6> {
    double dt_;
    double load_mass_;
    double mu_;
    double mu2_;
    double inertia_;
    double g_;
    double eps_ = 1000000.0;
    int atp2_;

public:
    LoadDynamicsFourRobotsWithLoadOrientationFactor(
        Key key_xl_k, 
        Key key_xr1_k, 
        Key key_tension1_k, 
        Key key_xr2_k, 
        Key key_tension2_k, 
        Key key_xr3_k, 
        Key key_tension3_k, 
        Key key_xr4_k, 
        Key key_tension4_k, 
        Key key_xl_k_plus_1,   
        double dt, 
        double load_mass, 
        double mu, 
        double mu2, 
        double g, 
        double inertia, 
        const SharedNoiseModel& model) :
        NoiseModelFactorN<Vector6, Vector4, Vector1, Vector4, Vector1, Vector4, Vector1, Vector4, Vector1, Vector6>(
            model, 
            key_xl_k, 
            key_xr1_k, key_tension1_k, 
            key_xr2_k, key_tension2_k, 
            key_xr3_k, key_tension3_k, 
            key_xr4_k, key_tension4_k, 
            key_xl_k_plus_1),
        dt_(dt), load_mass_(load_mass), mu_(mu), mu2_(mu), inertia_(inertia), g_(g) {}


    Vector evaluateErrorOnly(const Vector6& xl_k, 
                         const Vector4& xr1_k,
                         const Vector1& tension1_k,
                         const Vector4& xr2_k,
                         const Vector1& tension2_k,
                         const Vector4& xr3_k,
                         const Vector1& tension3_k,
                         const Vector4& xr4_k,
                         const Vector1& tension4_k,
                         const Vector6& xl_k_plus_1) const {
        Vector2 lin_vel_k(2);
        lin_vel_k << xl_k(3), xl_k(4);
        double v_norm = lin_vel_k.norm();
        Vector2 normed_vel_k = Vector2::Zero();
        if (v_norm > 1e-12) {
            normed_vel_k << lin_vel_k / v_norm;
        }

        Vector3 pos_k = xl_k.head<3>();
        Vector3 vel_k = xl_k.tail<3>();
        Vector3 next_pos = pos_k + vel_k * dt_;

        CableVectorHelper h1(xr1_k, xl_k);
        CableVectorHelper h2(xr2_k, xl_k);
        CableVectorHelper h3(xr3_k, xl_k);
        CableVectorHelper h4(xr4_k, xl_k);

        Vector2 next_lin_vel = lin_vel_k + dt_ / load_mass_ * (
            tension1_k(0) * h1.e_norm + 
            tension2_k(0) * h2.e_norm + 
            tension3_k(0) * h3.e_norm +
            tension4_k(0) * h4.e_norm - mu_ * load_mass_ * g_ * normed_vel_k);

        double r1 = -0.2 * cos(xl_k(2));
        double r2 = -0.2 * sin(xl_k(2));

        Vector1 next_ang_vel(1);
        next_ang_vel(0) = xl_k(5) + dt_ / inertia_ * (
            r1 * tension1_k(0) * h1.e_norm(1) - r2 * tension1_k(0) * h1.e_norm(0) + 
            r1 * tension2_k(0) * h2.e_norm(1) - r2 * tension2_k(0) * h2.e_norm(0) +
            r1 * tension3_k(0) * h3.e_norm(1) - r2 * tension3_k(0) * h3.e_norm(0) +
            r1 * tension4_k(0) * h4.e_norm(1) - r2 * tension4_k(0) * h4.e_norm(0) - mu2_ * load_mass_ * g_ * tanh(eps_ * xl_k(5))); 

        Vector6 predicted_xl_k_plus_1(6);
        predicted_xl_k_plus_1 << next_pos, next_lin_vel, next_ang_vel;

        return (Vector(6) << xl_k_plus_1 - predicted_xl_k_plus_1).finished();
    }


    Vector evaluateError(const Vector6& xl_k, 
                         const Vector4& xr1_k,
                         const Vector1& tension1_k,
                         const Vector4& xr2_k,
                         const Vector1& tension2_k,
                         const Vector4& xr3_k,
                         const Vector1& tension3_k,
                         const Vector4& xr4_k,
                         const Vector1& tension4_k,
                         const Vector6& xl_k_plus_1,
                         gtsam::OptionalMatrixType H1,
                         gtsam::OptionalMatrixType H2,
                         gtsam::OptionalMatrixType H3,
                         gtsam::OptionalMatrixType H4,
                         gtsam::OptionalMatrixType H5,
                         gtsam::OptionalMatrixType H6,
                         gtsam::OptionalMatrixType H7,
                         gtsam::OptionalMatrixType H8,
                         gtsam::OptionalMatrixType H9,
                         gtsam::OptionalMatrixType H10) const override {

        Vector2 lin_vel_k(2);
        lin_vel_k << xl_k(3), xl_k(4);
        double v_norm = lin_vel_k.norm();
        Vector2 normed_vel_k = Vector2::Zero();
        double vx_dvx = 0.0;
        double vx_dvy = 0.0;
        double vy_dvx = 0.0;
        double vy_dvy = 0.0;
        if (v_norm > 1e-12) {
            normed_vel_k << lin_vel_k / v_norm;
            vx_dvx = 1.0 / v_norm - lin_vel_k(0) * lin_vel_k(0) / (v_norm * v_norm * v_norm);
            vx_dvy = -1.0 * lin_vel_k(0) * lin_vel_k(1) / (v_norm * v_norm * v_norm);

            vy_dvx = -1.0 * lin_vel_k(0) * lin_vel_k(1) / (v_norm * v_norm * v_norm);        
            vy_dvy = 1.0 / v_norm - lin_vel_k(1) * lin_vel_k(1) / (v_norm * v_norm * v_norm);
        }

        CableVectorHelper h1(xr1_k, xl_k);
        CableVectorHelper h2(xr2_k, xl_k);
        CableVectorHelper h3(xr3_k, xl_k);
        CableVectorHelper h4(xr4_k, xl_k);

        double r1 = -0.2 * cos(xl_k(2));
        double r2 = -0.2 * sin(xl_k(2));
        double r1_dtheta = 0.2 * sin(xl_k(2));
        double r2_dtheta = -0.2 * cos(xl_k(2));

        double sech_sq_theta = eps_ * (1.0 / cosh(eps_ * xl_k(5))) * (1.0 / cosh(eps_ * xl_k(5)));
        double A_CNST1 = dt_ / load_mass_ * tension1_k(0);
        double A_CNST2 = dt_ / load_mass_ * tension2_k(0);
        double A_CNST3 = dt_ / load_mass_ * tension3_k(0);
        double A_CNST4 = dt_ / load_mass_ * tension4_k(0);

        if (H1) {
                *H1 = (gtsam::Matrix(6, 6) << 
                    -1,  0,  0, -dt_,  0,   0,
                    0, -1,  0,   0, -dt_,  0,
                    0,  0, -1,   0,   0, -dt_,
                    -A_CNST1 * h1.ex_dxl -A_CNST2 * h2.ex_dxl -A_CNST3 * h3.ex_dxl -A_CNST4 * h4.ex_dxl,  -A_CNST1 * h1.ex_dyl -A_CNST2 * h2.ex_dyl -A_CNST3 * h3.ex_dyl -A_CNST4 * h4.ex_dyl,  -A_CNST1 * h1.ex_dtheta -A_CNST2 * h2.ex_dtheta -A_CNST3 * h3.ex_dtheta -A_CNST4 * h4.ex_dtheta,  -1 + dt_ * mu_ * g_ * vx_dvx,       dt_ * mu_ * g_ * vx_dvy,  0,
                    -A_CNST1 * h1.ey_dxl -A_CNST2 * h2.ey_dxl -A_CNST3 * h3.ey_dxl -A_CNST4 * h4.ey_dxl,  -A_CNST1 * h1.ey_dyl -A_CNST2 * h2.ey_dyl -A_CNST3 * h3.ey_dyl -A_CNST4 * h4.ey_dyl,  -A_CNST1 * h1.ey_dtheta -A_CNST2 * h2.ey_dtheta -A_CNST3 * h3.ey_dtheta -A_CNST4 * h4.ey_dtheta,       dt_ * mu_ * g_ * vy_dvx,  -1 + dt_ * mu_ * g_ * vy_dvy,  0,
                    // The last row of the matrix: each element on its own row for visibility
                    -dt_ / inertia_ * tension1_k(0) * (r1 * h1.ey_dxl - r2 * h1.ex_dxl) -dt_ / inertia_ * tension2_k(0) * (r1 * h2.ey_dxl - r2 * h2.ex_dxl) -dt_ / inertia_ * tension3_k(0) * (r1 * h3.ey_dxl - r2 * h3.ex_dxl) -dt_ / inertia_ * tension4_k(0) * (r1 * h4.ey_dxl - r2 * h4.ex_dxl), 
                    -dt_ / inertia_ * tension1_k(0) * (r1 * h1.ey_dyl - r2 * h1.ex_dyl) -dt_ / inertia_ * tension2_k(0) * (r1 * h2.ey_dyl - r2 * h2.ex_dyl) -dt_ / inertia_ * tension3_k(0) * (r1 * h3.ey_dyl - r2 * h3.ex_dyl) -dt_ / inertia_ * tension4_k(0) * (r1 * h4.ey_dyl - r2 * h4.ex_dyl),  
                    -dt_ / inertia_ * tension1_k(0) * (r1_dtheta * h1.e_norm(1) + r1 * h1.ey_dtheta - r2_dtheta * h1.e_norm(0) - r2 * h1.ex_dtheta) 
                    -dt_ / inertia_ * tension2_k(0) * (r1_dtheta * h2.e_norm(1) + r1 * h2.ey_dtheta - r2_dtheta * h2.e_norm(0) - r2 * h2.ex_dtheta)
                    -dt_ / inertia_ * tension3_k(0) * (r1_dtheta * h3.e_norm(1) + r1 * h3.ey_dtheta - r2_dtheta * h3.e_norm(0) - r2 * h3.ex_dtheta)
                    -dt_ / inertia_ * tension4_k(0) * (r1_dtheta * h4.e_norm(1) + r1 * h4.ey_dtheta - r2_dtheta * h4.e_norm(0) - r2 * h4.ex_dtheta), 
                    0, 
                    0, 
                    -1 + dt_ / inertia_ * (mu2_ * load_mass_ * g_ * sech_sq_theta)
                    ).finished();
        }
        if (H2) {
                *H2 = (gtsam::Matrix(6, 4) << 
                    0, 0,  0,  0,
                    0, 0,  0,  0,
                    0, 0,  0,  0,
                    -A_CNST1 * h1.ex_dxr,  -A_CNST1 * h1.ex_dyr, 0, 0,
                    -A_CNST1 * h1.ey_dxr,  -A_CNST1 * h1.ey_dyr,  0, 0,
                    // The last row of the matrix: each element on its own row for visibility
                    -dt_ / inertia_ * tension1_k(0) * (r1 * h1.ey_dxr - r2 * h1.ex_dxr), 
                    -dt_ / inertia_ * tension1_k(0) * (r1 * h1.ey_dyr - r2 * h1.ex_dyr), 
                    0, 
                    0
                    ).finished();
        }
        if (H3) {
            
                *H3 = (gtsam::Matrix(6, 1) << 
                    0,
                    0,
                    0,
                    -(dt_ * h1.e_norm(0) / load_mass_),
                    -(dt_ * h1.e_norm(1) / load_mass_),
                    -dt_ / inertia_ * (r1 * h1.e_norm(1) - r2 * h1.e_norm(0))
                    ).finished();
        }
        if (H4) {
            
                *H4 = (gtsam::Matrix(6, 4) << 
                    0, 0,  0,  0,
                    0, 0,  0,  0,
                    0, 0,  0,  0,
                    -A_CNST2 * h2.ex_dxr,  -A_CNST2 * h2.ex_dyr, 0, 0,
                    -A_CNST2 * h2.ey_dxr,  -A_CNST2 * h2.ey_dyr,  0, 0,
                    // The last row of the matrix: each element on its own row for visibility
                    -dt_ / inertia_ * tension2_k(0) * (r1 * h2.ey_dxr - r2 * h2.ex_dxr), 
                    -dt_ / inertia_ * tension2_k(0) * (r1 * h2.ey_dyr - r2 * h2.ex_dyr), 
                    0, 
                    0
                    ).finished();
        }
        if (H5) {
                *H5 = (gtsam::Matrix(6, 1) << 
                    0,
                    0,
                    0,
                    -(dt_ * h2.e_norm(0) / load_mass_),
                    -(dt_ * h2.e_norm(1) / load_mass_),
                    -dt_ / inertia_ * (r1 * h2.e_norm(1) - r2 * h2.e_norm(0))
                    ).finished();
        }
         if (H6) {
            
                *H6 = (gtsam::Matrix(6, 4) << 
                    0, 0,  0,  0,
                    0, 0,  0,  0,
                    0, 0,  0,  0,
                    -A_CNST3 * h3.ex_dxr,  -A_CNST3 * h3.ex_dyr, 0, 0,
                    -A_CNST3 * h3.ey_dxr,  -A_CNST3 * h3.ey_dyr,  0, 0,
                    // The last row of the matrix: each element on its own row for visibility
                    -dt_ / inertia_ * tension3_k(0) * (r1 * h3.ey_dxr - r2 * h3.ex_dxr), 
                    -dt_ / inertia_ * tension3_k(0) * (r1 * h3.ey_dyr - r2 * h3.ex_dyr), 
                    0, 
                    0
                    ).finished();
        }
        if (H7) {
                *H7 = (gtsam::Matrix(6, 1) << 
                    0,
                    0,
                    0,
                    -(dt_ * h3.e_norm(0) / load_mass_),
                    -(dt_ * h3.e_norm(1) / load_mass_),
                    -dt_ / inertia_ * (r1 * h3.e_norm(1) - r2 * h3.e_norm(0))
                    ).finished();
        }
        if (H8) {
            
                *H8 = (gtsam::Matrix(6, 4) << 
                    0, 0,  0,  0,
                    0, 0,  0,  0,
                    0, 0,  0,  0,
                    -A_CNST4 * h4.ex_dxr,  -A_CNST4 * h4.ex_dyr, 0, 0,
                    -A_CNST4 * h4.ey_dxr,  -A_CNST4 * h4.ey_dyr,  0, 0,
                    // The last row of the matrix: each element on its own row for visibility
                    -dt_ / inertia_ * tension4_k(0) * (r1 * h4.ey_dxr - r2 * h4.ex_dxr), 
                    -dt_ / inertia_ * tension4_k(0) * (r1 * h4.ey_dyr - r2 * h4.ex_dyr), 
                    0, 
                    0
                    ).finished();
        }
        if (H9) {
                *H9 = (gtsam::Matrix(6, 1) << 
                    0,
                    0,
                    0,
                    -(dt_ * h4.e_norm(0) / load_mass_),
                    -(dt_ * h4.e_norm(1) / load_mass_),
                    -dt_ / inertia_ * (r1 * h4.e_norm(1) - r2 * h4.e_norm(0))
                    ).finished();
        }
        if (H10) {
            *H10 = gtsam::Matrix6::Identity();
        }

        return evaluateErrorOnly(xl_k,  xr1_k, tension1_k,  xr2_k, tension2_k,  xr3_k, tension3_k, xr4_k, tension4_k, xl_k_plus_1);
    }
};


#endif // DYNAMICS_FACTORS_WITH_ANGLE_HPP