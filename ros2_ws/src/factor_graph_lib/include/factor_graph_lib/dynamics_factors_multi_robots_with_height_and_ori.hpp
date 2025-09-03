#ifndef DYNAMICS_FACTORS_MULTI_ROBOTS_WITH_HEIGHT_AND_ORI_HPP
#define DYNAMICS_FACTORS_MULTI_ROBOTS_WITH_HEIGHT_AND_ORI_HPP

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

class RobotDynamicsWithHeightAndOriFactor: public NoiseModelFactor5<Vector6, Vector6, Vector3, Vector1, Vector6> {
    double dt_;
    double robot_mass_;

public:
    RobotDynamicsWithHeightAndOriFactor(Key key_xr_k, Key key_xl_k, Key key_u_k, Key key_tension_k, Key key_xr_k_plus_1,
                        double dt, double robot_mass, const SharedNoiseModel& model) :
        NoiseModelFactor5<Vector6, Vector6, Vector3, Vector1, Vector6>(model, key_xr_k, key_xl_k, key_u_k, key_tension_k, key_xr_k_plus_1),
        dt_(dt), robot_mass_(robot_mass) {}

    Vector evaluateError(const Vector6& xr_k,
                         const Vector6& xl_k,
                         const Vector3& u_k, 
                         const Vector1& tension_k,
                         const Vector6& xr_k_plus_1,
                         gtsam::OptionalMatrixType H1,
                         gtsam::OptionalMatrixType H2,
                         gtsam::OptionalMatrixType H3,
                         gtsam::OptionalMatrixType H4,
                         gtsam::OptionalMatrixType H5) const override {
    
        // Unpack state: xr = [px, py, vx, vy]
        Vector3 pos_k = xr_k.head<3>();
        Vector3 vel_k = xr_k.tail<3>();

        CableVectorHelper h(xr_k, xl_k);

        Vector3 next_pos = pos_k + vel_k * dt_;
        Vector3 next_vel = vel_k + dt_ / robot_mass_ * (u_k - tension_k(0) * h.e3_norm);

        Vector6 predicted_xr_k_plus_1(6);
        predicted_xr_k_plus_1 << next_pos, next_vel;

        double A_CNST =  dt_ * tension_k(0) / robot_mass_;

        if (H1) {
            *H1 = (gtsam::Matrix(6, 6) << 
                -1,  0,  0, -dt_, 0,   0, 
                 0, -1,  0,  0,  -dt_, 0,
                 0,  0, -1,  0,   0,  -dt_,
                A_CNST * h.ex_dxr,  A_CNST * h.ex_dyr, A_CNST * h.ex_dzr, -1,   0,   0,
                A_CNST * h.ey_dxr,  A_CNST * h.ey_dyr, A_CNST * h.ey_dzr,  0,  -1,   0,
                A_CNST * h.ez_dxr,  A_CNST * h.ez_dyr, A_CNST * h.ez_dzr,  0,   0,  -1
            ).finished();
        }
        if (H2) {
            *H2 = (gtsam::Matrix(6, 6) << 
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                A_CNST * h.ex_dxl,  A_CNST * h.ex_dyl,  A_CNST * h.ex_dtheta,  0,  0, 0,
                A_CNST * h.ey_dxl,  A_CNST * h.ey_dyl,  A_CNST * h.ey_dtheta,  0,  0, 0,
                A_CNST * h.ez_dxl,  A_CNST * h.ez_dyl,  A_CNST * h.ez_dtheta,  0,  0, 0).finished();
        }
        if (H3) {
            *H3 = (gtsam::Matrix(6, 3) << 
                0, 0, 0,
                0, 0, 0, 
                0, 0, 0, 
                -(dt_/robot_mass_), 0,   0,
                0, -(dt_/robot_mass_),    0, 
                0,    0,   -(dt_/robot_mass_) 
            ).finished();
        }
        if (H4) {
            *H4 = (gtsam::Matrix(6, 1) << 
                0,
                0,
                0,
                (dt_ * h.e3_norm(0) / robot_mass_), 
                (dt_ * h.e3_norm(1) / robot_mass_),
                (dt_ * h.e3_norm(2) / robot_mass_)).finished();
        }
        if (H5) {
            *H5 = gtsam::Matrix6::Identity();
        }

        return (Vector(6) << xr_k_plus_1 - predicted_xr_k_plus_1).finished();
    }

};


class LoadDynamicsWithHeightAndOriFactor: public NoiseModelFactor4<Vector6, Vector6, Vector1, Vector6> {
    double dt_;
    double load_mass_;
    double mu_;
    double mu2_;
    double g_;
    double inertia_;
    double eps_ = 1000000.0;

public:
    LoadDynamicsWithHeightAndOriFactor(Key key_xl_k, Key key_xr_k, Key key_tension_k, Key key_xl_k_plus_1,
                        double dt, double load_mass, double mu, double mu2, double g, double inertia, const SharedNoiseModel& model) :
        NoiseModelFactor4<Vector6, Vector6, Vector1, Vector6>(model, key_xl_k, key_xr_k, key_tension_k, key_xl_k_plus_1),
        dt_(dt), load_mass_(load_mass), mu_(mu), mu2_(mu), g_(g), inertia_(inertia) {}

    Vector evaluateError(const Vector6& xl_k, 
                         const Vector6& xr_k,
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

        Vector3 pos_k = xl_k.head<3>();
        Vector3 vel_k = xl_k.tail<3>();
        Vector3 next_pos = pos_k + vel_k * dt_;

        CableVectorHelper h(xr_k, xl_k);
        Vector2 hor_e_norm(h.e3_norm(0), h.e3_norm(1));
        Vector2 next_lin_vel = lin_vel_k + dt_ / load_mass_ * (tension_k(0) * hor_e_norm - mu_ * (load_mass_ * g_ - tension_k(0) * h.e3_norm(2)) * normed_vel_k);

        double r1 = -0.2 * cos(xl_k(2));
        double r2 = -0.2 * sin(xl_k(2));

        Vector1 next_ang_vel(1);
        next_ang_vel(0) = xl_k(5) + dt_ / inertia_ * (r1 * tension_k(0) * h.e_norm(1) - r2 * tension_k(0) * h.e_norm(0) - mu2_ * load_mass_ * g_ * tanh(eps_ * xl_k(5))); 

        Vector6 predicted_xl_k_plus_1(6);
        predicted_xl_k_plus_1 << next_pos, next_lin_vel, next_ang_vel;
    

        double r1_dtheta = 0.2 * sin(xl_k(2));
        double r2_dtheta = -0.2 * cos(xl_k(2));
        double sech_sq_theta = eps_ * (1.0 / cosh(eps_ * xl_k(5))) * (1.0 / cosh(eps_ * xl_k(5)));
        double A_CNST = dt_ / load_mass_ * tension_k(0);
        Vector2 MUV = mu_ * normed_vel_k;

        if (H1) {
            *H1 = (gtsam::Matrix(6, 6) << 
                -1,  0,  0, -dt_,  0,   0,
                0, -1,  0,   0, -dt_,  0,
                0,  0, -1,   0,   0, -dt_,
                -A_CNST * (h.ex_dxl + MUV(0) * h.ez_dxl),  -A_CNST * (h.ex_dyl + MUV(0) * h.ez_dyl),  -A_CNST * (h.ex_dtheta + MUV(0) * h.ez_dtheta),  -1 + dt_ * mu_ * g_ * vx_dvx,       dt_ * mu_ * g_ * vx_dvy,  0,
                -A_CNST * (h.ey_dxl + MUV(1) * h.ez_dxl),  -A_CNST * (h.ey_dyl + MUV(1) * h.ez_dyl),  -A_CNST * (h.ey_dtheta + MUV(1) * h.ez_dtheta),       dt_ * mu_ * g_ * vy_dvx,  -1 + dt_ * mu_ * g_ * vy_dvy,  0,
                // The last row of the matrix: each element on its own row for visibility
                -dt_ / inertia_ * tension_k(0) * (r1 * h.ey_dxl - r2 * h.ex_dxl), 
                -dt_ / inertia_ * tension_k(0) * (r1 * h.ey_dyl - r2 * h.ex_dyl),  
                -dt_ / inertia_ * tension_k(0) * (r1_dtheta * h.e_norm(1) + r1 * h.ey_dtheta - r2_dtheta * h.e_norm(0) - r2 * h.ex_dtheta), 
                0, 
                0, 
                -1 + dt_ / inertia_ * (mu2_ * load_mass_ * g_ * sech_sq_theta)).finished();
        }
        if (H2) {
            *H2 = (gtsam::Matrix(6, 6) << 
                0,  0, 0, 0, 0, 0,
                0, 0,  0, 0, 0, 0,
                0, 0,  0, 0, 0, 0,
                -A_CNST * (h.ex_dxr + MUV(0) * h.ez_dxr),  -A_CNST * (h.ex_dyr + MUV(0) * h.ez_dyr), -A_CNST * (h.ex_dzr + MUV(0) * h.ez_dzr), 0, 0, 0,
                -A_CNST * (h.ey_dxr + MUV(1) * h.ez_dxr),  -A_CNST * (h.ey_dyr + MUV(1) * h.ez_dyr), -A_CNST * (h.ey_dzr + MUV(1) * h.ez_dzr), 0, 0, 0,
                // The last row of the matrix: each element on its own row for visibility
                -dt_ / inertia_ * tension_k(0) * (r1 * h.ey_dxr - r2 * h.ex_dxr), 
                -dt_ / inertia_ * tension_k(0) * (r1 * h.ey_dyr - r2 * h.ex_dyr), 
                -dt_ / inertia_ * tension_k(0) * (r1 * h.ey_dzr - r2 * h.ex_dzr), 
                0, 
                0,
                0).finished();
        }
        if (H3) {
            *H3 = (gtsam::Matrix(6, 1) << 
                0,
                0,
                0,
                -(dt_ / load_mass_ * (h.e3_norm(0) + MUV(0) * h.e3_norm(2)) ),
                -(dt_ / load_mass_ * (h.e3_norm(1) + MUV(1) * h.e3_norm(2)) ),
                -dt_ / inertia_ * (r1 * h.e_norm(1) - r2 * h.e_norm(0))).finished();
        }
        if (H4) {
            *H4 = gtsam::Matrix6::Identity();
        }

        return (Vector(6) << xl_k_plus_1 - predicted_xl_k_plus_1).finished();
    }
};


class LoadDynamicsMultiRobotsWithHeightAndOriFactor2: public NoiseModelFactor6<Vector6, Vector6, Vector1, Vector6, Vector1, Vector6> {
    double dt_;
    double load_mass_;
    double mu_;
    double mu2_;
    double g_;
    double inertia_;
    double eps_ = 1000000.0;

public:
    LoadDynamicsMultiRobotsWithHeightAndOriFactor2(
        Key key_xl_k, 
        Key key_xr1_k, 
        Key key_tension1_k, 
        Key key_xr2_k, 
        Key key_tension2_k,
        Key key_xl_k_plus_1,
        double dt, double load_mass, double mu, double mu2, double g, double inertia, const SharedNoiseModel& model) :
        NoiseModelFactor6<Vector6, Vector6, Vector1, Vector6, Vector1, Vector6>(model, key_xl_k, key_xr1_k, key_tension1_k, key_xr2_k, key_tension2_k, key_xl_k_plus_1),
        dt_(dt), load_mass_(load_mass), mu_(mu), mu2_(mu2), g_(g), inertia_(inertia) {}

    Vector evaluateError(const Vector6& xl_k, 
                         const Vector6& xr1_k,
                         const Vector1& tension1_k,
                         const Vector6& xr2_k,
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

        Vector3 pos_k = xl_k.head<3>();
        Vector3 vel_k = xl_k.tail<3>();
        Vector3 next_pos = pos_k + vel_k * dt_;

        CableVectorHelper h1(xr1_k, xl_k);
        CableVectorHelper h2(xr2_k, xl_k);

        Vector2 hor1_e_norm(h1.e3_norm(0), h1.e3_norm(1));
        Vector2 hor2_e_norm(h2.e3_norm(0), h2.e3_norm(1));
        Vector2 next_lin_vel = lin_vel_k + dt_ / load_mass_ * (tension1_k(0) * hor1_e_norm + tension2_k(0) * hor2_e_norm - mu_ * (load_mass_ * g_ - tension1_k(0) * h1.e3_norm(2) - tension2_k(0) * h2.e3_norm(2)) * normed_vel_k);

        double r1 = -0.2 * cos(xl_k(2));
        double r2 = -0.2 * sin(xl_k(2));

        Vector1 next_ang_vel(1);
        next_ang_vel(0) = xl_k(5) + dt_ / inertia_ * (
            r1 * tension1_k(0) * h1.e_norm(1) - r2 * tension1_k(0) * h1.e_norm(0) + 
            r1 * tension2_k(0) * h2.e_norm(1) - r2 * tension2_k(0) * h2.e_norm(0) - mu2_ * load_mass_ * g_ * tanh(eps_ * xl_k(5))); 

        Vector6 predicted_xl_k_plus_1(6);
        predicted_xl_k_plus_1 << next_pos, next_lin_vel, next_ang_vel;


        double r1_dtheta = 0.2 * sin(xl_k(2));
        double r2_dtheta = -0.2 * cos(xl_k(2));
        double sech_sq_theta = eps_ * (1.0 / cosh(eps_ * xl_k(5))) * (1.0 / cosh(eps_ * xl_k(5)));
        double A_CNST1 = dt_ * tension1_k(0) / load_mass_;
        double A_CNST2 = dt_ * tension2_k(0) / load_mass_;
        Vector2 MUV = mu_ * normed_vel_k;

        if (H1) {
            *H1 = (gtsam::Matrix(6, 6) << 
                -1,  0,  0, -dt_,  0,   0,
                0, -1,  0,   0, -dt_,  0,
                0,  0, -1,   0,   0, -dt_,
                -A_CNST1 * (h1.ex_dxl + MUV(0) * h1.ez_dxl) -A_CNST2 * (h2.ex_dxl + MUV(0) * h2.ez_dxl),  -A_CNST1 * (h1.ex_dyl + MUV(0) * h1.ez_dyl) -A_CNST2 * (h2.ex_dyl + MUV(0) * h2.ez_dyl),  -A_CNST1 * (h1.ex_dtheta + MUV(0) * h1.ez_dtheta) -A_CNST2 * (h2.ex_dtheta + MUV(0) * h2.ez_dtheta),  -1 + dt_ * mu_ * g_ * vx_dvx,       dt_ * mu_ * g_ * vx_dvy,  0,
                -A_CNST1 * (h1.ey_dxl + MUV(1) * h1.ez_dxl) -A_CNST2 * (h2.ey_dxl + MUV(1) * h2.ez_dxl),  -A_CNST1 * (h1.ey_dyl + MUV(1) * h1.ez_dyl) -A_CNST2 * (h2.ey_dyl + MUV(1) * h2.ez_dyl),  -A_CNST1 * (h1.ey_dtheta + MUV(1) * h1.ez_dtheta) -A_CNST2 * (h2.ey_dtheta + MUV(1) * h2.ez_dtheta),       dt_ * mu_ * g_ * vy_dvx,  -1 + dt_ * mu_ * g_ * vy_dvy,  0,
                // The last row of the matrix: each element on its own row for visibility
                -dt_ / inertia_ * (tension1_k(0) * (r1 * h1.ey_dxl - r2 * h1.ex_dxl) + tension2_k(0) * (r1 * h2.ey_dxl - r2 * h2.ex_dxl)), 
                -dt_ / inertia_ * (tension1_k(0) * (r1 * h1.ey_dyl - r2 * h1.ex_dyl) + tension2_k(0) * (r1 * h2.ey_dyl - r2 * h2.ex_dyl)),  
                -dt_ / inertia_ * (tension1_k(0) * (r1_dtheta * h1.e_norm(1) + r1 * h1.ey_dtheta - r2_dtheta * h1.e_norm(0) - r2 * h1.ex_dtheta) + tension2_k(0) * (r1_dtheta * h2.e_norm(1) + r1 * h2.ey_dtheta - r2_dtheta * h2.e_norm(0) - r2 * h2.ex_dtheta)), 
                0, 
                0, 
                -1 + dt_ / inertia_ * (mu2_ * load_mass_ * g_ * sech_sq_theta)).finished();

            // *H1 = (gtsam::Matrix(4, 4) << 
            //     -1,  0, -dt_, 0,
            //     0, -1,  0,  -dt_,
            //     A_CNST1 * h1.ex_dxl + A_CNST2 * h2.ex_dxl,  A_CNST1 * h1.ex_dyl + A_CNST2 * h2.ex_dyl, -1 + dt_ * mu_ * g_ * SEVEN, dt_ * mu_ * g_ * NINE,
            //     A_CNST1 * h1.ey_dxl + A_CNST2 * h2.ey_dxl,  A_CNST1 * h1.ey_dyl + A_CNST2 * h2.ey_dyl,  dt_ * mu_ * g_ * NINE,  -1 + dt_ * mu_ * g_ * EIGHT).finished();
        }
        if (H2) {
            *H2 = (gtsam::Matrix(6, 6) << 
                0,  0, 0, 0, 0, 0,
                0, 0,  0, 0, 0, 0,
                0, 0,  0, 0, 0, 0,
                -A_CNST1 * (h1.ex_dxr + MUV(0) * h1.ez_dxr),  -A_CNST1 * (h1.ex_dyr + MUV(0) * h1.ez_dyr), -A_CNST1 * (h1.ex_dzr + MUV(0) * h1.ez_dzr), 0, 0, 0,
                -A_CNST1 * (h1.ey_dxr + MUV(1) * h1.ez_dxr),  -A_CNST1 * (h1.ey_dyr + MUV(1) * h1.ez_dyr), -A_CNST1 * (h1.ey_dzr + MUV(1) * h1.ez_dzr), 0, 0, 0,
                // The last row of the matrix: each element on its own row for visibility
                -dt_ / inertia_ * tension1_k(0) * (r1 * h1.ey_dxr - r2 * h1.ex_dxr), 
                -dt_ / inertia_ * tension1_k(0) * (r1 * h1.ey_dyr - r2 * h1.ex_dyr), 
                -dt_ / inertia_ * tension1_k(0) * (r1 * h1.ey_dzr - r2 * h1.ex_dzr), 
                0, 
                0,
                0).finished();

            // *H2 = (gtsam::Matrix(4, 6) << 
            //     0,  0, 0, 0, 0,0,
            //     0, 0,  0,  0,0,0,
            //     A_CNST1 * (h1.ex_dxr + h1.ez_dxr),  A_CNST1 * (h1.ex_dyr + h1.ez_dyr), A_CNST1 * (h1.ex_dzr + h1.ez_dzr), 0, 0, 0,
            //     A_CNST1 * (h1.ey_dxr + h1.ez_dxr),  A_CNST1 * (h1.ey_dyr + h1.ez_dyr), A_CNST1 * (h1.ey_dzr + h1.ez_dzr), 0, 0, 0).finished();
        }
        if (H3) {
           *H3 = (gtsam::Matrix(6, 1) << 
                0,
                0,
                0,
                -(dt_ / load_mass_ * (h1.e3_norm(0) + MUV(0) * h1.e3_norm(2)) ),
                -(dt_ / load_mass_ * (h1.e3_norm(1) + MUV(1) * h1.e3_norm(2)) ),
                -dt_ / inertia_ * (r1 * h1.e_norm(1) - r2 * h1.e_norm(0))).finished();
        }
        if (H4) {
            *H4 = (gtsam::Matrix(6, 6) << 
                0,  0, 0, 0, 0, 0,
                0, 0,  0, 0, 0, 0,
                0, 0,  0, 0, 0, 0,
                -A_CNST2 * (h2.ex_dxr + MUV(0) * h2.ez_dxr),  -A_CNST2 * (h2.ex_dyr + MUV(0) * h2.ez_dyr), -A_CNST2 * (h2.ex_dzr + MUV(0) * h2.ez_dzr), 0, 0, 0,
                -A_CNST2 * (h2.ey_dxr + MUV(1) * h2.ez_dxr),  -A_CNST2 * (h2.ey_dyr + MUV(1) * h2.ez_dyr), -A_CNST2 * (h2.ey_dzr + MUV(1) * h2.ez_dzr), 0, 0, 0,
                // The last row of the matrix: each element on its own row for visibility
                -dt_ / inertia_ * tension2_k(0) * (r1 * h2.ey_dxr - r2 * h2.ex_dxr), 
                -dt_ / inertia_ * tension2_k(0) * (r1 * h2.ey_dyr - r2 * h2.ex_dyr), 
                -dt_ / inertia_ * tension2_k(0) * (r1 * h2.ey_dzr - r2 * h2.ex_dzr), 
                0, 
                0,
                0).finished();
        }
        if (H5) {
            *H5 = (gtsam::Matrix(6, 1) << 
                0,
                0,
                0,
                -(dt_ / load_mass_ * (h2.e3_norm(0) + MUV(0) * h2.e3_norm(2)) ),
                -(dt_ / load_mass_ * (h2.e3_norm(1) + MUV(1) * h2.e3_norm(2)) ),
                -dt_ / inertia_ * (r1 * h2.e_norm(1) - r2 * h2.e_norm(0))).finished();
        }
        if (H6) {
            *H6 = gtsam::Matrix6::Identity();
        }

        return (Vector(6) << xl_k_plus_1 - predicted_xl_k_plus_1).finished();
    }
};


class LoadDynamicsMultiRobotsWithHeightAndOriFactor3: public NoiseModelFactorN<Vector6, Vector6, Vector1, Vector6, Vector1, Vector6, Vector1, Vector6> {
    double dt_;
    double load_mass_;
    double mu_;
    double mu2_;
    double g_;
    double inertia_;
    double eps_ = 1000000.0;

public:
    LoadDynamicsMultiRobotsWithHeightAndOriFactor3(
        Key key_xl_k, 
        Key key_xr1_k, 
        Key key_tension1_k, 
        Key key_xr2_k, 
        Key key_tension2_k,
        Key key_xr3_k, 
        Key key_tension3_k,
        Key key_xl_k_plus_1,
        double dt, double load_mass, double mu, double mu2, double g, double inertia, const SharedNoiseModel& model) :
        NoiseModelFactorN<Vector6, Vector6, Vector1, Vector6, Vector1, Vector6, Vector1, Vector6>(model, key_xl_k, key_xr1_k, key_tension1_k, key_xr2_k, key_tension2_k, key_xr3_k, key_tension3_k, key_xl_k_plus_1),
        dt_(dt), load_mass_(load_mass), mu_(mu), mu2_(mu2), g_(g), inertia_(inertia) {}

    Vector evaluateError(const Vector6& xl_k, 
                         const Vector6& xr1_k,
                         const Vector1& tension1_k,
                         const Vector6& xr2_k,
                         const Vector1& tension2_k,
                         const Vector6& xr3_k,
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

        Vector3 pos_k = xl_k.head<3>();
        Vector3 vel_k = xl_k.tail<3>();
        Vector3 next_pos = pos_k + vel_k * dt_;

        CableVectorHelper h1(xr1_k, xl_k);
        CableVectorHelper h2(xr2_k, xl_k);
        CableVectorHelper h3(xr3_k, xl_k);

        Vector2 hor1_e_norm(h1.e3_norm(0), h1.e3_norm(1));
        Vector2 hor2_e_norm(h2.e3_norm(0), h2.e3_norm(1));
        Vector2 hor3_e_norm(h3.e3_norm(0), h3.e3_norm(1));
        Vector2 next_lin_vel = lin_vel_k + dt_ / load_mass_ * (
            tension1_k(0) * hor1_e_norm + 
            tension2_k(0) * hor2_e_norm +
            tension3_k(0) * hor3_e_norm - mu_ * (
                load_mass_ * g_ 
                - tension1_k(0) * h1.e3_norm(2) 
                - tension2_k(0) * h2.e3_norm(2)
                - tension3_k(0) * h3.e3_norm(2)) * normed_vel_k);

        double r1 = -0.2 * cos(xl_k(2));
        double r2 = -0.2 * sin(xl_k(2));

        Vector1 next_ang_vel(1);
        next_ang_vel(0) = xl_k(5) + dt_ / inertia_ * (
            r1 * tension1_k(0) * h1.e_norm(1) - r2 * tension1_k(0) * h1.e_norm(0) + 
            r1 * tension2_k(0) * h2.e_norm(1) - r2 * tension2_k(0) * h2.e_norm(0) +
            r1 * tension3_k(0) * h3.e_norm(1) - r2 * tension3_k(0) * h3.e_norm(0) - mu2_ * load_mass_ * g_ * tanh(eps_ * xl_k(5))); 

        Vector6 predicted_xl_k_plus_1(6);
        predicted_xl_k_plus_1 << next_pos, next_lin_vel, next_ang_vel;


        double r1_dtheta = 0.2 * sin(xl_k(2));
        double r2_dtheta = -0.2 * cos(xl_k(2));
        double sech_sq_theta = eps_ * (1.0 / cosh(eps_ * xl_k(5))) * (1.0 / cosh(eps_ * xl_k(5)));
        double A_CNST1 = dt_ * tension1_k(0) / load_mass_;
        double A_CNST2 = dt_ * tension2_k(0) / load_mass_;
        double A_CNST3 = dt_ * tension3_k(0) / load_mass_;
        Vector2 MUV = mu_ * normed_vel_k;

        if (H1) {
            *H1 = (gtsam::Matrix(6, 6) << 
                -1,  0,  0, -dt_,  0,   0,
                0, -1,  0,   0, -dt_,  0,
                0,  0, -1,   0,   0, -dt_,
                -A_CNST1 * (h1.ex_dxl + MUV(0) * h1.ez_dxl) -A_CNST2 * (h2.ex_dxl + MUV(0) * h2.ez_dxl) -A_CNST3 * (h3.ex_dxl + MUV(0) * h3.ez_dxl),  -A_CNST1 * (h1.ex_dyl + MUV(0) * h1.ez_dyl) -A_CNST2 * (h2.ex_dyl + MUV(0) * h2.ez_dyl) -A_CNST3 * (h3.ex_dyl + MUV(0) * h3.ez_dyl),  -A_CNST1 * (h1.ex_dtheta + MUV(0) * h1.ez_dtheta) -A_CNST2 * (h2.ex_dtheta + MUV(0) * h2.ez_dtheta) -A_CNST3 * (h3.ex_dtheta + MUV(0) * h3.ez_dtheta),  -1 + dt_ * mu_ * g_ * vx_dvx,       dt_ * mu_ * g_ * vx_dvy,  0,
                -A_CNST1 * (h1.ey_dxl + MUV(1) * h1.ez_dxl) -A_CNST2 * (h2.ey_dxl + MUV(1) * h2.ez_dxl) -A_CNST3 * (h3.ey_dxl + MUV(1) * h3.ez_dxl),  -A_CNST1 * (h1.ey_dyl + MUV(1) * h1.ez_dyl) -A_CNST2 * (h2.ey_dyl + MUV(1) * h2.ez_dyl) -A_CNST3 * (h3.ey_dyl + MUV(1) * h3.ez_dyl),  -A_CNST1 * (h1.ey_dtheta + MUV(1) * h1.ez_dtheta) -A_CNST2 * (h2.ey_dtheta + MUV(1) * h2.ez_dtheta) -A_CNST3 * (h3.ey_dtheta + MUV(1) * h3.ez_dtheta),       dt_ * mu_ * g_ * vy_dvx,  -1 + dt_ * mu_ * g_ * vy_dvy,  0,
                // The last row of the matrix: each element on its own row for visibility
                -dt_ / inertia_ * (tension1_k(0) * (r1 * h1.ey_dxl - r2 * h1.ex_dxl) + tension2_k(0) * (r1 * h2.ey_dxl - r2 * h2.ex_dxl) + tension3_k(0) * (r1 * h3.ey_dxl - r2 * h3.ex_dxl)), 
                -dt_ / inertia_ * (tension1_k(0) * (r1 * h1.ey_dyl - r2 * h1.ex_dyl) + tension2_k(0) * (r1 * h2.ey_dyl - r2 * h2.ex_dyl) + tension3_k(0) * (r1 * h3.ey_dyl - r2 * h3.ex_dyl)),  
                -dt_ / inertia_ * (tension1_k(0) * (r1_dtheta * h1.e_norm(1) + r1 * h1.ey_dtheta - r2_dtheta * h1.e_norm(0) - r2 * h1.ex_dtheta) + tension2_k(0) * (r1_dtheta * h2.e_norm(1) + r1 * h2.ey_dtheta - r2_dtheta * h2.e_norm(0) - r2 * h2.ex_dtheta) + tension3_k(0) * (r1_dtheta * h3.e_norm(1) + r1 * h3.ey_dtheta - r2_dtheta * h3.e_norm(0) - r2 * h3.ex_dtheta)), 
                0, 
                0, 
                -1 + dt_ / inertia_ * (mu2_ * load_mass_ * g_ * sech_sq_theta)).finished();
        }
        if (H2) {
            *H2 = (gtsam::Matrix(6, 6) << 
                0,  0, 0, 0, 0, 0,
                0, 0,  0, 0, 0, 0,
                0, 0,  0, 0, 0, 0,
                -A_CNST1 * (h1.ex_dxr + MUV(0) * h1.ez_dxr),  -A_CNST1 * (h1.ex_dyr + MUV(0) * h1.ez_dyr), -A_CNST1 * (h1.ex_dzr + MUV(0) * h1.ez_dzr), 0, 0, 0,
                -A_CNST1 * (h1.ey_dxr + MUV(1) * h1.ez_dxr),  -A_CNST1 * (h1.ey_dyr + MUV(1) * h1.ez_dyr), -A_CNST1 * (h1.ey_dzr + MUV(1) * h1.ez_dzr), 0, 0, 0,
                // The last row of the matrix: each element on its own row for visibility
                -dt_ / inertia_ * tension1_k(0) * (r1 * h1.ey_dxr - r2 * h1.ex_dxr), 
                -dt_ / inertia_ * tension1_k(0) * (r1 * h1.ey_dyr - r2 * h1.ex_dyr), 
                -dt_ / inertia_ * tension1_k(0) * (r1 * h1.ey_dzr - r2 * h1.ex_dzr), 
                0, 
                0,
                0).finished();
        }
        if (H3) {
           *H3 = (gtsam::Matrix(6, 1) << 
                0,
                0,
                0,
                -(dt_ / load_mass_ * (h1.e3_norm(0) + MUV(0) * h1.e3_norm(2)) ),
                -(dt_ / load_mass_ * (h1.e3_norm(1) + MUV(1) * h1.e3_norm(2)) ),
                -dt_ / inertia_ * (r1 * h1.e_norm(1) - r2 * h1.e_norm(0))).finished();
        }
        if (H4) {
            *H4 = (gtsam::Matrix(6, 6) << 
                0,  0, 0, 0, 0, 0,
                0, 0,  0, 0, 0, 0,
                0, 0,  0, 0, 0, 0,
                -A_CNST2 * (h2.ex_dxr + MUV(0) * h2.ez_dxr),  -A_CNST2 * (h2.ex_dyr + MUV(0) * h2.ez_dyr), -A_CNST2 * (h2.ex_dzr + MUV(0) * h2.ez_dzr), 0, 0, 0,
                -A_CNST2 * (h2.ey_dxr + MUV(1) * h2.ez_dxr),  -A_CNST2 * (h2.ey_dyr + MUV(1) * h2.ez_dyr), -A_CNST2 * (h2.ey_dzr + MUV(1) * h2.ez_dzr), 0, 0, 0,
                // The last row of the matrix: each element on its own row for visibility
                -dt_ / inertia_ * tension2_k(0) * (r1 * h2.ey_dxr - r2 * h2.ex_dxr), 
                -dt_ / inertia_ * tension2_k(0) * (r1 * h2.ey_dyr - r2 * h2.ex_dyr), 
                -dt_ / inertia_ * tension2_k(0) * (r1 * h2.ey_dzr - r2 * h2.ex_dzr), 
                0, 
                0,
                0).finished();
        }
        if (H5) {
            *H5 = (gtsam::Matrix(6, 1) << 
                0,
                0,
                0,
                -(dt_ / load_mass_ * (h2.e3_norm(0) + MUV(0) * h2.e3_norm(2)) ),
                -(dt_ / load_mass_ * (h2.e3_norm(1) + MUV(1) * h2.e3_norm(2)) ),
                -dt_ / inertia_ * (r1 * h2.e_norm(1) - r2 * h2.e_norm(0))).finished();
        }
        if (H6) {
            *H6 = (gtsam::Matrix(6, 6) << 
                0,  0, 0, 0, 0, 0,
                0, 0,  0, 0, 0, 0,
                0, 0,  0, 0, 0, 0,
                -A_CNST3 * (h3.ex_dxr + MUV(0) * h3.ez_dxr),  -A_CNST3 * (h3.ex_dyr + MUV(0) * h3.ez_dyr), -A_CNST3 * (h3.ex_dzr + MUV(0) * h3.ez_dzr), 0, 0, 0,
                -A_CNST3 * (h3.ey_dxr + MUV(1) * h3.ez_dxr),  -A_CNST3 * (h3.ey_dyr + MUV(1) * h3.ez_dyr), -A_CNST3 * (h3.ey_dzr + MUV(1) * h3.ez_dzr), 0, 0, 0,
                // The last row of the matrix: each element on its own row for visibility
                -dt_ / inertia_ * tension3_k(0) * (r1 * h3.ey_dxr - r2 * h3.ex_dxr), 
                -dt_ / inertia_ * tension3_k(0) * (r1 * h3.ey_dyr - r2 * h3.ex_dyr), 
                -dt_ / inertia_ * tension3_k(0) * (r1 * h3.ey_dzr - r2 * h3.ex_dzr), 
                0, 
                0,
                0).finished();
        }
        if (H7) {
            *H7 = (gtsam::Matrix(6, 1) << 
                0,
                0,
                0,
                -(dt_ / load_mass_ * (h3.e3_norm(0) + MUV(0) * h3.e3_norm(2)) ),
                -(dt_ / load_mass_ * (h3.e3_norm(1) + MUV(1) * h3.e3_norm(2)) ),
                -dt_ / inertia_ * (r1 * h3.e_norm(1) - r2 * h3.e_norm(0))).finished();
        }
        if (H8) {
            *H8 = gtsam::Matrix6::Identity();
        }

        return (Vector(6) << xl_k_plus_1 - predicted_xl_k_plus_1).finished();
    }
};


class LoadDynamicsMultiRobotsWithHeightAndOriFactor4: public NoiseModelFactorN<Vector6, Vector6, Vector1, Vector6, Vector1, Vector6, Vector1, Vector6, Vector1, Vector6> {
    double dt_;
    double load_mass_;
    double mu_;
    double mu2_;
    double g_;
    double inertia_;
    double eps_ = 1000000.0;

public:
    LoadDynamicsMultiRobotsWithHeightAndOriFactor4(
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
        double dt, double load_mass, double mu, double mu2, double g, double inertia, const SharedNoiseModel& model) :
        NoiseModelFactorN<Vector6, Vector6, Vector1, Vector6, Vector1, Vector6, Vector1, Vector6, Vector1, Vector6>(
            model, key_xl_k, 
            key_xr1_k, key_tension1_k, 
            key_xr2_k, key_tension2_k, 
            key_xr3_k, key_tension3_k, 
            key_xr4_k, key_tension4_k, 
            key_xl_k_plus_1),
        dt_(dt), load_mass_(load_mass), mu_(mu), mu2_(mu2), g_(g), inertia_(inertia) {}

    Vector evaluateError(const Vector6& xl_k, 
                         const Vector6& xr1_k,
                         const Vector1& tension1_k,
                         const Vector6& xr2_k,
                         const Vector1& tension2_k,
                         const Vector6& xr3_k,
                         const Vector1& tension3_k,
                         const Vector6& xr4_k,
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

        Vector3 pos_k = xl_k.head<3>();
        Vector3 vel_k = xl_k.tail<3>();
        Vector3 next_pos = pos_k + vel_k * dt_;

        CableVectorHelper h1(xr1_k, xl_k);
        CableVectorHelper h2(xr2_k, xl_k);
        CableVectorHelper h3(xr3_k, xl_k);
        CableVectorHelper h4(xr4_k, xl_k);

        Vector2 hor1_e_norm(h1.e3_norm(0), h1.e3_norm(1));
        Vector2 hor2_e_norm(h2.e3_norm(0), h2.e3_norm(1));
        Vector2 hor3_e_norm(h3.e3_norm(0), h3.e3_norm(1));
        Vector2 hor4_e_norm(h4.e3_norm(0), h4.e3_norm(1));
        Vector2 next_lin_vel = lin_vel_k + dt_ / load_mass_ * (
            tension1_k(0) * hor1_e_norm + 
            tension2_k(0) * hor2_e_norm +
            tension3_k(0) * hor3_e_norm +
            tension4_k(0) * hor4_e_norm 
            - mu_ * (
                load_mass_ * g_ 
                - tension1_k(0) * h1.e3_norm(2) 
                - tension2_k(0) * h2.e3_norm(2)
                - tension3_k(0) * h3.e3_norm(2)
                - tension4_k(0) * h4.e3_norm(2)) * normed_vel_k);

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


        double r1_dtheta = 0.2 * sin(xl_k(2));
        double r2_dtheta = -0.2 * cos(xl_k(2));
        double sech_sq_theta = eps_ * (1.0 / cosh(eps_ * xl_k(5))) * (1.0 / cosh(eps_ * xl_k(5)));
        double A_CNST1 = dt_ * tension1_k(0) / load_mass_;
        double A_CNST2 = dt_ * tension2_k(0) / load_mass_;
        double A_CNST3 = dt_ * tension3_k(0) / load_mass_;
        double A_CNST4 = dt_ * tension4_k(0) / load_mass_;
        Vector2 MUV = mu_ * normed_vel_k;

        if (H1) {
            *H1 = (gtsam::Matrix(6, 6) << 
                -1,  0,  0, -dt_,  0,   0,
                0, -1,  0,   0, -dt_,  0,
                0,  0, -1,   0,   0, -dt_,
                // row 4
                -A_CNST1 * (h1.ex_dxl + MUV(0) * h1.ez_dxl) -A_CNST2 * (h2.ex_dxl + MUV(0) * h2.ez_dxl) -A_CNST3 * (h3.ex_dxl + MUV(0) * h3.ez_dxl) -A_CNST4 * (h4.ex_dxl + MUV(0) * h4.ez_dxl),  
                -A_CNST1 * (h1.ex_dyl + MUV(0) * h1.ez_dyl) -A_CNST2 * (h2.ex_dyl + MUV(0) * h2.ez_dyl) -A_CNST3 * (h3.ex_dyl + MUV(0) * h3.ez_dyl) -A_CNST4 * (h4.ex_dyl + MUV(0) * h4.ez_dyl),  
                -A_CNST1 * (h1.ex_dtheta + MUV(0) * h1.ez_dtheta) -A_CNST2 * (h2.ex_dtheta + MUV(0) * h2.ez_dtheta) -A_CNST3 * (h3.ex_dtheta + MUV(0) * h3.ez_dtheta) -A_CNST4 * (h4.ex_dtheta + MUV(0) * h4.ez_dtheta),  
                -1 + dt_ * mu_ * g_ * vx_dvx,       dt_ * mu_ * g_ * vx_dvy,  0,
                // row 5
                -A_CNST1 * (h1.ey_dxl + MUV(1) * h1.ez_dxl) -A_CNST2 * (h2.ey_dxl + MUV(1) * h2.ez_dxl) -A_CNST3 * (h3.ey_dxl + MUV(1) * h3.ez_dxl) -A_CNST4 * (h4.ey_dxl + MUV(1) * h4.ez_dxl),  
                -A_CNST1 * (h1.ey_dyl + MUV(1) * h1.ez_dyl) -A_CNST2 * (h2.ey_dyl + MUV(1) * h2.ez_dyl) -A_CNST3 * (h3.ey_dyl + MUV(1) * h3.ez_dyl) -A_CNST4 * (h4.ey_dyl + MUV(1) * h4.ez_dyl),  
                -A_CNST1 * (h1.ey_dtheta + MUV(1) * h1.ez_dtheta) -A_CNST2 * (h2.ey_dtheta + MUV(1) * h2.ez_dtheta) -A_CNST3 * (h3.ey_dtheta + MUV(1) * h3.ez_dtheta) -A_CNST4 * (h4.ey_dtheta + MUV(1) * h4.ez_dtheta),       
                dt_ * mu_ * g_ * vy_dvx,  -1 + dt_ * mu_ * g_ * vy_dvy,  0,
                // The last row of the matrix: each element on its own row for visibility
                -dt_ / inertia_ * (tension1_k(0) * (r1 * h1.ey_dxl - r2 * h1.ex_dxl) + tension2_k(0) * (r1 * h2.ey_dxl - r2 * h2.ex_dxl) + tension3_k(0) * (r1 * h3.ey_dxl - r2 * h3.ex_dxl) + tension4_k(0) * (r1 * h4.ey_dxl - r2 * h4.ex_dxl)), 
                -dt_ / inertia_ * (tension1_k(0) * (r1 * h1.ey_dyl - r2 * h1.ex_dyl) + tension2_k(0) * (r1 * h2.ey_dyl - r2 * h2.ex_dyl) + tension3_k(0) * (r1 * h3.ey_dyl - r2 * h3.ex_dyl) + tension4_k(0) * (r1 * h4.ey_dyl - r2 * h4.ex_dyl)),  
                -dt_ / inertia_ * (tension1_k(0) * (r1_dtheta * h1.e_norm(1) + r1 * h1.ey_dtheta - r2_dtheta * h1.e_norm(0) - r2 * h1.ex_dtheta) + tension2_k(0) * (r1_dtheta * h2.e_norm(1) + r1 * h2.ey_dtheta - r2_dtheta * h2.e_norm(0) - r2 * h2.ex_dtheta) + tension3_k(0) * (r1_dtheta * h3.e_norm(1) + r1 * h3.ey_dtheta - r2_dtheta * h3.e_norm(0) - r2 * h3.ex_dtheta) + tension4_k(0) * (r1_dtheta * h4.e_norm(1) + r1 * h4.ey_dtheta - r2_dtheta * h4.e_norm(0) - r2 * h4.ex_dtheta)), 
                0, 
                0, 
                -1 + dt_ / inertia_ * (mu2_ * load_mass_ * g_ * sech_sq_theta)).finished();
        }
        if (H2) {
            *H2 = (gtsam::Matrix(6, 6) << 
                0,  0, 0, 0, 0, 0,
                0, 0,  0, 0, 0, 0,
                0, 0,  0, 0, 0, 0,
                -A_CNST1 * (h1.ex_dxr + MUV(0) * h1.ez_dxr),  -A_CNST1 * (h1.ex_dyr + MUV(0) * h1.ez_dyr), -A_CNST1 * (h1.ex_dzr + MUV(0) * h1.ez_dzr), 0, 0, 0,
                -A_CNST1 * (h1.ey_dxr + MUV(1) * h1.ez_dxr),  -A_CNST1 * (h1.ey_dyr + MUV(1) * h1.ez_dyr), -A_CNST1 * (h1.ey_dzr + MUV(1) * h1.ez_dzr), 0, 0, 0,
                // The last row of the matrix: each element on its own row for visibility
                -dt_ / inertia_ * tension1_k(0) * (r1 * h1.ey_dxr - r2 * h1.ex_dxr), 
                -dt_ / inertia_ * tension1_k(0) * (r1 * h1.ey_dyr - r2 * h1.ex_dyr), 
                -dt_ / inertia_ * tension1_k(0) * (r1 * h1.ey_dzr - r2 * h1.ex_dzr), 
                0, 
                0,
                0).finished();
        }
        if (H3) {
           *H3 = (gtsam::Matrix(6, 1) << 
                0,
                0,
                0,
                -(dt_ / load_mass_ * (h1.e3_norm(0) + MUV(0) * h1.e3_norm(2)) ),
                -(dt_ / load_mass_ * (h1.e3_norm(1) + MUV(1) * h1.e3_norm(2)) ),
                -dt_ / inertia_ * (r1 * h1.e_norm(1) - r2 * h1.e_norm(0))).finished();
        }
        if (H4) {
            *H4 = (gtsam::Matrix(6, 6) << 
                0,  0, 0, 0, 0, 0,
                0, 0,  0, 0, 0, 0,
                0, 0,  0, 0, 0, 0,
                -A_CNST2 * (h2.ex_dxr + MUV(0) * h2.ez_dxr),  -A_CNST2 * (h2.ex_dyr + MUV(0) * h2.ez_dyr), -A_CNST2 * (h2.ex_dzr + MUV(0) * h2.ez_dzr), 0, 0, 0,
                -A_CNST2 * (h2.ey_dxr + MUV(1) * h2.ez_dxr),  -A_CNST2 * (h2.ey_dyr + MUV(1) * h2.ez_dyr), -A_CNST2 * (h2.ey_dzr + MUV(1) * h2.ez_dzr), 0, 0, 0,
                // The last row of the matrix: each element on its own row for visibility
                -dt_ / inertia_ * tension2_k(0) * (r1 * h2.ey_dxr - r2 * h2.ex_dxr), 
                -dt_ / inertia_ * tension2_k(0) * (r1 * h2.ey_dyr - r2 * h2.ex_dyr), 
                -dt_ / inertia_ * tension2_k(0) * (r1 * h2.ey_dzr - r2 * h2.ex_dzr), 
                0, 
                0,
                0).finished();
        }
        if (H5) {
            *H5 = (gtsam::Matrix(6, 1) << 
                0,
                0,
                0,
                -(dt_ / load_mass_ * (h2.e3_norm(0) + MUV(0) * h2.e3_norm(2)) ),
                -(dt_ / load_mass_ * (h2.e3_norm(1) + MUV(1) * h2.e3_norm(2)) ),
                -dt_ / inertia_ * (r1 * h2.e_norm(1) - r2 * h2.e_norm(0))).finished();
        }
        if (H6) {
            *H6 = (gtsam::Matrix(6, 6) << 
                0,  0, 0, 0, 0, 0,
                0, 0,  0, 0, 0, 0,
                0, 0,  0, 0, 0, 0,
                -A_CNST3 * (h3.ex_dxr + MUV(0) * h3.ez_dxr),  -A_CNST3 * (h3.ex_dyr + MUV(0) * h3.ez_dyr), -A_CNST3 * (h3.ex_dzr + MUV(0) * h3.ez_dzr), 0, 0, 0,
                -A_CNST3 * (h3.ey_dxr + MUV(1) * h3.ez_dxr),  -A_CNST3 * (h3.ey_dyr + MUV(1) * h3.ez_dyr), -A_CNST3 * (h3.ey_dzr + MUV(1) * h3.ez_dzr), 0, 0, 0,
                // The last row of the matrix: each element on its own row for visibility
                -dt_ / inertia_ * tension3_k(0) * (r1 * h3.ey_dxr - r2 * h3.ex_dxr), 
                -dt_ / inertia_ * tension3_k(0) * (r1 * h3.ey_dyr - r2 * h3.ex_dyr), 
                -dt_ / inertia_ * tension3_k(0) * (r1 * h3.ey_dzr - r2 * h3.ex_dzr), 
                0, 
                0,
                0).finished();
        }
        if (H7) {
            *H7 = (gtsam::Matrix(6, 1) << 
                0,
                0,
                0,
                -(dt_ / load_mass_ * (h3.e3_norm(0) + MUV(0) * h3.e3_norm(2)) ),
                -(dt_ / load_mass_ * (h3.e3_norm(1) + MUV(1) * h3.e3_norm(2)) ),
                -dt_ / inertia_ * (r1 * h3.e_norm(1) - r2 * h3.e_norm(0))).finished();
        }
        if (H8) {
            *H8 = (gtsam::Matrix(6, 6) << 
                0,  0, 0, 0, 0, 0,
                0, 0,  0, 0, 0, 0,
                0, 0,  0, 0, 0, 0,
                -A_CNST4 * (h4.ex_dxr + MUV(0) * h4.ez_dxr),  -A_CNST4 * (h4.ex_dyr + MUV(0) * h4.ez_dyr), -A_CNST4 * (h4.ex_dzr + MUV(0) * h4.ez_dzr), 0, 0, 0,
                -A_CNST4 * (h4.ey_dxr + MUV(1) * h4.ez_dxr),  -A_CNST4 * (h4.ey_dyr + MUV(1) * h4.ez_dyr), -A_CNST4 * (h4.ey_dzr + MUV(1) * h4.ez_dzr), 0, 0, 0,
                // The last row of the matrix: each element on its own row for visibility
                -dt_ / inertia_ * tension4_k(0) * (r1 * h4.ey_dxr - r2 * h4.ex_dxr), 
                -dt_ / inertia_ * tension4_k(0) * (r1 * h4.ey_dyr - r2 * h4.ex_dyr), 
                -dt_ / inertia_ * tension4_k(0) * (r1 * h4.ey_dzr - r2 * h4.ex_dzr), 
                0, 
                0,
                0).finished();
        }
        if (H9) {
            *H9 = (gtsam::Matrix(6, 1) << 
                0,
                0,
                0,
                -(dt_ / load_mass_ * (h4.e3_norm(0) + MUV(0) * h4.e3_norm(2)) ),
                -(dt_ / load_mass_ * (h4.e3_norm(1) + MUV(1) * h4.e3_norm(2)) ),
                -dt_ / inertia_ * (r1 * h4.e_norm(1) - r2 * h4.e_norm(0))).finished();
        }
        if (H10) {
            *H10 = gtsam::Matrix6::Identity();
        }

        return (Vector(6) << xl_k_plus_1 - predicted_xl_k_plus_1).finished();
    }
};


#endif // DYNAMICS_FACTORS_MULTI_ROBOTS_WITH_HEIGHT_AND_ORI_HPP