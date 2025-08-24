#ifndef MULTI_DYNAMICS_FACTORS_HPP
#define MULTI_DYNAMICS_FACTORS_HPP

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


class LoadDynamicsMultiRobotsWithLoadOrientationFactor4: public NoiseModelFactorN<Vector6, Vector4, Vector1, Vector4, Vector1, Vector4, Vector1, Vector4, Vector1, Vector6> {
    double dt_;
    double load_mass_;
    double mu_;
    double mu2_;
    double inertia_;
    double g_;
    double eps_ = 1000000.0;

public:
    LoadDynamicsMultiRobotsWithLoadOrientationFactor4(
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
            key_xr1_k, key_tension1_k, key_xr2_k, key_tension2_k, 
            key_xr3_k, key_tension3_k, key_xr4_k, key_tension4_k, key_xl_k_plus_1),
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

        Vector2 next_lin_vel = lin_vel_k + dt_ / load_mass_ * (tension1_k(0) * h1.e_norm + tension2_k(0) * h2.e_norm + tension3_k(0) * h3.e_norm + tension4_k(0) * h4.e_norm - mu_ * load_mass_ * g_ * normed_vel_k);

        double r1 = -0.2 * cos(xl_k(2));
        double r2 = -0.2 * sin(xl_k(2));

        double t1 = r1 * tension1_k(0) * h1.e_norm(1) - r2 * tension1_k(0) * h1.e_norm(0);
        double t2 = r1 * tension2_k(0) * h2.e_norm(1) - r2 * tension2_k(0) * h2.e_norm(0);
        double t3 = r1 * tension3_k(0) * h3.e_norm(1) - r2 * tension3_k(0) * h3.e_norm(0);
        double t4 = r1 * tension4_k(0) * h4.e_norm(1) - r2 * tension4_k(0) * h4.e_norm(0);

        Vector1 next_ang_vel(1);
        next_ang_vel(0) = xl_k(5) + dt_ / inertia_ * (t1 + t2 + t3 + t4 - mu2_ * load_mass_ * g_ * tanh(eps_ * xl_k(5))); 

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

        std::vector<Vector4> xr_k = {xr1_k, xr2_k, xr3_k, xr4_k};
        std::vector<double> tension_k = {tension1_k(0), tension2_k(0), tension3_k(0), tension4_k(0)};
        std::vector<gtsam::OptionalMatrixType> H = {H2, H3, H4, H5, H6, H7, H8, H9};

        std::vector<CableVectorHelper> h;
        std::vector<double> A_CNST;
        for (int i = 0; i < 4; i++) {
            h.push_back(CableVectorHelper(xr_k[i], xl_k));
            A_CNST.push_back(dt_ / load_mass_ * tension_k[i]);
        }

        double r1 = -0.2 * cos(xl_k(2));
        double r2 = -0.2 * sin(xl_k(2));
        double r1_dtheta = 0.2 * sin(xl_k(2));
        double r2_dtheta = -0.2 * cos(xl_k(2));
        double sech_sq_theta = eps_ * (1.0 / cosh(eps_ * xl_k(5))) * (1.0 / cosh(eps_ * xl_k(5)));

        if (H1) {
            
            Vector3 xl3_dxl(3), xl4_dxl(3), xl5_dxl(3);
            for (int i = 0; i < 4; i++) {
                Vector3 ex(-A_CNST[i] * h[i].ex_dxl, -A_CNST[i] * h[i].ex_dyl, -A_CNST[i] * h[i].ex_dtheta);
                Vector3 ey(-A_CNST[i] * h[i].ey_dxl, -A_CNST[i] * h[i].ey_dyl, -A_CNST[i] * h[i].ey_dtheta);
                Vector3 et(
                -dt_ / inertia_ * tension_k[i] * (r1 * h[i].ey_dxl - r2 * h[i].ex_dxl), 
                -dt_ / inertia_ * tension_k[i] * (r1 * h[i].ey_dyl - r2 * h[i].ex_dyl),
                -dt_ / inertia_ * tension_k[i] * (r1_dtheta * h[i].e_norm(1) + r1 * h[i].ey_dtheta - r2_dtheta * h[i].e_norm(0) - r2 * h[i].ex_dtheta));
                
                xl3_dxl = xl3_dxl + ex;
                xl4_dxl = xl4_dxl + ey;
                xl5_dxl = xl5_dxl + et;
            }
            
            *H1 = (gtsam::Matrix(6, 6) << 
                -1,  0,  0, -dt_,  0,   0,
                0, -1,  0,   0, -dt_,  0,
                0,  0, -1,   0,   0, -dt_,
                xl3_dxl(0), xl3_dxl(1), xl3_dxl(2), -1 + dt_ * mu_ * g_ * vx_dvx,       dt_ * mu_ * g_ * vx_dvy,  0,
                xl4_dxl(0), xl4_dxl(1), xl4_dxl(2),      dt_ * mu_ * g_ * vy_dvx,  -1 + dt_ * mu_ * g_ * vy_dvy,  0,
                xl5_dxl(0), xl5_dxl(1), xl5_dxl(2),      0,                             0,                       -1 + dt_ / inertia_ * (mu2_ * load_mass_ * g_ * sech_sq_theta)
                ).finished();

        }
        for (int i = 0; i < 4; i++) {
            if (H[i * 2]) {
                *(H[i * 2]) = (gtsam::Matrix(6, 4) << 
                    0, 0,  0,  0,
                    0, 0,  0,  0,
                    0, 0,  0,  0,
                    -A_CNST[i] * h[i].ex_dxr,  -A_CNST[i] * h[i].ex_dyr, 0, 0,
                    -A_CNST[i] * h[i].ey_dxr,  -A_CNST[i] * h[i].ey_dyr,  0, 0,
                    // The last row of the matrix: each element on its own row for visibility
                    -dt_ / inertia_ * tension_k[i] * (r1 * h[i].ey_dxr - r2 * h[i].ex_dxr), 
                    -dt_ / inertia_ * tension_k[i] * (r1 * h[i].ey_dyr - r2 * h[i].ex_dyr), 
                    0, 
                    0
                    ).finished();
                
            }
            if (H[i * 2 + 1]) {
                    *(H[i * 2 + 1]) = (gtsam::Matrix(6, 1) << 
                        0,
                        0,
                        0,
                        -(dt_ * h[i].e_norm(0) / load_mass_),
                        -(dt_ * h[i].e_norm(1) / load_mass_),
                        -dt_ / inertia_ * (r1 * h[i].e_norm(1) - r2 * h[i].e_norm(0))
                        ).finished();
            }
        }
        if (H10) {
            *H10 = gtsam::Matrix6::Identity();
        }

        return evaluateErrorOnly(xl_k,  xr1_k, tension1_k,  xr2_k, tension2_k, xr3_k, tension3_k,  xr4_k, tension4_k, xl_k_plus_1);

    }
};

#endif // MULTI_DYNAMICS_FACTORS_HPP