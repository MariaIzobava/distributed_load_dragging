

#ifndef DYNAMICS_FACTORS_WITH_HEIGHT_HPP
#define DYNAMICS_FACTORS_WITH_HEIGHT_HPP

#include <string>
#include <algorithm>

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>

#include "common.hpp"

using namespace std;
using namespace gtsam;


class RobotsHeightUpperBoundFactor : public NoiseModelFactor1<Vector6> {
private:
    double max_magnitude_;
    double e_ = 1e-6;
public:
    RobotsHeightUpperBoundFactor(Key key, 
                              double maxMagnitude,
                              const SharedNoiseModel& model)
        : NoiseModelFactor1<Vector6>(model, key), max_magnitude_(maxMagnitude) {}


    Vector evaluateError(const Vector6& xr,
                         OptionalMatrixType H) const override {

        double current_magnitude = xr[2];
        double x = current_magnitude - max_magnitude_;

        double err = 0.5 * (x + std::sqrt(x * x + e_ * e_));


        if (H) {
            double de_dx = 0.5 * (1.0 + x / std::sqrt(x * x + e_ * e_));

            if (current_magnitude > 1e-9) {
                (*H) = (gtsam::Matrix(1, 6) << 0, 0, de_dx, 0, 0, 0).finished();
            } else {
                (*H) = (gtsam::Matrix(1, 6) << 0, 0, 0, 0, 0, 0).finished();
            }
        }
        return (Vector(1) << err).finished();
    }
};

class RobotsHeightLowerBoundFactor : public NoiseModelFactor1<Vector6> {
private:
    double min_magnitude_;
    double e_ = 1e-6;
public:
    RobotsHeightLowerBoundFactor(Key key, 
                              double minMagnitude,
                              const SharedNoiseModel& model)
        : NoiseModelFactor1<Vector6>(model, key), min_magnitude_(minMagnitude) {}

    Vector evaluateError(const Vector6& xr,
                         OptionalMatrixType H) const override {

        double current_magnitude = xr[2];
        double x = min_magnitude_ - current_magnitude;

        double err = 0.5 * (x + std::sqrt(x * x + e_ * e_));


        if (H) {
            double de_dx = 0.5 * (1.0 + x / std::sqrt(x * x + e_ * e_));
            double dx_dmagnitude = -1.0;

            if (current_magnitude > 1e-9) {
                (*H) = (gtsam::Matrix(1, 6) << 0, 0, de_dx * dx_dmagnitude, 0, 0, 0).finished();
            } else {
                (*H) = (gtsam::Matrix(1, 6) << 0, 0, 0, 0, 0, 0).finished();
            }
        }
        return (Vector(1) << err).finished();
    }
};


class RobotsDistanceWithHeightFactor: public NoiseModelFactor2<Vector6, Vector6> {
    double limit_;

    double smooth_max_zero_(double x, double epsilon) const {
        return 0.5 * (x + std::sqrt(x*x + epsilon*epsilon));
    }

public:
    RobotsDistanceWithHeightFactor(Key key_xr1_k, Key key_xr2_k, double limit, const SharedNoiseModel& model) :
        NoiseModelFactor2<Vector6, Vector6>(model, key_xr1_k, key_xr2_k), limit_(limit) {}

    Vector evaluateError(const Vector6& xr1_k, const Vector6& xr2_k,
                        gtsam::OptionalMatrixType H1,
                        gtsam::OptionalMatrixType H2) const override {
    
        Vector3 diff(xr1_k[0] - xr2_k[0], xr1_k[1] - xr2_k[1], xr1_k[2] - xr2_k[2]);
        double dist = diff.norm();

        double epsilon = 1e-9;
        double err = smooth_max_zero_(limit_ - dist, epsilon);

        double inner_term = limit_ - dist;
        double deriv_smooth_max_wrt_inner = 0.5 * (1.0 + inner_term / std::sqrt(inner_term*inner_term + epsilon*epsilon));
        Vector3 partial_deriv_distance_r1 = -diff / dist; 
        Vector3 partial_deriv_distance_r2 = diff / dist; 

        if (H1) {
            *H1 = (gtsam::Matrix(1, 6) << deriv_smooth_max_wrt_inner * partial_deriv_distance_r1(0), deriv_smooth_max_wrt_inner * partial_deriv_distance_r1(1), deriv_smooth_max_wrt_inner * partial_deriv_distance_r1(2), 0, 0, 0
            ).finished();
        }

        if (H2) {
            *H2 = (gtsam::Matrix(1, 6) << deriv_smooth_max_wrt_inner * partial_deriv_distance_r2(0), deriv_smooth_max_wrt_inner * partial_deriv_distance_r2(1), deriv_smooth_max_wrt_inner * partial_deriv_distance_r2(2), 0, 0, 0
            ).finished();
        }

        return (Vector(1) << err).finished();
    }
};


/**
 * Custom factor to model the robot's dynamics.
 * Connects Xr_k, U_k, T_k, Xr_{k+1}
 * Error = Xr_{k+1} - (Xr_k + dt * f(Xr_k, U_k, T_k))
 */
class RobotDynamicsWithHeightFactor: public NoiseModelFactor5<Vector6, Vector4, Vector3, Vector1, Vector6> {
    double dt_;
    double robot_mass_;

public:
    RobotDynamicsWithHeightFactor(Key key_xr_k, Key key_xl_k, Key key_u_k, Key key_tension_k, Key key_xr_k_plus_1,
                        double dt, double robot_mass, const SharedNoiseModel& model) :
        NoiseModelFactor5<Vector6, Vector4, Vector3, Vector1, Vector6>(model, key_xr_k, key_xl_k, key_u_k, key_tension_k, key_xr_k_plus_1),
        dt_(dt), robot_mass_(robot_mass) {}

    Vector evaluateError(const Vector6& xr_k,
                         const Vector4& xl_k,
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

        CableVectorHelper h(xr_k, xl_k, "middle");

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
            *H2 = (gtsam::Matrix(6, 4) << 
                0, 0, 0, 0,
                0, 0, 0, 0,
                0, 0, 0, 0,
                A_CNST * h.ex_dxl,  A_CNST * h.ex_dyl, 0,   0,
                A_CNST * h.ey_dxl,  A_CNST * h.ey_dyl,  0,  0,
                A_CNST * h.ez_dxl,  A_CNST * h.ez_dyl,  0,  0).finished();
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

class LoadDynamicsWithHeightFactor: public NoiseModelFactor4<Vector4, Vector6, Vector1, Vector4> {
    double dt_;
    double load_mass_;
    double mu_;
    double g_;

public:
    LoadDynamicsWithHeightFactor(Key key_xl_k, Key key_xr_k, Key key_tension_k, Key key_xl_k_plus_1,
                        double dt, double load_mass, double mu, double g, const SharedNoiseModel& model) :
        NoiseModelFactor4<Vector4, Vector6, Vector1, Vector4>(model, key_xl_k, key_xr_k, key_tension_k, key_xl_k_plus_1),
        dt_(dt), load_mass_(load_mass), mu_(mu), g_(g) {}

    Vector evaluateError(const Vector4& xl_k, 
                         const Vector6& xr_k,
                         const Vector1& tension_k,
                         const Vector4& xl_k_plus_1,
                         gtsam::OptionalMatrixType H1,
                         gtsam::OptionalMatrixType H2,
                         gtsam::OptionalMatrixType H3,
                         gtsam::OptionalMatrixType H4) const override {
        
        Vector2 pos_k = xl_k.head<2>();
        Vector2 vel_k = xl_k.tail<2>();
        double v_norm = vel_k.norm();
        Vector2 normed_vel_k = Vector2::Zero();
        double SEVEN = 0.0;
        double EIGHT = 0.0;
        double NINE = 0.0;
        if (v_norm > 1e-12) {
            normed_vel_k << vel_k / v_norm;
            SEVEN = 1.0 / v_norm - vel_k(0) * vel_k(0) / (v_norm * v_norm * v_norm);
            EIGHT = 1.0 / v_norm - vel_k(1) * vel_k(1) / (v_norm * v_norm * v_norm);
            NINE = -1.0 * vel_k(0) * vel_k(1) / (v_norm * v_norm * v_norm);
        }

        CableVectorHelper h(xr_k, xl_k, "middle");

        Vector2 next_pos = pos_k + vel_k * dt_;
        Vector2 hor_e_norm(h.e3_norm(0), h.e3_norm(1));
        Vector2 next_vel = vel_k +  dt_ / load_mass_ * (tension_k(0) * hor_e_norm - mu_ * (load_mass_ * g_ - tension_k(0) * h.e3_norm(2)) * normed_vel_k);

        Vector4 predicted_xl_k_plus_1(4);
        predicted_xl_k_plus_1 << next_pos, next_vel;


        double A_CNST = -dt_ * tension_k(0) / load_mass_;
        Vector2 MUV = mu_ * normed_vel_k;

        if (H1) {
            *H1 = (gtsam::Matrix(4, 4) << 
                // -1,  0, -dt_, 0,
                // 0, -1,  0,  -dt_,
                // A_CNST * h.ex_dxl,  A_CNST * h.ex_dyl, -1 + dt_ * mu_ * g_ * SEVEN, dt_ * mu_ * g_ * NINE,
                // A_CNST * h.ey_dxl,  A_CNST * h.ey_dyl,  dt_ * mu_ * g_ * NINE,  -1 + dt_ * mu_ * g_ * EIGHT).finished();
                -1,  0, -dt_, 0,
                0, -1,  0,  -dt_,
                A_CNST * (h.ex_dxl + MUV(0) * h.ez_dxl),  A_CNST * (h.ex_dyl + MUV(0) * h.ez_dyl), -1 + dt_ * mu_ * g_ * SEVEN, dt_ * mu_ * g_ * NINE,
                A_CNST * (h.ey_dxl + MUV(1) * h.ez_dxl),  A_CNST * (h.ey_dyl + MUV(1) * h.ez_dyl),  dt_ * mu_ * g_ * NINE,  -1 + dt_ * mu_ * g_ * EIGHT).finished();
        }
        if (H2) {
            // *H2 = (gtsam::Matrix(4, 6) << 
            //     0,  0, 0, 0, 0,0,
            //     0, 0,  0,  0,0,0,
            //     A_CNST * (h.ex_dxr + h.ez_dxr),  A_CNST * (h.ex_dyr + h.ez_dyr), A_CNST * (h.ex_dzr + h.ez_dzr), 0, 0, 0,
            //     A_CNST * (h.ey_dxr + h.ez_dxr),  A_CNST * (h.ey_dyr + h.ez_dyr), A_CNST * (h.ey_dzr + h.ez_dzr), 0, 0, 0).finished();
            *H2 = (gtsam::Matrix(4, 6) << 
                0,  0, 0, 0, 0,0,
                0, 0,  0,  0,0,0,
                A_CNST * (h.ex_dxr + MUV(0) * h.ez_dxr),  A_CNST * (h.ex_dyr + MUV(0) * h.ez_dyr), A_CNST * (h.ex_dzr + MUV(0) * h.ez_dzr), 0, 0, 0,
                A_CNST * (h.ey_dxr + MUV(1) * h.ez_dxr),  A_CNST * (h.ey_dyr + MUV(1) * h.ez_dyr), A_CNST * (h.ey_dzr + MUV(1) * h.ez_dzr), 0, 0, 0).finished();
        }
        if (H3) {
            // *H3 = (gtsam::Matrix(4, 1) << 
            //     0,
            //     0,
            //     -(dt_ * h.e3_norm(0) / load_mass_),
            //     -(dt_ * h.e3_norm(1) / load_mass_)).finished();
            *H3 = (gtsam::Matrix(4, 1) << 
                0,
                0,
                -(dt_ / load_mass_ * (h.e3_norm(0) + MUV(0) * h.e3_norm(2)) ),
                -(dt_ / load_mass_ * (h.e3_norm(1) + MUV(1) * h.e3_norm(2)) )).finished();
        }
        if (H4) {
            *H4 = gtsam::Matrix4::Identity();
        }

        return (Vector(4) << xl_k_plus_1 - predicted_xl_k_plus_1).finished();
    }
};



#endif // DYNAMICS_FACTORS_WITH_HEIGHT_HPP