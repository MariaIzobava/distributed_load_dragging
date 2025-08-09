#ifndef DYNAMICS_FACTORS_HPP
#define DYNAMICS_FACTORS_HPP

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


class VelocityLowerBound: public NoiseModelFactor1<Vector4> {
    double limit_;

public:
    VelocityLowerBound(Key key_xr_k, double limit, const SharedNoiseModel& model) :
        NoiseModelFactor1<Vector4>(model, key_xr_k), limit_(limit) {}

    Vector evaluateError(const Vector4& xr_k,
                         gtsam::OptionalMatrixType H1) const override {

        Vector4 err(0, 0, min(0.0, xr_k[2] - limit_), min(0.0, xr_k[3] - limit_));

        if (H1) {
            int k1 = (xr_k[2] < limit_) ? 1 : 0;
            int k2 = (xr_k[3] < limit_) ? 1 : 0;
            *H1 = (gtsam::Matrix(4, 4) << 
                0, 0, 0, 0,
                0, 0, 0, 0,
                0, 0, k1, 0,
                0, 0, 0, k2
            ).finished();
        }

        return err;
    }
};


class VelocityUpperBound: public NoiseModelFactor1<Vector4> {
    double limit_;

public:
    VelocityUpperBound(Key key_xr_k, double limit, const SharedNoiseModel& model) :
        NoiseModelFactor1<Vector4>(model, key_xr_k), limit_(limit) {}

    Vector evaluateError(const Vector4& xr_k,
                         gtsam::OptionalMatrixType H1) const override {
    
        Vector4 err(0, 0, max(0.0, xr_k[2] - limit_), max(0.0, xr_k[3] - limit_));

        if (H1) {
            int k1 = (xr_k[2] > limit_) ? 1 : 0;
            int k2 = (xr_k[3] > limit_) ? 1 : 0;
            *H1 = (gtsam::Matrix(4, 4) << 
                0, 0, 0, 0,
                0, 0, 0, 0,
                0, 0, k1, 0,
                0, 0, 0, k2
            ).finished();
        }

        return err;
    }
};

/**
 * Custom factor to model the robot's dynamics.
 * Connects Xr_k, U_k, T_k, Xr_{k+1}
 * Error = Xr_{k+1} - (Xr_k + dt * f(Xr_k, U_k, T_k))
 */
class RobotDynamicsFactor: public NoiseModelFactor5<Vector4, Vector4, Vector2, Vector1, Vector4> {
    double dt_;
    double robot_mass_;

public:
    RobotDynamicsFactor(Key key_xr_k, Key key_xl_k, Key key_u_k, Key key_tension_k, Key key_xr_k_plus_1,
                        double dt, double robot_mass, const SharedNoiseModel& model) :
        NoiseModelFactor5<Vector4, Vector4, Vector2, Vector1, Vector4>(model, key_xr_k, key_xl_k, key_u_k, key_tension_k, key_xr_k_plus_1),
        dt_(dt), robot_mass_(robot_mass) {}

    // The evaluateError function, which implements the factor's error calculation.
    Vector evaluateError(const Vector4& xr_k,
                         const Vector4& xl_k,
                         const Vector2& u_k, 
                         const Vector1& tension_k,
                         const Vector4& xr_k_plus_1,
                         gtsam::OptionalMatrixType H1,
                         gtsam::OptionalMatrixType H2,
                         gtsam::OptionalMatrixType H3,
                         gtsam::OptionalMatrixType H4,
                         gtsam::OptionalMatrixType H5) const override {
    
        // Unpack state: xr = [px, py, vx, vy]
        Vector2 pos_k = xr_k.head<2>();
        Vector2 vel_k = xr_k.tail<2>();

        CableVectorHelper h(xr_k, xl_k);

        Vector2 next_pos = pos_k + vel_k * dt_;
        Vector2 next_vel = vel_k + dt_ / robot_mass_ * (u_k - tension_k(0) * h.e_norm);

        Vector4 predicted_xr_k_plus_1(4);
        predicted_xr_k_plus_1 << next_pos, next_vel;

        double A_CNST =  dt_ * tension_k(0) / robot_mass_;

        if (H1) {
            // H1 is a valid reference to a matrix, so you can assign to it.
            *H1 = (gtsam::Matrix(4, 4) << 
                -1,  0, -dt_, 0,
                0, -1,  0,  -dt_,
                A_CNST * h.ex_dxr,  A_CNST * h.ex_dyr, -1,   0,
                A_CNST * h.ey_dxr,  A_CNST * h.ey_dyr,  0,  -1).finished();

        }
        if (H2) {
            *H2 = (gtsam::Matrix(4, 4) << 
                0, 0, 0, 0,
                0, 0, 0, 0,
                A_CNST * h.ex_dxl,  A_CNST * h.ex_dyl, 0,   0,
                A_CNST * h.ey_dxl,  A_CNST * h.ey_dyl,  0,  0).finished();

        }

        if (H3) {
            *H3 = (gtsam::Matrix(4, 2) << 
                0, 0, 
                0, 0,  
                -(dt_/robot_mass_), 0, 
                0, -(dt_/robot_mass_)).finished();
        }
        if (H4) {
            *H4 = (gtsam::Matrix(4, 1) << 
                0,
                0,
                (dt_ * h.e_norm(0) / robot_mass_), 
                (dt_ * h.e_norm(1) / robot_mass_)).finished();
        }
        if (H5) {
            *H5 = gtsam::Matrix4::Identity();
        }

        return (Vector(4) << xr_k_plus_1 - predicted_xr_k_plus_1).finished();
    }

};

class LoadDynamicsFactor: public NoiseModelFactor4<Vector4, Vector4, Vector1, Vector4> {
    double dt_;
    double load_mass_;
    double mu_;
    double g_;

public:
    LoadDynamicsFactor(Key key_xl_k, Key key_xr_k, Key key_tension_k, Key key_xl_k_plus_1,
                        double dt, double load_mass, double mu, double g, const SharedNoiseModel& model) :
        NoiseModelFactor4<Vector4, Vector4, Vector1, Vector4>(model, key_xl_k, key_xr_k, key_tension_k, key_xl_k_plus_1),
        dt_(dt), load_mass_(load_mass), mu_(mu), g_(g) {}

    Vector evaluateError(const Vector4& xl_k, 
                         const Vector4& xr_k,
                         const Vector1& tension_k,
                         const Vector4& xl_k_plus_1,
                         gtsam::OptionalMatrixType H1,
                         gtsam::OptionalMatrixType H2,
                         gtsam::OptionalMatrixType H3,
                         gtsam::OptionalMatrixType H4) const override {
        
        // Unpack state: xr = [px, py, vx, vy]
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

        CableVectorHelper h(xr_k, xl_k);

        Vector2 next_pos = pos_k + vel_k * dt_;
        Vector2 next_vel = vel_k + (tension_k(0) * h.e_norm - mu_ * load_mass_ * g_ * normed_vel_k) / load_mass_ * dt_;

        Vector4 predicted_xl_k_plus_1(4);
        predicted_xl_k_plus_1 << next_pos, next_vel;


        double A_CNST = -dt_ * tension_k(0) / load_mass_;

        if (H1) {
            *H1 = (gtsam::Matrix(4, 4) << 
                -1,  0, -dt_, 0,
                0, -1,  0,  -dt_,
                A_CNST * h.ex_dxl,  A_CNST * h.ex_dyl, -1 + dt_ * mu_ * g_ * SEVEN, dt_ * mu_ * g_ * NINE,
                A_CNST * h.ey_dxl,  A_CNST * h.ey_dyl,  dt_ * mu_ * g_ * NINE,  -1 + dt_ * mu_ * g_ * EIGHT).finished();
        }
        if (H2) {
            *H2 = (gtsam::Matrix(4, 4) << 
                0,  0, 0, 0,
                0, 0,  0,  0,
                A_CNST * h.ex_dxr,  A_CNST * h.ex_dyr, 0, 0,
                A_CNST * h.ey_dxr,  A_CNST * h.ey_dyr, 0, 0).finished();
        }
        if (H3) {
            *H3 = (gtsam::Matrix(4, 1) << 
                0,
                0,
                -(dt_ * h.e_norm(0) / load_mass_),
                -(dt_ * h.e_norm(1) / load_mass_)).finished();
        }
        if (H4) {
            *H4 = gtsam::Matrix4::Identity();
        }

        return (Vector(4) << xl_k_plus_1 - predicted_xl_k_plus_1).finished();
    }
};


class LoadDynamicsTwoRobotsFactor: public NoiseModelFactor6<Vector4, Vector4, Vector1, Vector4, Vector1, Vector4> {
    double dt_;
    double load_mass_;
    double mu_;
    double g_;

public:
    LoadDynamicsTwoRobotsFactor(Key key_xl_k, Key key_xr1_k, Key key_tension1_k, Key key_xr2_k, Key key_tension2_k, Key key_xl_k_plus_1,
                        double dt, double load_mass, double mu, double g, const SharedNoiseModel& model) :
        NoiseModelFactor6<Vector4, Vector4, Vector1, Vector4, Vector1, Vector4>(model, key_xl_k, key_xr1_k, key_tension1_k, key_xr2_k, key_tension2_k, key_xl_k_plus_1),
        dt_(dt), load_mass_(load_mass), mu_(mu), g_(g) {}

    Vector evaluateErrorOnly(const Vector4& xl_k, 
                         const Vector4& xr1_k,
                         const Vector1& tension1_k,
                         const Vector4& xr2_k,
                         const Vector1& tension2_k,
                         const Vector4& xl_k_plus_1) const {

        // Unpack state: xr = [px, py, vx, vy]
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

        // Simple Euler integration for dynamics
        // next_pos = current_pos + dt * current_vel
        // next_vel = current_vel + dt * (control_force/mass - tension_force/mass)
        Vector2 next_pos = pos_k + vel_k * dt_;

        double xd1 = xr1_k(0) - xl_k(0);
        double yd1 = xr1_k(1) - xl_k(1);
        Vector2 e1(2);
        e1 << xd1, yd1;
        double norm1 = e1.norm();
        Vector2 e_norm1(2);
        e_norm1 << e1 / e1.norm();

        double xd2 = xr2_k(0) - xl_k(0);
        double yd2 = xr2_k(1) - xl_k(1);
        Vector2 e2(2);
        e2 << xd2, yd2;
        double norm2 = e2.norm();
        Vector2 e_norm2(2);
        e_norm2 << e2 / e2.norm();

        Vector2 next_vel = vel_k + (tension1_k(0) * e_norm1 + tension2_k(0) * e_norm2 - mu_ * load_mass_ * g_ * normed_vel_k) / load_mass_ * dt_;

        Vector4 predicted_xl_k_plus_1(4);
        predicted_xl_k_plus_1 << next_pos, next_vel;

        return (Vector(4) << xl_k_plus_1 - predicted_xl_k_plus_1).finished();

    }


    Vector evaluateError(const Vector4& xl_k, 
                         const Vector4& xr1_k,
                         const Vector1& tension1_k,
                         const Vector4& xr2_k,
                         const Vector1& tension2_k,
                         const Vector4& xl_k_plus_1,
                         gtsam::OptionalMatrixType H1,
                         gtsam::OptionalMatrixType H2,
                         gtsam::OptionalMatrixType H3,
                         gtsam::OptionalMatrixType H4,
                         gtsam::OptionalMatrixType H5,
                         gtsam::OptionalMatrixType H6) const override {

        // Unpack state: xr = [px, py, vx, vy]
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

        // Simple Euler integration for dynamics
        // next_pos = current_pos + dt * current_vel
        // next_vel = current_vel + dt * (control_force/mass - tension_force/mass)
        Vector2 next_pos = pos_k + vel_k * dt_;

        double xd1 = xr1_k(0) - xl_k(0);
        double yd1 = xr1_k(1) - xl_k(1);
        Vector2 e1(2);
        e1 << xd1, yd1;
        double norm1 = e1.norm();
        Vector2 e_norm1(2);
        e_norm1 << e1 / e1.norm();

        double xd2 = xr2_k(0) - xl_k(0);
        double yd2 = xr2_k(1) - xl_k(1);
        Vector2 e2(2);
        e2 << xd2, yd2;
        double norm2 = e2.norm();
        Vector2 e_norm2(2);
        e_norm2 << e2 / e2.norm();

        Vector2 next_vel = vel_k + (tension1_k(0) * e_norm1 + tension2_k(0) * e_norm2 - mu_ * load_mass_ * g_ * normed_vel_k) / load_mass_ * dt_;

        Vector4 predicted_xl_k_plus_1(4);
        predicted_xl_k_plus_1 << next_pos, next_vel;

        double ONE = 1.0 / norm1 - xd1 * xd1 / (norm1 * norm1 * norm1);
        double TWO = -1.0 / norm1 + xd1 * xd1 / (norm1 * norm1 * norm1);
        double THREE = -yd1 * xd1 / (norm1 * norm1 * norm1);
        double FOUR = xd1 * yd1 / (norm1 * norm1 * norm1);
        double FIVE = 1.0 / norm1 - yd1 * yd1 / (norm1 * norm1 * norm1);
        double SIX = -1.0 / norm1 + yd1 * yd1 / (norm1 * norm1 * norm1);

        double ONE2 = 1.0 / norm2 - xd2 * xd2 / (norm2 * norm2 * norm2);
        double TWO2 = -1.0 / norm2 + xd2 * xd2 / (norm2 * norm2 * norm2);
        double THREE2 = -yd2 * xd2 / (norm2 * norm2 * norm2);
        double FOUR2 = xd2 * yd2 / (norm2 * norm2 * norm2);
        double FIVE2 = 1.0 / norm2 - yd2 * yd2 / (norm2 * norm2 * norm2);
        double SIX2 = -1.0 / norm2 + yd2 * yd2 / (norm2 * norm2 * norm2);

        if (H1) {
            // *H1 = (gtsam::Matrix(4, 4) << 
            //     -1,  0, -dt_, 0,
            //     0, -1,  0,  -dt_,
            //     -dt_ * (tension1_k(0) * TWO + tension2_k(0) * TWO2) / load_mass_,  -dt_ * (tension1_k(0) * FOUR + tension2_k(0) * FOUR2) / load_mass_, -1 + dt_ * mu_ * g_ * SEVEN, dt_ * mu_ * g_ * NINE,
            //     -dt_ * (tension1_k(0) * FOUR + tension2_k(0) * FOUR2) / load_mass_,  -dt_ * (tension1_k(0) * SIX + tension2_k(0) * SIX2) / load_mass_,  dt_ * mu_ * g_ * NINE,  -1 + dt_ * mu_ * g_ * EIGHT).finished();
            *H1 = numericalDerivative61<Vector,Vector4, Vector4, Vector1, Vector4, Vector1, Vector4 > (
                    [this](const Vector4& a, 
                            const Vector4& b,
                            const Vector1& c,
                            const Vector4& d,
                            const Vector1& e,
                            const Vector4& f) {
                                return this->evaluateErrorOnly(a, b, c, d, e, f);
                            }, xl_k, xr1_k, tension1_k, xr2_k, tension2_k, xl_k_plus_1, 1e-5
                );
        }
        if (H2) {
            // *H2 = (gtsam::Matrix(4, 4) << 
            //     0,  0, 0, 0,
            //     0, 0,  0,  0,
            //     -dt_ * tension1_k(0) * ONE / load_mass_,  -dt_ * tension1_k(0) * THREE / load_mass_, 0, 0,
            //     -dt_ * tension1_k(0) * THREE / load_mass_,  -dt_ * tension1_k(0) * FIVE / load_mass_, 0, 0).finished();
            *H2 = numericalDerivative62<Vector,Vector4, Vector4, Vector1, Vector4, Vector1, Vector4 > (
                [this](const Vector4& a, 
                        const Vector4& b,
                        const Vector1& c,
                        const Vector4& d,
                        const Vector1& e,
                        const Vector4& f) {
                            return this->evaluateErrorOnly(a, b, c, d, e, f);
                        }, xl_k, xr1_k, tension1_k, xr2_k, tension2_k, xl_k_plus_1, 1e-5
            );
        }
        if (H3) {
            // *H3 = (gtsam::Matrix(4, 1) << 
            //     0,
            //     0,
            //     -(dt_ * xd1 / (load_mass_ * norm1)),
            //     -(dt_ * yd1 / (load_mass_ * norm1))).finished();
            *H3 = numericalDerivative63<Vector,Vector4, Vector4, Vector1, Vector4, Vector1, Vector4 > (
                [this](const Vector4& a, 
                        const Vector4& b,
                        const Vector1& c,
                        const Vector4& d,
                        const Vector1& e,
                        const Vector4& f) {
                            return this->evaluateErrorOnly(a, b, c, d, e, f);
                        }, xl_k, xr1_k, tension1_k, xr2_k, tension2_k, xl_k_plus_1, 1e-5
            );
        }
        if (H4) {
            // *H4 = (gtsam::Matrix(4, 4) << 
            //     0,  0, 0, 0,
            //     0, 0,  0,  0,
            //     -dt_ * tension2_k(0) * ONE2 / load_mass_,  -dt_ * tension2_k(0) * THREE2 / load_mass_, 0, 0,
            //     -dt_ * tension2_k(0) * THREE2 / load_mass_,  -dt_ * tension2_k(0) * FIVE2 / load_mass_, 0, 0).finished();
            *H4 = numericalDerivative64<Vector,Vector4, Vector4, Vector1, Vector4, Vector1, Vector4 > (
                [this](const Vector4& a, 
                        const Vector4& b,
                        const Vector1& c,
                        const Vector4& d,
                        const Vector1& e,
                        const Vector4& f) {
                            return this->evaluateErrorOnly(a, b, c, d, e, f);
                        }, xl_k, xr1_k, tension1_k, xr2_k, tension2_k, xl_k_plus_1, 1e-5
            );
        }
        if (H5) {
            // *H5 = (gtsam::Matrix(4, 1) << 
            //     0,
            //     0,
            //     -(dt_ * xd2 / (load_mass_ * norm2)),
            //     -(dt_ * yd2 / (load_mass_ * norm2))).finished();
            *H5 = numericalDerivative65<Vector,Vector4, Vector4, Vector1, Vector4, Vector1, Vector4 > (
                [this](const Vector4& a, 
                        const Vector4& b,
                        const Vector1& c,
                        const Vector4& d,
                        const Vector1& e,
                        const Vector4& f) {
                            return this->evaluateErrorOnly(a, b, c, d, e, f);
                        }, xl_k, xr1_k, tension1_k, xr2_k, tension2_k, xl_k_plus_1, 1e-5
            );
        }
        if (H6) {
            *H6 = gtsam::Matrix4::Identity();
        }

        return evaluateErrorOnly(xl_k, xr1_k, tension1_k, xr2_k, tension2_k, xl_k_plus_1); 
    }
};



#endif // DYNAMICS_FACTORS_HPP