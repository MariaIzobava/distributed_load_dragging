#ifndef DYNAMICS_FACTORS_MULTI_ROBOTS_WITH_HEIGHT_HPP
#define DYNAMICS_FACTORS_MULTI_ROBOTS_WITH_HEIGHT_HPP

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

class LoadDynamicsMultiRobotsWithHeightFactor2: public NoiseModelFactor6<Vector4, Vector6, Vector1, Vector6, Vector1, Vector4> {
    double dt_;
    double load_mass_;
    double mu_;
    double g_;

public:
    LoadDynamicsMultiRobotsWithHeightFactor2(
        Key key_xl_k, 
        Key key_xr1_k, 
        Key key_tension1_k, 
        Key key_xr2_k, 
        Key key_tension2_k,
        Key key_xl_k_plus_1,
        double dt, double load_mass, double mu, double g, const SharedNoiseModel& model) :
        NoiseModelFactor6<Vector4, Vector6, Vector1, Vector6, Vector1, Vector4>(model, key_xl_k, key_xr1_k, key_tension1_k, key_xr2_k, key_tension2_k, key_xl_k_plus_1),
        dt_(dt), load_mass_(load_mass), mu_(mu), g_(g) {}

    Vector evaluateError(const Vector4& xl_k, 
                         const Vector6& xr1_k,
                         const Vector1& tension1_k,
                         const Vector6& xr2_k,
                         const Vector1& tension2_k,
                         const Vector4& xl_k_plus_1,
                         gtsam::OptionalMatrixType H1,
                         gtsam::OptionalMatrixType H2,
                         gtsam::OptionalMatrixType H3,
                         gtsam::OptionalMatrixType H4,
                         gtsam::OptionalMatrixType H5,
                         gtsam::OptionalMatrixType H6) const override {
        
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

        CableVectorHelper h1(xr1_k, xl_k, "middle");
        CableVectorHelper h2(xr2_k, xl_k, "middle");

        Vector2 next_pos = pos_k + vel_k * dt_;
        Vector2 hor1_e_norm(h1.e3_norm(0), h1.e3_norm(1));
        Vector2 hor2_e_norm(h2.e3_norm(0), h2.e3_norm(1));
        Vector2 next_vel = vel_k +  dt_ / load_mass_ * (tension1_k(0) * hor1_e_norm + tension2_k(0) * hor2_e_norm - mu_ * (load_mass_ * g_ - tension1_k(0) * h1.e3_norm(2) - tension2_k(0) * h2.e3_norm(2)) * normed_vel_k);

        Vector4 predicted_xl_k_plus_1(4);
        predicted_xl_k_plus_1 << next_pos, next_vel;


        double A_CNST1 = -dt_ * tension1_k(0) / load_mass_;
        double A_CNST2 = -dt_ * tension2_k(0) / load_mass_;
        Vector2 MUV = mu_ * normed_vel_k;

        if (H1) {
            *H1 = (gtsam::Matrix(4, 4) << 
                -1,  0, -dt_, 0,
                0, -1,  0,  -dt_,
                A_CNST1 * (h1.ex_dxl + MUV(0) * h1.ez_dxl) + A_CNST2 * (h2.ex_dxl + MUV(0) * h2.ez_dxl),  A_CNST1 * (h1.ex_dyl + MUV(0) * h1.ez_dyl) + A_CNST2 * (h2.ex_dyl + MUV(0) * h2.ez_dyl), -1 + dt_ * mu_ * g_ * SEVEN, dt_ * mu_ * g_ * NINE,
                A_CNST1 * (h1.ey_dxl + MUV(1) * h1.ez_dxl) + A_CNST2 * (h2.ey_dxl + MUV(1) * h2.ez_dxl),  A_CNST1 * (h1.ey_dyl + MUV(1) * h1.ez_dyl) + A_CNST2 * (h2.ey_dyl + MUV(1) * h2.ez_dyl),  dt_ * mu_ * g_ * NINE,  -1 + dt_ * mu_ * g_ * EIGHT).finished();
        }
        if (H2) {
            *H2 = (gtsam::Matrix(4, 6) << 
                0,  0, 0, 0, 0,0,
                0, 0,  0,  0,0,0,
                A_CNST1 * (h1.ex_dxr + MUV(0) * h1.ez_dxr),  A_CNST1 * (h1.ex_dyr + MUV(0) * h1.ez_dyr), A_CNST1 * (h1.ex_dzr + MUV(0) * h1.ez_dzr), 0, 0, 0,
                A_CNST1 * (h1.ey_dxr + MUV(1) * h1.ez_dxr),  A_CNST1 * (h1.ey_dyr + MUV(1) * h1.ez_dyr), A_CNST1 * (h1.ey_dzr + MUV(1) * h1.ez_dzr), 0, 0, 0).finished();
        }
        if (H3) {
            *H3 = (gtsam::Matrix(4, 1) << 
                0,
                0,
                -(dt_ * (h1.e3_norm(0) + MUV(0) * h1.e3_norm(2)) / load_mass_),
                -(dt_ * (h1.e3_norm(1) + MUV(1) * h1.e3_norm(2)) / load_mass_)).finished();
        }
        if (H4) {
            *H4 = (gtsam::Matrix(4, 6) << 
                0,  0, 0, 0, 0,0,
                0, 0,  0,  0,0,0,
                A_CNST2 * (h2.ex_dxr + MUV(0) * h2.ez_dxr),  A_CNST2 * (h2.ex_dyr + MUV(0) * h2.ez_dyr), A_CNST2 * (h2.ex_dzr + MUV(0) * h2.ez_dzr), 0, 0, 0,
                A_CNST2 * (h2.ey_dxr + MUV(1) * h2.ez_dxr),  A_CNST2 * (h2.ey_dyr + MUV(1) * h2.ez_dyr), A_CNST2 * (h2.ey_dzr + MUV(1) * h2.ez_dzr), 0, 0, 0).finished();
        }
        if (H5) {
            *H5 = (gtsam::Matrix(4, 1) << 
                0,
                0,
                -(dt_ * (h2.e3_norm(0)+ MUV(0) * h2.e3_norm(2)) / load_mass_),
                -(dt_ * (h2.e3_norm(1)+ MUV(1) * h2.e3_norm(2)) / load_mass_)).finished();
        }
        if (H6) {
            *H6 = gtsam::Matrix4::Identity();
        }

        return (Vector(4) << xl_k_plus_1 - predicted_xl_k_plus_1).finished();
    }
};


class LoadDynamicsMultiRobotsWithHeightFactor3: public NoiseModelFactorN<Vector4, Vector6, Vector1, Vector6, Vector1, Vector6, Vector1, Vector4> {
    double dt_;
    double load_mass_;
    double mu_;
    double g_;

public:
    LoadDynamicsMultiRobotsWithHeightFactor3(
        Key key_xl_k, 
        Key key_xr1_k, 
        Key key_tension1_k, 
        Key key_xr2_k, 
        Key key_tension2_k,
        Key key_xr3_k, 
        Key key_tension3_k,
        Key key_xl_k_plus_1,
        double dt, double load_mass, double mu, double g, const SharedNoiseModel& model) :
        NoiseModelFactorN<Vector4, Vector6, Vector1, Vector6, Vector1, Vector6, Vector1, Vector4>(
            model, key_xl_k, key_xr1_k, key_tension1_k, key_xr2_k, key_tension2_k,
            key_xr3_k, key_tension3_k, key_xl_k_plus_1),
        dt_(dt), load_mass_(load_mass), mu_(mu), g_(g) {}

    Vector evaluateError(const Vector4& xl_k, 
                         const Vector6& xr1_k,
                         const Vector1& tension1_k,
                         const Vector6& xr2_k,
                         const Vector1& tension2_k,
                         const Vector6& xr3_k,
                         const Vector1& tension3_k,
                         const Vector4& xl_k_plus_1,
                         gtsam::OptionalMatrixType H1,
                         gtsam::OptionalMatrixType H2,
                         gtsam::OptionalMatrixType H3,
                         gtsam::OptionalMatrixType H4,
                         gtsam::OptionalMatrixType H5,
                         gtsam::OptionalMatrixType H6,
                         gtsam::OptionalMatrixType H7,
                         gtsam::OptionalMatrixType H8) const override {
        
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

        CableVectorHelper h1(xr1_k, xl_k, "middle");
        CableVectorHelper h2(xr2_k, xl_k, "middle");
        CableVectorHelper h3(xr3_k, xl_k, "middle");

        Vector2 next_pos = pos_k + vel_k * dt_;
        Vector2 hor1_e_norm(h1.e3_norm(0), h1.e3_norm(1));
        Vector2 hor2_e_norm(h2.e3_norm(0), h2.e3_norm(1));
        Vector2 hor3_e_norm(h3.e3_norm(0), h3.e3_norm(1));

        Vector2 next_vel = vel_k +  dt_ / load_mass_ * (
            tension1_k(0) * hor1_e_norm + 
            tension2_k(0) * hor2_e_norm +
            tension3_k(0) * hor3_e_norm - 
            mu_ * (load_mass_ * g_ - 
            tension1_k(0) * h1.e3_norm(2) - 
            tension2_k(0) * h2.e3_norm(2) -
            tension3_k(0) * h3.e3_norm(2)) * normed_vel_k);

        Vector4 predicted_xl_k_plus_1(4);
        predicted_xl_k_plus_1 << next_pos, next_vel;


        double A_CNST1 = -dt_ * tension1_k(0) / load_mass_;
        double A_CNST2 = -dt_ * tension2_k(0) / load_mass_;
        double A_CNST3 = -dt_ * tension3_k(0) / load_mass_;

        if (H1) {
            *H1 = (gtsam::Matrix(4, 4) << 
                -1,  0, -dt_, 0,
                0, -1,  0,  -dt_,
                A_CNST1 * h1.ex_dxl + A_CNST2 * h2.ex_dxl + A_CNST3 * h3.ex_dxl,  A_CNST1 * h1.ex_dyl + A_CNST2 * h2.ex_dyl + A_CNST3 * h3.ex_dyl, -1 + dt_ * mu_ * g_ * SEVEN, dt_ * mu_ * g_ * NINE,
                A_CNST1 * h1.ey_dxl + A_CNST2 * h2.ey_dxl + A_CNST3 * h3.ey_dxl,  A_CNST1 * h1.ey_dyl + A_CNST2 * h2.ey_dyl + A_CNST3 * h3.ey_dyl,  dt_ * mu_ * g_ * NINE,  -1 + dt_ * mu_ * g_ * EIGHT).finished();
        }
        if (H2) {
            *H2 = (gtsam::Matrix(4, 6) << 
                0,  0, 0, 0, 0,0,
                0, 0,  0,  0,0,0,
                A_CNST1 * (h1.ex_dxr + h1.ez_dxr),  A_CNST1 * (h1.ex_dyr + h1.ez_dyr), A_CNST1 * (h1.ex_dzr + h1.ez_dzr), 0, 0, 0,
                A_CNST1 * (h1.ey_dxr + h1.ez_dxr),  A_CNST1 * (h1.ey_dyr + h1.ez_dyr), A_CNST1 * (h1.ey_dzr + h1.ez_dzr), 0, 0, 0).finished();
        }
        if (H3) {
            *H3 = (gtsam::Matrix(4, 1) << 
                0,
                0,
                -(dt_ * h1.e3_norm(0) / load_mass_),
                -(dt_ * h1.e3_norm(1) / load_mass_)).finished();
        }
        if (H4) {
            *H4 = (gtsam::Matrix(4, 6) << 
                0,  0, 0, 0, 0,0,
                0, 0,  0,  0,0,0,
                A_CNST2 * (h2.ex_dxr + h2.ez_dxr),  A_CNST2 * (h2.ex_dyr + h2.ez_dyr), A_CNST2 * (h2.ex_dzr + h2.ez_dzr), 0, 0, 0,
                A_CNST2 * (h2.ey_dxr + h2.ez_dxr),  A_CNST2 * (h2.ey_dyr + h2.ez_dyr), A_CNST2 * (h2.ey_dzr + h2.ez_dzr), 0, 0, 0).finished();
        }
        if (H5) {
            *H5 = (gtsam::Matrix(4, 1) << 
                0,
                0,
                -(dt_ * h2.e3_norm(0) / load_mass_),
                -(dt_ * h2.e3_norm(1) / load_mass_)).finished();
        }
        if (H6) {
            *H6 = (gtsam::Matrix(4, 6) << 
                0,  0, 0, 0, 0,0,
                0, 0,  0,  0,0,0,
                A_CNST3 * (h3.ex_dxr + h3.ez_dxr),  A_CNST3 * (h3.ex_dyr + h3.ez_dyr), A_CNST3 * (h3.ex_dzr + h3.ez_dzr), 0, 0, 0,
                A_CNST3 * (h3.ey_dxr + h3.ez_dxr),  A_CNST3 * (h3.ey_dyr + h3.ez_dyr), A_CNST3 * (h3.ey_dzr + h3.ez_dzr), 0, 0, 0).finished();
        }
        if (H7) {
            *H7 = (gtsam::Matrix(4, 1) << 
                0,
                0,
                -(dt_ * h3.e3_norm(0) / load_mass_),
                -(dt_ * h3.e3_norm(1) / load_mass_)).finished();
        }
        if (H8) {
            *H8 = gtsam::Matrix4::Identity();
        }

        return (Vector(4) << xl_k_plus_1 - predicted_xl_k_plus_1).finished();
    }
};

#endif // DYNAMICS_FACTORS_MULTI_ROBOTS_WITH_HEIGHT_HPP