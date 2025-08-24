#ifndef CONTROL_FACTORS_HPP
#define CONTROL_FACTORS_HPP

#include <string>

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>

using namespace std;
using namespace gtsam;


class MagnitudeUpperBoundFactor : public NoiseModelFactor1<Vector2> {
private:
    double max_magnitude_;
    double e_ = 1e-6;
public:
    MagnitudeUpperBoundFactor(Key key, 
                              double maxMagnitude,
                              const SharedNoiseModel& model)
        : NoiseModelFactor1<Vector2>(model, key), max_magnitude_(maxMagnitude) {}

    // Vector evaluateError(const Vector2& u_val,
    //                     OptionalMatrixType H) const override {
    //     double current_magnitude = u_val.norm();

    //     Vector residual(1);
    //     if (current_magnitude > max_magnitude_) {
    //         residual[0] = current_magnitude - max_magnitude_;
    //     } else {
    //         residual[0] = 0.0;
    //     }

    //     if (H) {
    //         if (current_magnitude > max_magnitude_ && current_magnitude > 1e-9) {
    //             (*H) = (gtsam::Matrix(1, 2) << u_val[0] / current_magnitude, u_val[1] / current_magnitude).finished();
    //         } else {
    //             (*H) = (gtsam::Matrix(1, 2) << 0, 0).finished();
    //         }
    //     }
    //     return residual;
    // }

    Vector evaluateError(const Vector2& u_val,
                         OptionalMatrixType H) const override {

        double current_magnitude = u_val.norm();
        double x = current_magnitude - max_magnitude_;

        double err = 0.5 * (x + std::sqrt(x * x + e_ * e_));


        if (H) {
            double de_dx = 0.5 * (1.0 + x / std::sqrt(x * x + e_ * e_));

            if (current_magnitude > 1e-9) {
                Vector2 dmagnitute_dv = u_val / current_magnitude;
                (*H) = (gtsam::Matrix(1, 2) << de_dx * dmagnitute_dv(0), de_dx * dmagnitute_dv(1)).finished();
            } else {
                (*H) = (gtsam::Matrix(1, 2) << 0, 0).finished();
            }
        }
        return (Vector(1) << err).finished();
    }
};

class MagnitudeLowerBoundFactor : public NoiseModelFactor1<Vector2> {
private:
    double min_magnitude_;
    double e_ = 1e-6;
public:
    MagnitudeLowerBoundFactor(Key key, 
                              double minMagnitude,
                              const SharedNoiseModel& model)
        : NoiseModelFactor1<Vector2>(model, key), min_magnitude_(minMagnitude) {}

    // Vector evaluateError(const Vector2& u_val,
    //                      OptionalMatrixType H) const override {

    //     double current_magnitude = u_val.norm();

    //     Vector residual(1);
    //     if (current_magnitude < min_magnitude_) {
    //         residual[0] = min_magnitude_ - current_magnitude;
    //     } else {
    //         residual[0] = 0.0;
    //     }

    //     if (H) {
    //         if (current_magnitude < min_magnitude_ && current_magnitude > 1e-9) {
    //             (*H) = (gtsam::Matrix(1, 2) << -u_val[0] / current_magnitude, -u_val[1] / current_magnitude).finished();
    //         } else {
    //             (*H) = (gtsam::Matrix(1, 2) << 0, 0).finished();
    //         }
    //     }
    //     return residual;
    // }

    Vector evaluateError(const Vector2& u_val,
                         OptionalMatrixType H) const override {

        double current_magnitude = u_val.norm();
        double x = min_magnitude_ - current_magnitude;

        double err = 0.5 * (x + std::sqrt(x * x + e_ * e_));


        if (H) {
            double de_dx = 0.5 * (1.0 + x / std::sqrt(x * x + e_ * e_));
            double dx_dmagnitude = -1.0;

            if (current_magnitude > 1e-9) {
                Vector2 dmagnitute_dv = u_val / current_magnitude;
                (*H) = (gtsam::Matrix(1, 2) << de_dx * dx_dmagnitude * dmagnitute_dv(0), de_dx * dx_dmagnitude * dmagnitute_dv(1)).finished();
            } else {
                (*H) = (gtsam::Matrix(1, 2) << 0, 0).finished();
            }
        }
        return (Vector(1) << err).finished();
    }
};


class MagnitudeUpperBoundWithHeightFactor : public NoiseModelFactor1<Vector3> {
private:
    double max_magnitude_;
    double e_ = 1e-6;
public:
    MagnitudeUpperBoundWithHeightFactor(Key key, 
                              double maxMagnitude,
                              const SharedNoiseModel& model)
        : NoiseModelFactor1<Vector3>(model, key), max_magnitude_(maxMagnitude) {}


    Vector evaluateError(const Vector3& u_val,
                         OptionalMatrixType H) const override {

        double current_magnitude = u_val.norm();
        double x = current_magnitude - max_magnitude_;

        double err = 0.5 * (x + std::sqrt(x * x + e_ * e_));


        if (H) {
            double de_dx = 0.5 * (1.0 + x / std::sqrt(x * x + e_ * e_));

            if (current_magnitude > 1e-9) {
                Vector3 dmagnitute_dv = u_val / current_magnitude;
                (*H) = (gtsam::Matrix(1, 3) << de_dx * dmagnitute_dv(0), de_dx * dmagnitute_dv(1), de_dx * dmagnitute_dv(2)).finished();
            } else {
                (*H) = (gtsam::Matrix(1, 3) << 0, 0, 0).finished();
            }
        }
        return (Vector(1) << err).finished();
    }
};

class MagnitudeLowerBoundWithHeightFactor : public NoiseModelFactor1<Vector3> {
private:
    double min_magnitude_;
    double e_ = 1e-6;
public:
    MagnitudeLowerBoundWithHeightFactor(Key key, 
                              double minMagnitude,
                              const SharedNoiseModel& model)
        : NoiseModelFactor1<Vector3>(model, key), min_magnitude_(minMagnitude) {}

    Vector evaluateError(const Vector3& u_val,
                         OptionalMatrixType H) const override {

        double current_magnitude = u_val.norm();
        double x = min_magnitude_ - current_magnitude;

        double err = 0.5 * (x + std::sqrt(x * x + e_ * e_));


        if (H) {
            double de_dx = 0.5 * (1.0 + x / std::sqrt(x * x + e_ * e_));
            double dx_dmagnitude = -1.0;

            if (current_magnitude > 1e-9) {
                Vector3 dmagnitute_dv = u_val / current_magnitude;
                (*H) = (gtsam::Matrix(1, 3) << de_dx * dx_dmagnitude * dmagnitute_dv(0), de_dx * dx_dmagnitude * dmagnitute_dv(1), de_dx * dx_dmagnitude * dmagnitute_dv(2)).finished();
            } else {
                (*H) = (gtsam::Matrix(1, 3) << 0, 0, 0).finished();
            }
        }
        return (Vector(1) << err).finished();
    }
};

#endif // CONTROL_FACTORS_HPP