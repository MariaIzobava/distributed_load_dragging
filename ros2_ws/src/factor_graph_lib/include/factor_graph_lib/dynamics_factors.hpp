#ifndef DYNAMICS_FACTORS_HPP
#define DYNAMICS_FACTORS_HPP

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
                         gtsam::OptionalMatrixType H5) const override;
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
                         gtsam::OptionalMatrixType H4) const override;
};

#endif // DYNAMICS_FACTORS_HPP