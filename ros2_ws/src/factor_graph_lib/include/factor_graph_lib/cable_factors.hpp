#ifndef CABLE_FACTORS_HPP
#define CABLE_FACTORS_HPP

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


class TensionLowerBoundFactor : public NoiseModelFactor1<Vector1> {
private:
    double weight_; // Weight for this factor

public:
    TensionLowerBoundFactor(Key tensionKey, double weight, const SharedNoiseModel& model)
        : NoiseModelFactor1<Vector1>(model, tensionKey), weight_(weight) {}

    Vector evaluateError(const Vector1& T_k,
                         gtsam::OptionalMatrixType H) const override;
};


// -------------------------------------------------------------------------
// 2. Custom Factor: Cable Stretch Penalty (||p_r - p_l|| <= L_cable)
// Error: w_stretch * max(0, ||p_r - p_l|| - L_cable)
// This factor penalizes if the cable is stretched beyond its nominal length.
// -------------------------------------------------------------------------
class CableStretchPenaltyFactor : public NoiseModelFactor2<Vector4, Vector4> {
private:
    double cable_length_; // The nominal length of the cable
    double weight_;       // Weight for this factor

    double smooth_max_zero_(double x, double epsilon = 1e-6) const {
        return 0.5 * (x + std::sqrt(x*x + epsilon*epsilon));
    }

public:
    CableStretchPenaltyFactor(Key robotPosKey, Key loadPosKey, double cableLength,
                              double weight, const SharedNoiseModel& model)
        : NoiseModelFactor2<Vector4, Vector4>(model, robotPosKey, loadPosKey),
          cable_length_(cableLength), weight_(weight) {}

    Vector evaluateError(const Vector4& p_r, const Vector4& p_l,
                         gtsam::OptionalMatrixType H1,
                         gtsam::OptionalMatrixType H2) const override;
};


// -------------------------------------------------------------------------
// 3. Custom Factor: Tension-Slack Penalty (T_k * max(0, L_cable - ||p_r - p_l||))
// Error: w_T_slack * T_k * max(0, L_cable - ||p_r - p_l||)
// This factor discourages positive tension when the cable is slack.
// -------------------------------------------------------------------------
class TensionSlackPenaltyFactor : public NoiseModelFactor3<Vector1, Vector4, Vector4> {
private:
    double cable_length_; // The nominal length of the cable
    double weight_;       // Weight for this factor

    double smooth_max_zero_(double x, double epsilon = 1e-6) const {
        return 0.5 * (x + std::sqrt(x*x + epsilon*epsilon));
    }

public:
    TensionSlackPenaltyFactor(Key tensionKey, Key robotPosKey, Key loadPosKey, double cableLength,
                               double weight, const SharedNoiseModel& model)
        : NoiseModelFactor3<Vector1, Vector4, Vector4>(model, tensionKey, robotPosKey, loadPosKey),
          cable_length_(cableLength), weight_(weight) {}

    Vector evaluateError(const Vector1& T_k, const Vector4& p_r, const Vector4& p_l,
                         gtsam::OptionalMatrixType H1,
                         gtsam::OptionalMatrixType H2,
                         gtsam::OptionalMatrixType H3) const override ;
};

#endif // CABLE_FACTORS_HPP