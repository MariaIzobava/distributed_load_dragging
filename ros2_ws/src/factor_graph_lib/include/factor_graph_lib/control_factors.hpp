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
public:
    MagnitudeUpperBoundFactor(Key key, 
                              double maxMagnitude,
                              const SharedNoiseModel& model)
        : NoiseModelFactor1<Vector2>(model, key), max_magnitude_(maxMagnitude) {}

    Vector evaluateError(const Vector2& u_val,
                        OptionalMatrixType H) const override ;
};

class MagnitudeLowerBoundFactor : public NoiseModelFactor1<Vector2> {
private:
    double min_magnitude_;
public:
    MagnitudeLowerBoundFactor(Key key, 
                              double minMagnitude,
                              const SharedNoiseModel& model)
        : NoiseModelFactor1<Vector2>(model, key), min_magnitude_(minMagnitude) {}

    Vector evaluateError(const Vector2& u_val,
                         OptionalMatrixType H) const override;
};

#endif // CONTROL_FACTORS_HPP