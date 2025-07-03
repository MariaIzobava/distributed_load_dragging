#include "factor_graph_lib/control_factors.hpp"

using namespace gtsam;


Vector MagnitudeUpperBoundFactor::evaluateError(const Vector2& u_val,
                            OptionalMatrixType H) const {
    double current_magnitude = u_val.norm();

    Vector residual(1);
    if (current_magnitude > max_magnitude_) {
        residual[0] = current_magnitude - max_magnitude_;
    } else {
        residual[0] = 0.0;
    }

    if (H) {
        if (current_magnitude > max_magnitude_ && current_magnitude > 1e-9) {
            (*H) = (gtsam::Matrix(1, 2) << u_val[0] / current_magnitude, u_val[1] / current_magnitude).finished();
        } else {
            (*H) = (gtsam::Matrix(1, 2) << 0, 0).finished();
        }
    }
    return residual;
}


Vector MagnitudeLowerBoundFactor::evaluateError(const Vector2& u_val,
                                OptionalMatrixType H) const {

    double current_magnitude = u_val.norm();

    Vector residual(1);
    if (current_magnitude < min_magnitude_) {
        residual[0] = min_magnitude_ - current_magnitude;
    } else {
        residual[0] = 0.0;
    }

    if (H) {
        if (current_magnitude < min_magnitude_ && current_magnitude > 1e-9) {
            (*H) = (gtsam::Matrix(1, 2) << -u_val[0] / current_magnitude, -u_val[1] / current_magnitude).finished();
        } else {
            (*H) = (gtsam::Matrix(1, 2) << 0, 0).finished();
        }
    }
    return residual;
}