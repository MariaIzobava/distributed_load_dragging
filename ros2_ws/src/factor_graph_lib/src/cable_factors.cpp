// #include "factor_graph_lib/cable_factors.hpp"

// using namespace gtsam;


// Vector TensionLowerBoundFactor::evaluateError(const Vector1& T_k,
//                          gtsam::OptionalMatrixType H) const {
//     // The error value is `weight_ * smooth_max_zero(-T_k)`.
//     double epsilon = 1e-6;
//     double x = -T_k(0);
//     double smooth_max_zero = 0.5 * (x + std::sqrt(x*x + epsilon*epsilon));
//     double error_val = weight_ * smooth_max_zero;

//     if (H) {
//         // Calculate the Jacobian (derivative of the error with respect to T_k).
//         // Let f(x) = smooth_max_zero(x). We want d(weight_ * f(-T_k))/dT_k.
//         // Using the chain rule: weight_ * f'(-T_k) * d(-T_k)/dT_k = weight_ * f'(-T_k) * (-1).
//         // The derivative of f(x) is f'(x) = 0.5 * (1 + x / sqrt(x^2 + epsilon^2)).
//         // So, f'(-T_k) = 0.5 * (1 + (-T_k) / sqrt((-T_k)^2 + epsilon^2)).
//         // d(error_val)/dT_k = weight_ * (-0.5) * (1 - T_k / sqrt(T_k^2 + epsilon^2)).
//         double deriv_smooth_max_inner = 0.5 * (1.0 - T_k(0) / std::sqrt(T_k(0)*T_k(0) + 1e-6*1e-6));
//         (*H) = (Vector(1) << -weight_ * deriv_smooth_max_inner).finished();
//     }
//     return (Vector(1) << error_val).finished(); // Return a 1x1 vector for scalar error
// }


// Vector CableStretchPenaltyFactor::evaluateError(const Vector4& p_r, const Vector4& p_l,
//                      gtsam::OptionalMatrixType H1,
//                      gtsam::OptionalMatrixType H2) const {
//     Vector2 diff(2);
//     diff << p_r(0) - p_l(0), p_r(1) - p_l(1);
//     double distance = diff.norm(); // Euclidean distance ||p_r - p_l||

//     // The error value is `weight_ * smooth_max_zero(distance - cable_length_)`.
//     double error_val = weight_ * smooth_max_zero_(distance - cable_length_);

//     if (H1 || H2) {
//         double inner_term = distance - cable_length_;
//         double deriv_smooth_max_wrt_inner = 0.5 * (1.0 + inner_term / std::sqrt(inner_term*inner_term + 1e-6*1e-6));

//         // The derivative of distance `||p_r - p_l||` with respect to `p_r` is `(p_r - p_l) / ||p_r - p_l||`,
//         // which is the unit vector `e` pointing from load to robot.
//         Vector2 partial_deriv_distance_pr = diff / distance;
//         // The derivative with respect to `p_l` is `-(p_r - p_l) / ||p_r - p_l||`, or `-e`.
//         Vector2 partial_deriv_distance_pl = -diff / distance;

//         if (H1) { // Jacobian with respect to p_r
//             // Chain rule: d(error)/dp_r = weight * d(smooth_max_zero)/d(inner) * d(inner)/d(distance) * d(distance)/dp_r
//             // d(inner)/d(distance) is 1.0.
//             (*H1) = (gtsam::Matrix(1, 4) << weight_ * deriv_smooth_max_wrt_inner * partial_deriv_distance_pr(0), weight_ * deriv_smooth_max_wrt_inner * partial_deriv_distance_pr(1), 0, 0).finished();
//         }
//         if (H2) { // Jacobian with respect to p_l
//             (*H2) = (gtsam::Matrix(1, 4) << weight_ * deriv_smooth_max_wrt_inner * partial_deriv_distance_pl(0), weight_ * deriv_smooth_max_wrt_inner * partial_deriv_distance_pl(1), 0, 0).finished();
//         }
//     }
//     return (Vector(1) << error_val).finished();
// }


// Vector TensionSlackPenaltyFactor::evaluateError(const Vector1& T_k, const Vector4& p_r, const Vector4& p_l,
//                      gtsam::OptionalMatrixType H1,
//                      gtsam::OptionalMatrixType H2,
//                      gtsam::OptionalMatrixType H3) const {
//     Vector2 diff(2);
//     diff << p_r(0) - p_l(0), p_r(1) - p_l(1);
//     double distance = diff.norm();
//     // Calculate the slack term: max(0, L_cable - distance).
//     // This term is positive if the cable is slack (distance < L_cable).
//     double slack_term_val = smooth_max_zero_(cable_length_ - distance);

//     // The error value is `weight_ * T_k * slack_term_val`.
//     // If T_k > 0 AND slack_term_val > 0, this factor incurs a penalty.
//     double error_val = weight_ * T_k(0) * slack_term_val;

//     if (H1 || H2 || H3) {
//         // Derivatives for Jacobian calculations:
//         // d(error_val)/dT_k = weight_ * slack_term_val
//         // d(error_val)/dp_r = weight_ * T_k * d(slack_term_val)/dp_r
//         // d(error_val)/dp_l = weight_ * T_k * d(slack_term_val)/dp_l

//         // First, calculate derivative of smooth_max_zero(x) where x = (cable_length_ - distance).
//         // d(smooth_max_zero(x))/dx = 0.5 * (1 + x / sqrt(x^2 + epsilon^2))
//         double inner_term = cable_length_ - distance;
//         double deriv_smooth_max_wrt_inner = 0.5 * (1.0 + inner_term / std::sqrt(inner_term*inner_term + 1e-6*1e-6));

//         // Chain rule for d(slack_term_val)/d(distance): d(smooth_max_zero(C-D))/dD = d(smooth_max_zero)/d(inner) * d(inner)/dD
//         // Here, d(inner)/dD = d(C-D)/dD = -1.
//         double deriv_slack_wrt_distance = deriv_smooth_max_wrt_inner * (-1.0);

//         // Derivatives of distance with respect to positions (unit vectors).
//         Vector2 partial_deriv_distance_pr = diff / distance; // unit vector e
//         Vector2 partial_deriv_distance_pl = -diff / distance; // -e vector

//         if (H1) { // Jacobian with respect to T_k
//             (*H1) = (Vector(1) << weight_ * slack_term_val).finished();
//         }
//         if (H2) { // Jacobian with respect to p_r
//             // Chain rule: weight * T_k * d(slack_term_val)/d(distance) * d(distance)/dp_r
//             (*H2) = (gtsam::Matrix(1, 4) << weight_ * T_k(0) * deriv_slack_wrt_distance * partial_deriv_distance_pr(0), weight_ * T_k(0) * deriv_slack_wrt_distance * partial_deriv_distance_pr(1), 0, 0).finished();
//         }
//         if (H3) { // Jacobian with respect to p_l
//             // Chain rule: weight * T_k * d(slack_term_val)/d(distance) * d(distance)/dp_l
//             (*H3) = (gtsam::Matrix(1, 4) << weight_ * T_k(0) * deriv_slack_wrt_distance * partial_deriv_distance_pl(0), weight_ * T_k(0) * deriv_slack_wrt_distance * partial_deriv_distance_pl(1), 0, 0).finished();
//         }
//     }
//     return (Vector(1) << error_val).finished();
// }