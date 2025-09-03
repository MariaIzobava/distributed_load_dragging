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
                         gtsam::OptionalMatrixType H) const override {
        // The error value is `weight_ * smooth_max_zero(-T_k)`.
        double epsilon = 1e-6;
        double x = -T_k(0);
        double smooth_max_zero = 0.5 * (x + std::sqrt(x*x + epsilon*epsilon));
        double error_val = weight_ * smooth_max_zero;

        if (H) {
            // Calculate the Jacobian (derivative of the error with respect to T_k).
            // Let f(x) = smooth_max_zero(x). We want d(weight_ * f(-T_k))/dT_k.
            // Using the chain rule: weight_ * f'(-T_k) * d(-T_k)/dT_k = weight_ * f'(-T_k) * (-1).
            // The derivative of f(x) is f'(x) = 0.5 * (1 + x / sqrt(x^2 + epsilon^2)).
            // So, f'(-T_k) = 0.5 * (1 + (-T_k) / sqrt((-T_k)^2 + epsilon^2)).
            // d(error_val)/dT_k = weight_ * (-0.5) * (1 - T_k / sqrt(T_k^2 + epsilon^2)).
            double deriv_smooth_max_inner = 0.5 * (1.0 - T_k(0) / std::sqrt(T_k(0)*T_k(0) + 1e-6*1e-6));
            (*H) = (Vector(1) << -weight_ * deriv_smooth_max_inner).finished();
        }
        return (Vector(1) << error_val).finished(); // Return a 1x1 vector for scalar error
    }
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
                         gtsam::OptionalMatrixType H2) const override {
        Vector2 diff(2);
        diff << p_r(0) - p_l(0), p_r(1) - p_l(1);
        double distance = diff.norm(); // Euclidean distance ||p_r - p_l||

        // The error value is `weight_ * smooth_max_zero(distance - cable_length_)`.
        double error_val = weight_ * smooth_max_zero_(distance - cable_length_);

        if (H1 || H2) {
            double inner_term = distance - cable_length_;
            double deriv_smooth_max_wrt_inner = 0.5 * (1.0 + inner_term / std::sqrt(inner_term*inner_term + 1e-6*1e-6));

            // The derivative of distance `||p_r - p_l||` with respect to `p_r` is `(p_r - p_l) / ||p_r - p_l||`,
            // which is the unit vector `e` pointing from load to robot.
            Vector2 partial_deriv_distance_pr = diff / distance;
            // The derivative with respect to `p_l` is `-(p_r - p_l) / ||p_r - p_l||`, or `-e`.
            Vector2 partial_deriv_distance_pl = -diff / distance;

            if (H1) { // Jacobian with respect to p_r
                // Chain rule: d(error)/dp_r = weight * d(smooth_max_zero)/d(inner) * d(inner)/d(distance) * d(distance)/dp_r
                // d(inner)/d(distance) is 1.0.
                (*H1) = (gtsam::Matrix(1, 4) << weight_ * deriv_smooth_max_wrt_inner * partial_deriv_distance_pr(0), weight_ * deriv_smooth_max_wrt_inner * partial_deriv_distance_pr(1), 0, 0).finished();
            }
            if (H2) { // Jacobian with respect to p_l
                (*H2) = (gtsam::Matrix(1, 4) << weight_ * deriv_smooth_max_wrt_inner * partial_deriv_distance_pl(0), weight_ * deriv_smooth_max_wrt_inner * partial_deriv_distance_pl(1), 0, 0).finished();
            }
        }
        return (Vector(1) << error_val).finished();
    }

};


class CableStretchPenaltyWithHeightFactor : public NoiseModelFactor2<Vector6, Vector4> {
private:
    double cable_length_; // The nominal length of the cable
    double weight_;       // Weight for this factor

    double smooth_max_zero_(double x, double epsilon = 1e-6) const {
        return 0.5 * (x + std::sqrt(x*x + epsilon*epsilon));
    }

public:
    CableStretchPenaltyWithHeightFactor(Key robotPosKey, Key loadPosKey, double cableLength,
                              double weight, const SharedNoiseModel& model)
        : NoiseModelFactor2<Vector6, Vector4>(model, robotPosKey, loadPosKey),
          cable_length_(cableLength), weight_(weight) {}

    Vector evaluateError(const Vector6& p_r, const Vector4& p_l,
                         gtsam::OptionalMatrixType H1,
                         gtsam::OptionalMatrixType H2) const override {
        Vector3 diff(3);
        diff << p_r(0) - p_l(0), p_r(1) - p_l(1), p_r(2) - 0.2;
        double distance = diff.norm(); // Euclidean distance ||p_r - p_l||

        // The error value is `weight_ * smooth_max_zero(distance - cable_length_)`.
        double error_val = weight_ * smooth_max_zero_(distance - cable_length_);

        if (H1 || H2) {
            double inner_term = distance - cable_length_;
            double deriv_smooth_max_wrt_inner = 0.5 * (1.0 + inner_term / std::sqrt(inner_term*inner_term + 1e-6*1e-6));

            // The derivative of distance `||p_r - p_l||` with respect to `p_r` is `(p_r - p_l) / ||p_r - p_l||`,
            // which is the unit vector `e` pointing from load to robot.
            Vector3 partial_deriv_distance_pr = diff / distance;
            // The derivative with respect to `p_l` is `-(p_r - p_l) / ||p_r - p_l||`, or `-e`.
            Vector3 partial_deriv_distance_pl = -diff / distance;

            if (H1) { // Jacobian with respect to p_r
                // Chain rule: d(error)/dp_r = weight * d(smooth_max_zero)/d(inner) * d(inner)/d(distance) * d(distance)/dp_r
                // d(inner)/d(distance) is 1.0.
                (*H1) = (gtsam::Matrix(1, 6) << weight_ * deriv_smooth_max_wrt_inner * partial_deriv_distance_pr(0), weight_ * deriv_smooth_max_wrt_inner * partial_deriv_distance_pr(1), weight_ * deriv_smooth_max_wrt_inner * partial_deriv_distance_pr(2), 0, 0, 0).finished();
            }
            if (H2) { // Jacobian with respect to p_l
                (*H2) = (gtsam::Matrix(1, 4) << weight_ * deriv_smooth_max_wrt_inner * partial_deriv_distance_pl(0), weight_ * deriv_smooth_max_wrt_inner * partial_deriv_distance_pl(1), 0, 0).finished();
            }
        }
        return (Vector(1) << error_val).finished();
    }

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
                         gtsam::OptionalMatrixType H3) const override {
        Vector2 diff(2);
        diff << p_r(0) - p_l(0), p_r(1) - p_l(1);
        double distance = diff.norm();
        // Calculate the slack term: max(0, L_cable - distance).
        // This term is positive if the cable is slack (distance < L_cable).
        double slack_term_val = smooth_max_zero_(cable_length_ - distance);

        // The error value is `weight_ * T_k * slack_term_val`.
        // If T_k > 0 AND slack_term_val > 0, this factor incurs a penalty.
        double error_val = weight_ * T_k(0) * slack_term_val;

        if (H1 || H2 || H3) {
            // Derivatives for Jacobian calculations:
            // d(error_val)/dT_k = weight_ * slack_term_val
            // d(error_val)/dp_r = weight_ * T_k * d(slack_term_val)/dp_r
            // d(error_val)/dp_l = weight_ * T_k * d(slack_term_val)/dp_l

            // First, calculate derivative of smooth_max_zero(x) where x = (cable_length_ - distance).
            // d(smooth_max_zero(x))/dx = 0.5 * (1 + x / sqrt(x^2 + epsilon^2))
            double inner_term = cable_length_ - distance;
            double deriv_smooth_max_wrt_inner = 0.5 * (1.0 + inner_term / std::sqrt(inner_term*inner_term + 1e-6*1e-6));

            // Chain rule for d(slack_term_val)/d(distance): d(smooth_max_zero(C-D))/dD = d(smooth_max_zero)/d(inner) * d(inner)/dD
            // Here, d(inner)/dD = d(C-D)/dD = -1.
            double deriv_slack_wrt_distance = deriv_smooth_max_wrt_inner * (-1.0);

            // Derivatives of distance with respect to positions (unit vectors).
            Vector2 partial_deriv_distance_pr = diff / distance; // unit vector e
            Vector2 partial_deriv_distance_pl = -diff / distance; // -e vector

            if (H1) { // Jacobian with respect to T_k
                (*H1) = (Vector(1) << weight_ * slack_term_val).finished();
            }
            if (H2) { // Jacobian with respect to p_r
                // Chain rule: weight * T_k * d(slack_term_val)/d(distance) * d(distance)/dp_r
                (*H2) = (gtsam::Matrix(1, 4) << weight_ * T_k(0) * deriv_slack_wrt_distance * partial_deriv_distance_pr(0), weight_ * T_k(0) * deriv_slack_wrt_distance * partial_deriv_distance_pr(1), 0, 0).finished();
            }
            if (H3) { // Jacobian with respect to p_l
                // Chain rule: weight * T_k * d(slack_term_val)/d(distance) * d(distance)/dp_l
                (*H3) = (gtsam::Matrix(1, 4) << weight_ * T_k(0) * deriv_slack_wrt_distance * partial_deriv_distance_pl(0), weight_ * T_k(0) * deriv_slack_wrt_distance * partial_deriv_distance_pl(1), 0, 0).finished();
            }
        }
        return (Vector(1) << error_val).finished();
    }
};

class TensionSlackPenaltyWithHeightFactor : public NoiseModelFactor3<Vector1, Vector6, Vector4> {
private:
    double cable_length_; // The nominal length of the cable
    double weight_;       // Weight for this factor

    double smooth_max_zero_(double x, double epsilon = 1e-6) const {
        return 0.5 * (x + std::sqrt(x*x + epsilon*epsilon));
    }

public:
    TensionSlackPenaltyWithHeightFactor(Key tensionKey, Key robotPosKey, Key loadPosKey, double cableLength,
                               double weight, const SharedNoiseModel& model)
        : NoiseModelFactor3<Vector1, Vector6, Vector4>(model, tensionKey, robotPosKey, loadPosKey),
          cable_length_(cableLength), weight_(weight) {}

    Vector evaluateError(const Vector1& T_k, const Vector6& p_r, const Vector4& p_l,
                         gtsam::OptionalMatrixType H1,
                         gtsam::OptionalMatrixType H2,
                         gtsam::OptionalMatrixType H3) const override {
        Vector3 diff(3);
        diff << p_r(0) - p_l(0), p_r(1) - p_l(1), p_r(2) - 0.2;
        double distance = diff.norm();
        // Calculate the slack term: max(0, L_cable - distance).
        // This term is positive if the cable is slack (distance < L_cable).
        double slack_term_val = smooth_max_zero_(cable_length_ - distance);

        // The error value is `weight_ * T_k * slack_term_val`.
        // If T_k > 0 AND slack_term_val > 0, this factor incurs a penalty.
        double error_val = weight_ * T_k(0) * slack_term_val;

        if (H1 || H2 || H3) {
            // Derivatives for Jacobian calculations:
            // d(error_val)/dT_k = weight_ * slack_term_val
            // d(error_val)/dp_r = weight_ * T_k * d(slack_term_val)/dp_r
            // d(error_val)/dp_l = weight_ * T_k * d(slack_term_val)/dp_l

            // First, calculate derivative of smooth_max_zero(x) where x = (cable_length_ - distance).
            // d(smooth_max_zero(x))/dx = 0.5 * (1 + x / sqrt(x^2 + epsilon^2))
            double inner_term = cable_length_ - distance;
            double deriv_smooth_max_wrt_inner = 0.5 * (1.0 + inner_term / std::sqrt(inner_term*inner_term + 1e-6*1e-6));

            // Chain rule for d(slack_term_val)/d(distance): d(smooth_max_zero(C-D))/dD = d(smooth_max_zero)/d(inner) * d(inner)/dD
            // Here, d(inner)/dD = d(C-D)/dD = -1.
            double deriv_slack_wrt_distance = deriv_smooth_max_wrt_inner * (-1.0);

            // Derivatives of distance with respect to positions (unit vectors).
            Vector3 partial_deriv_distance_pr = diff / distance; // unit vector e
            Vector3 partial_deriv_distance_pl = -diff / distance; // -e vector

            if (H1) { // Jacobian with respect to T_k
                (*H1) = (Vector(1) << weight_ * slack_term_val).finished();
            }
            if (H2) { // Jacobian with respect to p_r
                // Chain rule: weight * T_k * d(slack_term_val)/d(distance) * d(distance)/dp_r
                (*H2) = (gtsam::Matrix(1, 6) << weight_ * T_k(0) * deriv_slack_wrt_distance * partial_deriv_distance_pr(0), weight_ * T_k(0) * deriv_slack_wrt_distance * partial_deriv_distance_pr(1), weight_ * T_k(0) * deriv_slack_wrt_distance * partial_deriv_distance_pr(2), 0, 0, 0).finished();
            }
            if (H3) { // Jacobian with respect to p_l
                // Chain rule: weight * T_k * d(slack_term_val)/d(distance) * d(distance)/dp_l
                (*H3) = (gtsam::Matrix(1, 4) << weight_ * T_k(0) * deriv_slack_wrt_distance * partial_deriv_distance_pl(0), weight_ * T_k(0) * deriv_slack_wrt_distance * partial_deriv_distance_pl(1), 0, 0).finished();
            }
        }
        return (Vector(1) << error_val).finished();
    }
};


class CableStretchPenaltyWithOrientationFactor : public NoiseModelFactor2<Vector4, Vector6> {
private:
    double cable_length_; // The nominal length of the cable
    double weight_;       // Weight for this factor
    double k_;

    double smooth_max_zero_(double x, double epsilon = 1e-6) const {
        return 0.5 * (x + std::sqrt(x*x + epsilon*epsilon));
    }

    Vector2 getAttachementPoint(const Vector6& xl_k) const {
        Vector2 p(2);
        p << xl_k(0) - 0.2 * cos(xl_k(2)), xl_k(1) - 0.2 * sin(xl_k(2));
        return p;
    }

    Vector2 getAttachementPoint2(const Vector6& xl_k) const {
        Vector2 p(2);
        p << xl_k(0) - 0.2 * sin(xl_k(2)), xl_k(1) + 0.2 * cos(xl_k(2));
        return p;
    }

public:
    CableStretchPenaltyWithOrientationFactor(Key robotPosKey, Key loadPosKey, double cableLength,
                              double weight, const SharedNoiseModel& model, int k = 1)
        : NoiseModelFactor2<Vector4, Vector6>(model, robotPosKey, loadPosKey),
          cable_length_(cableLength), weight_(weight), k_(k) {}

    Vector evaluateError(const Vector4& p_r, const Vector6& p_l,
                         gtsam::OptionalMatrixType H1,
                         gtsam::OptionalMatrixType H2) const override {

        Vector2 ap = getAttachementPoint(p_l);

        if (k_ == 2) {
            ap = getAttachementPoint2(p_l);
        }

        Vector2 diff(2);
        diff << p_r(0) - ap(0), p_r(1) - ap(1);
        double distance = diff.norm(); // Euclidean distance ||p_r - p_l||

        // The error value is `weight_ * smooth_max_zero(distance - cable_length_)`.
        double error_val = weight_ * smooth_max_zero_(distance - cable_length_);

        if (H1 || H2) {
            double inner_term = distance - cable_length_;
            double deriv_smooth_max_wrt_inner = 0.5 * (1.0 + inner_term / std::sqrt(inner_term*inner_term + 1e-6*1e-6));

            // The derivative of distance `||p_r - p_l||` with respect to `p_r` is `(p_r - p_l) / ||p_r - p_l||`,
            // which is the unit vector `e` pointing from load to robot.
            Vector2 partial_deriv_distance_pr = diff / distance;
            // The derivative with respect to `p_l` is `-(p_r - p_l) / ||p_r - p_l||`, or `-e`.
            Vector2 partial_deriv_distance_pl = -diff / distance;
            double partial_deriv_distance_pt = 0.2 * (diff(1) * cos(p_l(2)) - diff(0) * sin(p_l(2))) / distance;

            if (k_ == 2) {
                partial_deriv_distance_pt = 0.2 * (diff(0) * cos(p_l(2)) + diff(1) * sin(p_l(2))) / distance;
            }

            if (H1) { // Jacobian with respect to p_r
                // Chain rule: d(error)/dp_r = weight * d(smooth_max_zero)/d(inner) * d(inner)/d(distance) * d(distance)/dp_r
                // d(inner)/d(distance) is 1.0.
                (*H1) = (gtsam::Matrix(1, 4) << weight_ * deriv_smooth_max_wrt_inner * partial_deriv_distance_pr(0), weight_ * deriv_smooth_max_wrt_inner * partial_deriv_distance_pr(1), 0, 0).finished();
            }
            if (H2) { // Jacobian with respect to p_l
                (*H2) = (gtsam::Matrix(1, 6) << weight_ * deriv_smooth_max_wrt_inner * partial_deriv_distance_pl(0), weight_ * deriv_smooth_max_wrt_inner * partial_deriv_distance_pl(1), weight_ * deriv_smooth_max_wrt_inner * partial_deriv_distance_pt, 0, 0, 0).finished();
            }
        }
        return (Vector(1) << error_val).finished();
    }

};


class TensionSlackPenaltyWithLoadOrientationFactor : public NoiseModelFactor3<Vector1, Vector4, Vector6> {
private:
    double cable_length_; // The nominal length of the cable
    double weight_;       // Weight for this factor
    double k_;

    double smooth_max_zero_(double x, double epsilon = 1e-6) const {
        return 0.5 * (x + std::sqrt(x*x + epsilon*epsilon));
    }

    Vector2 getAttachementPoint(const Vector6& xl_k) const {
        Vector2 p(2);
        p << xl_k(0) - 0.2 * cos(xl_k(2)), xl_k(1) - 0.2 * sin(xl_k(2));
        return p;
    }

    Vector2 getAttachementPoint2(const Vector6& xl_k) const {
        Vector2 p(2);
        p << xl_k(0) - 0.2 * sin(xl_k(2)), xl_k(1) + 0.2 * cos(xl_k(2));
        return p;
    }

public:
    TensionSlackPenaltyWithLoadOrientationFactor(Key tensionKey, Key robotPosKey, Key loadPosKey, double cableLength,
                               double weight, const SharedNoiseModel& model, int k = 1)
        : NoiseModelFactor3<Vector1, Vector4, Vector6>(model, tensionKey, robotPosKey, loadPosKey),
          cable_length_(cableLength), weight_(weight), k_(k) {}

    Vector evaluateError(const Vector1& T_k, const Vector4& p_r, const Vector6& p_l,
                         gtsam::OptionalMatrixType H1,
                         gtsam::OptionalMatrixType H2,
                         gtsam::OptionalMatrixType H3) const override {
        
                            // find attachement point
        Vector2 p = getAttachementPoint(p_l);

        if (k_ == 2) {
            p = getAttachementPoint2(p_l);
        }

        
        // Vector2 p(2);
        // p << p_l(0) - 0.2 * cos(p_l(2)), p_l(1) - 0.2 * sin(p_l(2));

        Vector2 diff(2);
        diff << p_r(0) - p(0), p_r(1) - p(1);
        double distance = diff.norm();
        // Calculate the slack term: max(0, L_cable - distance).
        // This term is positive if the cable is slack (distance < L_cable).
        double slack_term_val = smooth_max_zero_(cable_length_ - distance);

        // The error value is `weight_ * T_k * slack_term_val`.
        // If T_k > 0 AND slack_term_val > 0, this factor incurs a penalty.
        double error_val = weight_ * T_k(0) * slack_term_val;

        if (H1 || H2 || H3) {
            // Derivatives for Jacobian calculations:
            // d(error_val)/dT_k = weight_ * slack_term_val
            // d(error_val)/dp_r = weight_ * T_k * d(slack_term_val)/dp_r
            // d(error_val)/dp_l = weight_ * T_k * d(slack_term_val)/dp_l

            // First, calculate derivative of smooth_max_zero(x) where x = (cable_length_ - distance).
            // d(smooth_max_zero(x))/dx = 0.5 * (1 + x / sqrt(x^2 + epsilon^2))
            double inner_term = cable_length_ - distance;
            double deriv_smooth_max_wrt_inner = 0.5 * (1.0 + inner_term / std::sqrt(inner_term*inner_term + 1e-6*1e-6));

            // Chain rule for d(slack_term_val)/d(distance): d(smooth_max_zero(C-D))/dD = d(smooth_max_zero)/d(inner) * d(inner)/dD
            // Here, d(inner)/dD = d(C-D)/dD = -1.
            double deriv_slack_wrt_distance = deriv_smooth_max_wrt_inner * (-1.0);

            // Derivatives of distance with respect to positions (unit vectors).
            Vector2 partial_deriv_distance_pr = diff / distance; // unit vector e
            Vector2 partial_deriv_distance_pl = -diff / distance; // -e vector
            Vector1 partial_deriv_distance_pt(1);
            partial_deriv_distance_pt(0) = 0.2 / distance * (diff(1) * cos(p_l(2)) - diff(0) * sin(p_l(2)));

            if (k_ == 2) {
                partial_deriv_distance_pt(0) = 0.2 * (diff(0) * cos(p_l(2)) + diff(1) * sin(p_l(2))) / distance;
            }

            if (H1) { // Jacobian with respect to T_k
                (*H1) = (Vector(1) << weight_ * slack_term_val).finished();
            }
            if (H2) { // Jacobian with respect to p_r
                // Chain rule: weight * T_k * d(slack_term_val)/d(distance) * d(distance)/dp_r
                (*H2) = (gtsam::Matrix(1, 4) << weight_ * T_k(0) * deriv_slack_wrt_distance * partial_deriv_distance_pr(0), weight_ * T_k(0) * deriv_slack_wrt_distance * partial_deriv_distance_pr(1), 0, 0).finished();
            }
            if (H3) { // Jacobian with respect to p_l
                // Chain rule: weight * T_k * d(slack_term_val)/d(distance) * d(distance)/dp_l
                (*H3) = (gtsam::Matrix(1, 6) << weight_ * T_k(0) * deriv_slack_wrt_distance * partial_deriv_distance_pl(0), weight_ * T_k(0) * deriv_slack_wrt_distance * partial_deriv_distance_pl(1), weight_ * T_k(0) * deriv_slack_wrt_distance * partial_deriv_distance_pt, 0, 0, 0).finished();
            }
        }
        return (Vector(1) << error_val).finished();
    }
};


class TethertensionFactor : public NoiseModelFactor3<Vector1, Vector4, Vector6> {
private:
    double cable_length_; // The nominal length of the cable
    double weight_;       // Weight for this factor
    double k_;

    double smooth_max_zero_(double x, double epsilon = 1e-6) const {
        return 0.5 * (x + std::sqrt(x*x + epsilon*epsilon));
    }

    Vector2 getAttachementPoint(const Vector6& xl_k) const {
        Vector2 p(2);
        p << xl_k(0) - 0.2 * cos(xl_k(2)), xl_k(1) - 0.2 * sin(xl_k(2));
        return p;
    }

    Vector2 getAttachementPoint2(const Vector6& xl_k) const {
        Vector2 p(2);
        p << xl_k(0) - 0.2 * sin(xl_k(2)), xl_k(1) + 0.2 * cos(xl_k(2));
        return p;
    }

public:
    TethertensionFactor(Key tensionKey, Key robotPosKey, Key loadPosKey, double cableLength,
                               double weight, const SharedNoiseModel& model, int k = 1)
        : NoiseModelFactor3<Vector1, Vector4, Vector6>(model, tensionKey, robotPosKey, loadPosKey),
          cable_length_(cableLength), weight_(weight), k_(k) {}

    Vector evaluateError(const Vector1& T_k, const Vector4& p_r, const Vector6& p_l,
                         gtsam::OptionalMatrixType H1,
                         gtsam::OptionalMatrixType H2,
                         gtsam::OptionalMatrixType H3) const override {

        Vector2 p = getAttachementPoint(p_l);

        if (k_ == 2) {
            p = getAttachementPoint2(p_l);
        }

        Vector2 diff(2);
        diff << p_r(0) - p(0), p_r(1) - p(1);
        double distance = diff.norm();
        
        double slack_term_val = smooth_max_zero_(distance - cable_length_);
        

        double error_val = T_k(0) - weight_ * slack_term_val;

        //cout << "TethertensionFactor: " << slack_term_val << ' '<< T_k(0) << ' ' << error_val << endl;

        double inner_term = distance - cable_length_;
        double deriv_smooth_max_wrt_inner = 0.5 * (1.0 + inner_term / std::sqrt(inner_term*inner_term + 1e-6*1e-6));
        Vector2 partial_deriv_distance_pr = diff / distance; // unit vector e
        Vector2 partial_deriv_distance_pl = -diff / distance; // -e vector
        double partial_deriv_distance_pt = 0.2 / distance * (diff(1) * cos(p_l(2)) - diff(0) * sin(p_l(2)));

        if (k_ == 2) {
            partial_deriv_distance_pt = 0.2 * (diff(0) * cos(p_l(2)) + diff(1) * sin(p_l(2))) / distance;
        }

        if (H1) { // Jacobian with respect to T_k
            (*H1) = (Vector(1) << 1).finished();
        }
        if (H2) { // Jacobian with respect to p_r
            // Chain rule: weight * T_k * d(slack_term_val)/d(distance) * d(distance)/dp_r
            (*H2) = (gtsam::Matrix(1, 4) << -weight_ * deriv_smooth_max_wrt_inner * partial_deriv_distance_pr(0), -weight_ * deriv_smooth_max_wrt_inner * partial_deriv_distance_pr(1), 0, 0).finished();
        }
        if (H3) { // Jacobian with respect to p_l
            // Chain rule: weight * T_k * d(slack_term_val)/d(distance) * d(distance)/dp_l
            (*H3) = (gtsam::Matrix(1, 6) << -weight_ * deriv_smooth_max_wrt_inner * partial_deriv_distance_pl(0), -weight_ * deriv_smooth_max_wrt_inner * partial_deriv_distance_pl(1), -weight_ * deriv_smooth_max_wrt_inner * partial_deriv_distance_pt, 0, 0, 0).finished();
        }

        return (Vector(1) << error_val).finished();
    }

};

class TethertensionFactorNoOri : public NoiseModelFactor3<Vector1, Vector4, Vector4> {
private:
    double cable_length_; // The nominal length of the cable
    double weight_;       // Weight for this factor
    double k_;

    double smooth_max_zero_(double x, double epsilon = 1e-6) const {
        return 0.5 * (x + std::sqrt(x*x + epsilon*epsilon));
    }

public:
    TethertensionFactorNoOri(Key tensionKey, Key robotPosKey, Key loadPosKey, double cableLength,
                               double weight, const SharedNoiseModel& model, int k = 1)
        : NoiseModelFactor3<Vector1, Vector4, Vector4>(model, tensionKey, robotPosKey, loadPosKey),
          cable_length_(cableLength), weight_(weight), k_(k) {}

    Vector evaluateError(const Vector1& T_k, const Vector4& p_r, const Vector4& p_l,
                         gtsam::OptionalMatrixType H1,
                         gtsam::OptionalMatrixType H2,
                         gtsam::OptionalMatrixType H3) const override {

        Vector2 diff(2);
        diff << p_r(0) - p_l(0), p_r(1) - p_l(1);
        double distance = diff.norm();
        
        double slack_term_val = smooth_max_zero_(distance - cable_length_);
        

        double error_val = T_k(0) - weight_ * slack_term_val;

        //cout << "TethertensionFactor: " << slack_term_val << ' '<< T_k(0) << ' ' << error_val << endl;

        double inner_term = distance - cable_length_;
        double deriv_smooth_max_wrt_inner = 0.5 * (1.0 + inner_term / std::sqrt(inner_term*inner_term + 1e-6*1e-6));
        Vector2 partial_deriv_distance_pr = diff / distance; // unit vector e
        Vector2 partial_deriv_distance_pl = -diff / distance; // -e vector

        if (H1) { // Jacobian with respect to T_k
            (*H1) = (Vector(1) << 1).finished();
        }
        if (H2) { // Jacobian with respect to p_r
            // Chain rule: weight * T_k * d(slack_term_val)/d(distance) * d(distance)/dp_r
            (*H2) = (gtsam::Matrix(1, 4) << -weight_ * deriv_smooth_max_wrt_inner * partial_deriv_distance_pr(0), -weight_ * deriv_smooth_max_wrt_inner * partial_deriv_distance_pr(1), 0, 0).finished();
        }
        if (H3) { // Jacobian with respect to p_l
            // Chain rule: weight * T_k * d(slack_term_val)/d(distance) * d(distance)/dp_l
            (*H3) = (gtsam::Matrix(1, 4) << -weight_ * deriv_smooth_max_wrt_inner * partial_deriv_distance_pl(0), -weight_ * deriv_smooth_max_wrt_inner * partial_deriv_distance_pl(1), 0, 0).finished();
        }

        return (Vector(1) << error_val).finished();
    }

};

class TethertensionFactorNoOriWithHeight : public NoiseModelFactor3<Vector1, Vector6, Vector4> {
private:
    double cable_length_; // The nominal length of the cable
    double weight_;       // Weight for this factor
    double k_;
    string ap_;

    double smooth_max_zero_(double x, double epsilon = 1e-6) const {
        return 0.5 * (x + std::sqrt(x*x + epsilon*epsilon));
    }

public:
    TethertensionFactorNoOriWithHeight(Key tensionKey, Key robotPosKey, Key loadPosKey, double cableLength,
                               double weight, const SharedNoiseModel& model, string ap, int k = 1)
        : NoiseModelFactor3<Vector1, Vector6, Vector4>(model, tensionKey, robotPosKey, loadPosKey),
          cable_length_(cableLength), weight_(weight), ap_(ap), k_(k) {}

    Vector evaluateError(const Vector1& T_k, const Vector6& p_r, const Vector4& p_l,
                         gtsam::OptionalMatrixType H1,
                         gtsam::OptionalMatrixType H2,
                         gtsam::OptionalMatrixType H3) const override {

        double pl_z = (ap_ == "bottom") ? 0.0 : 0.2;
        Vector3 diff(3);
        diff << p_r(0) - p_l(0), p_r(1) - p_l(1), p_r(2) - pl_z;
        double distance = diff.norm();
        
        double slack_term_val = smooth_max_zero_(distance - cable_length_);
        

        double error_val = T_k(0) - weight_ * slack_term_val;

        //cout << "TethertensionFactor: " << slack_term_val << ' '<< T_k(0) << ' ' << error_val << endl;

        double inner_term = distance - cable_length_;
        double deriv_smooth_max_wrt_inner = 0.5 * (1.0 + inner_term / std::sqrt(inner_term*inner_term + 1e-6*1e-6));
        Vector3 partial_deriv_distance_pr = diff / distance; // unit vector e
        Vector3 partial_deriv_distance_pl = -diff / distance; // -e vector

        if (H1) { // Jacobian with respect to T_k
            (*H1) = (Vector(1) << 1).finished();
        }
        if (H2) { // Jacobian with respect to p_r
            // Chain rule: weight * T_k * d(slack_term_val)/d(distance) * d(distance)/dp_r
            (*H2) = (gtsam::Matrix(1, 6) << -weight_ * deriv_smooth_max_wrt_inner * partial_deriv_distance_pr(0), -weight_ * deriv_smooth_max_wrt_inner * partial_deriv_distance_pr(1), -weight_ * deriv_smooth_max_wrt_inner * partial_deriv_distance_pr(2), 0, 0, 0).finished();
        }
        if (H3) { // Jacobian with respect to p_l
            // Chain rule: weight * T_k * d(slack_term_val)/d(distance) * d(distance)/dp_l
            (*H3) = (gtsam::Matrix(1, 4) << -weight_ * deriv_smooth_max_wrt_inner * partial_deriv_distance_pl(0), -weight_ * deriv_smooth_max_wrt_inner * partial_deriv_distance_pl(1), 0, 0).finished();
        }

        return (Vector(1) << error_val).finished();
    }

};


class TethertensionFactorWithHeight : public NoiseModelFactor3<Vector1, Vector6, Vector6> {
private:
    double cable_length_; // The nominal length of the cable
    double weight_;       // Weight for this factor
    double k_;
    string ap_;

    double smooth_max_zero_(double x, double epsilon = 1e-6) const {
        return 0.5 * (x + std::sqrt(x*x + epsilon*epsilon));
    }

    Vector2 getAttachementPoint(const Vector6& xl_k) const {
        Vector2 p(2);
        p << xl_k(0) - 0.2 * cos(xl_k(2)), xl_k(1) - 0.2 * sin(xl_k(2));
        return p;
    }

public:
    TethertensionFactorWithHeight(Key tensionKey, Key robotPosKey, Key loadPosKey, double cableLength,
                               double weight, const SharedNoiseModel& model, string ap, int k = 1)
        : NoiseModelFactor3<Vector1, Vector6, Vector6>(model, tensionKey, robotPosKey, loadPosKey),
          cable_length_(cableLength), weight_(weight), ap_(ap), k_(k) {}

    Vector evaluateError(const Vector1& T_k, const Vector6& p_r, const Vector6& p_l,
                         gtsam::OptionalMatrixType H1,
                         gtsam::OptionalMatrixType H2,
                         gtsam::OptionalMatrixType H3) const override {

        double pl_z = (ap_ == "bottom") ? 0.0 : 0.2;
        Vector3 diff(3);
        Vector2 p = getAttachementPoint(p_l);

        diff << p_r(0) - p(0), p_r(1) - p(1), p_r(2) - pl_z;
        double distance = diff.norm();
        
        double slack_term_val = smooth_max_zero_(distance - cable_length_);
        

        double error_val = T_k(0) - weight_ * slack_term_val;

        //cout << "TethertensionFactor: " << slack_term_val << ' '<< T_k(0) << ' ' << error_val << endl;

        double inner_term = distance - cable_length_;
        double deriv_smooth_max_wrt_inner = 0.5 * (1.0 + inner_term / std::sqrt(inner_term*inner_term + 1e-6*1e-6));
        Vector3 partial_deriv_distance_pr = diff / distance; // unit vector e
        Vector3 partial_deriv_distance_pl = -diff / distance; // -e vector
        double partial_deriv_distance_pt = 0.2 / distance * (diff(1) * cos(p_l(2)) - diff(0) * sin(p_l(2)));

        if (H1) { // Jacobian with respect to T_k
            (*H1) = (Vector(1) << 1).finished();
        }
        if (H2) { // Jacobian with respect to p_r
            // Chain rule: weight * T_k * d(slack_term_val)/d(distance) * d(distance)/dp_r
            (*H2) = (gtsam::Matrix(1, 6) << -weight_ * deriv_smooth_max_wrt_inner * partial_deriv_distance_pr(0), -weight_ * deriv_smooth_max_wrt_inner * partial_deriv_distance_pr(1), -weight_ * deriv_smooth_max_wrt_inner * partial_deriv_distance_pr(2), 0, 0, 0).finished();
        }
        if (H3) { // Jacobian with respect to p_l
            // Chain rule: weight * T_k * d(slack_term_val)/d(distance) * d(distance)/dp_l
            (*H3) = (gtsam::Matrix(1, 6) << -weight_ * deriv_smooth_max_wrt_inner * partial_deriv_distance_pl(0), -weight_ * deriv_smooth_max_wrt_inner * partial_deriv_distance_pl(1), -weight_ * deriv_smooth_max_wrt_inner * partial_deriv_distance_pt, 0, 0, 0).finished();
        }

        return (Vector(1) << error_val).finished();
    }

};

#endif // CABLE_FACTORS_HPP