#include "factor_graph_lib/dynamics_factors.hpp"

using namespace gtsam;


Vector RobotDynamicsFactor::evaluateError(const Vector4& xr_k,
                         const Vector4& xl_k,
                         const Vector2& u_k, 
                         const Vector1& tension_k,
                         const Vector4& xr_k_plus_1,
                         gtsam::OptionalMatrixType H1,
                         gtsam::OptionalMatrixType H2,
                         gtsam::OptionalMatrixType H3,
                         gtsam::OptionalMatrixType H4,
                         gtsam::OptionalMatrixType H5) const {
    
    // Unpack state: xr = [px, py, vx, vy]
    Vector2 pos_k = xr_k.head<2>();
    Vector2 vel_k = xr_k.tail<2>();

    // Simple Euler integration for dynamics
    // next_pos = current_pos + dt * current_vel
    // next_vel = current_vel + dt * (control_force/mass - tension_force/mass)
    Vector2 next_pos = pos_k + vel_k * dt_;

    double xd = xr_k(0) - xl_k(0);
    double yd = xr_k(1) - xl_k(1);
    Vector2 e(2);
    e << xd, yd;

    double norm = e.norm();
    Vector2 e_norm(2);
    e_norm << e / norm;

    Vector2 next_vel = vel_k + (u_k - tension_k(0) * e_norm) / robot_mass_ * dt_;

    Vector4 predicted_xr_k_plus_1(4);
    predicted_xr_k_plus_1 << next_pos, next_vel;

    double ONE = 1.0 / norm - xd * xd / (norm * norm * norm);
    double TWO = -1.0 / norm + xd * xd / (norm * norm * norm);
    double THREE = -xd * yd / (norm * norm * norm);
    double FOUR = xd * yd / (norm * norm * norm);
    double FIVE = 1.0 / norm - yd * yd / (norm * norm * norm);
    double SIX = -1.0 / norm + yd * yd / (norm * norm * norm);

    if (H1) {
        // H1 is a valid reference to a matrix, so you can assign to it.
        *H1 = (gtsam::Matrix(4, 4) << 
            -1,  0, -dt_, 0,
             0, -1,  0,  -dt_,
             dt_ * tension_k(0) * ONE / robot_mass_,  dt_ * tension_k(0) * THREE / robot_mass_, -1,   0,
             dt_ * tension_k(0) * THREE / robot_mass_,  dt_ * tension_k(0) * FIVE / robot_mass_,  0,  -1).finished();

    }
    if (H2) {
        *H2 = (gtsam::Matrix(4, 4) << 
            0, 0, 0, 0,
            0, 0, 0, 0,
             dt_ * tension_k(0) * TWO / robot_mass_,  dt_ * tension_k(0) * FOUR / robot_mass_, 0,   0,
             dt_ * tension_k(0) * FOUR / robot_mass_,  dt_ * tension_k(0) * SIX / robot_mass_,  0,  0).finished();

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
            (dt_ * xd / (robot_mass_ * norm)), 
            (dt_ * yd / (robot_mass_ * norm))).finished();
    }
    if (H5) {
        *H5 = gtsam::Matrix4::Identity();
    }

    return (Vector(4) << xr_k_plus_1 - predicted_xr_k_plus_1).finished();
}


Vector LoadDynamicsFactor::evaluateError(const Vector4& xl_k, 
                         const Vector4& xr_k,
                         const Vector1& tension_k,
                         const Vector4& xl_k_plus_1,
                         gtsam::OptionalMatrixType H1,
                         gtsam::OptionalMatrixType H2,
                         gtsam::OptionalMatrixType H3,
                         gtsam::OptionalMatrixType H4) const {
        
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

    double xd = xr_k(0) - xl_k(0);
    double yd = xr_k(1) - xl_k(1);
    Vector2 e(2);
    e << xd, yd;

    double norm = e.norm();
    Vector2 e_norm(2);
    e_norm << e / e.norm();

    Vector2 next_vel = vel_k + (tension_k(0) * e_norm - mu_ * load_mass_ * g_ * normed_vel_k) / load_mass_ * dt_;

    Vector4 predicted_xl_k_plus_1(4);
    predicted_xl_k_plus_1 << next_pos, next_vel;

    double ONE = 1.0 / norm - xd * xd / (norm * norm * norm);
    double TWO = -1.0 / norm + xd * xd / (norm * norm * norm);
    double THREE = -yd * xd / (norm * norm * norm);
    double FOUR = xd * yd / (norm * norm * norm);
    double FIVE = 1.0 / norm - yd * yd / (norm * norm * norm);
    double SIX = -1.0 / norm + yd * yd / (norm * norm * norm);

    if (H1) {
        *H1 = (gtsam::Matrix(4, 4) << 
            -1,  0, -dt_, 0,
             0, -1,  0,  -dt_,
             -dt_ * tension_k(0) * TWO / load_mass_,  -dt_ * tension_k(0) * FOUR / load_mass_, -1 + dt_ * mu_ * g_ * SEVEN, dt_ * mu_ * g_ * NINE,
             -dt_ * tension_k(0) * FOUR / load_mass_,  -dt_ * tension_k(0) * SIX / load_mass_,  dt_ * mu_ * g_ * NINE,  -1 + dt_ * mu_ * g_ * EIGHT).finished();
    }
    if (H2) {
        *H2 = (gtsam::Matrix(4, 4) << 
            0,  0, 0, 0,
             0, 0,  0,  0,
             -dt_ * tension_k(0) * ONE / load_mass_,  -dt_ * tension_k(0) * THREE / load_mass_, 0, 0,
             -dt_ * tension_k(0) * THREE / load_mass_,  -dt_ * tension_k(0) * FIVE / load_mass_, 0, 0).finished();
    }
    if (H3) {
        *H3 = (gtsam::Matrix(4, 1) << 
            0,
            0,
            -(dt_ * xd / (load_mass_ * norm)),
            -(dt_ * yd / (load_mass_ * norm))).finished();
    }
    if (H4) {
        *H4 = gtsam::Matrix4::Identity();
    }

    return (Vector(4) << xl_k_plus_1 - predicted_xl_k_plus_1).finished();
}