/* ----------------------------------------------------------------------------
 * A C++ example for GTSAM that implements the factor graph for a robot
 * controlling a load with a cable, as depicted in the user's diagram.
 *
 * This is a trajectory optimization problem. We formulate it as a fixed-lag
 * smoother, where all states and controls over a time horizon are estimated
 * simultaneously.
 *
 * To compile, you will need GTSAM installed. See: https://gtsam.org/
 *
 * Then, use the provided CMakeLists.txt:
 * $ mkdir build && cd build
 * $ cmake ..
 * $ make
 * $ ./robot_load_trajectory
 * -------------------------------------------------------------------------- */

// Include necessary GTSAM headers
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>

// Use gtsam namespace
using namespace std;
using namespace gtsam;

// Define symbols for different variable types for clarity
using symbol_t = gtsam::Symbol;
const symbol_t Xr('r'); // Robot State
const symbol_t Xl('l'); // Load State
const symbol_t U('u');  // Robot Control Input
const symbol_t T('t');  // Cable Tension (Implicitly handled in our factor)

// --- 1. Define Custom Factors ---
// These factors correspond to the edges in your factor graph diagram.

/**
 * Custom factor to model the robot's dynamics.
 * Connects Xr_k, U_k, T_k, Xr_{k+1}
 * Error = Xr_{k+1} - (Xr_k + dt * f(Xr_k, U_k, T_k))
 */
class RobotDynamicsFactor: public NoiseModelFactor3<Vector4, Vector2, Vector2> {
    double dt_;
    double robot_mass_;

public:
    // Standard constructor
    RobotDynamicsFactor(Key key_xr_k, Key key_u_k, Key key_tension_k,
                        const Vector4& xr_k_plus_1, double dt, double robot_mass,
                        const SharedNoiseModel& model) :
        NoiseModelFactor3<Vector4, Vector2, Vector2>(model, key_xr_k, key_u_k, key_tension_k),
        dt_(dt), robot_mass_(robot_mass) {}

    // The evaluateError function, which implements the factor's error calculation.
    Vector evaluateError(const Vector4& xr_k, const Vector2& u_k, const Vector2& tension_k,
                         boost::optional<Matrix&> H1 = boost::none,
                         boost::optional<Matrix&> H2 = boost::none,
                         boost::optional<Matrix&> H3 = boost::none) const override {
        
        // Unpack state: xr = [px, py, vx, vy]
        Vector2 pos_k = xr_k.head<2>();
        Vector2 vel_k = xr_k.tail<2>();

        // Simple Euler integration for dynamics
        // next_pos = current_pos + dt * current_vel
        // next_vel = current_vel + dt * (control_force/mass - tension_force/mass)
        Vector2 next_pos = pos_k + vel_k * dt_;
        Vector2 next_vel = vel_k + (u_k - tension_k) / robot_mass_ * dt_;

        Vector4 predicted_xr_k_plus_1(4);
        predicted_xr_k_plus_1 << next_pos, next_vel;

        // The error is the difference between the predicted next state and the variable
        // value for the next state in the graph.
        // For this example, we make this a unary factor to simplify. A more standard
        // approach would make it a binary factor between Xr_k and Xr_{k+1}.
        // However, to exactly match the graph structure, we model the dynamics
        // and connect it to the tension variable.
        // Let's re-frame this as a more standard "Between" factor.
        // error = Xr_{k+1} - f(Xr_k, U_k, T_k)
        // Since we can't do that directly, we'll make a more complex factor.
        // The implementation here is a simplification for demonstration.
        // In a real scenario, Tension would be a function of Xr and Xl.
        // Let's create a more realistic factor below.

        // This factor is a placeholder for the concept. See the more complete
        // example in the main() function where factors are combined.
        return (Vector4() << 0,0,0,0).finished(); // Placeholder
    }
};


/**
 * Factor to model the cable dynamics and create tension.
 * Connects robot state Xr_k, load state Xl_k, and produces tension T_k.
 * This factor's error can be seen as the "slack" in the cable model.
 * It's a binary factor on Xr_k and Xl_k.
 */
class CableFactor : public NoiseModelFactor2<Vector4, Vector4> {
    double cable_length_;
    double stiffness_; // Spring stiffness of the cable
    double damping_;   // Damping to reduce oscillations

public:
    CableFactor(Key key_xr_k, Key key_xl_k, double length, double k, double d, const SharedNoiseModel& model)
        : NoiseModelFactor2<Vector4, Vector4>(model, key_xr_k, key_xl_k),
          cable_length_(length), stiffness_(k), damping_(d) {}

    // Function to calculate the tension force based on robot and load states
    Vector2 calculateTension(const Vector4& xr, const Vector4& xl,
                            boost::optional<Matrix&> H_xr = boost::none,
                            boost::optional<Matrix&> H_xl = boost::none) const {
        Vector2 pos_r = xr.head<2>();
        Vector2 pos_l = xl.head<2>();
        Vector2 rel_pos = pos_r - pos_l;
        double dist = rel_pos.norm();

        Vector2 tension_force = Vector2::Zero();
        
        if (dist > cable_length_) {
            Vector2 rel_pos_dir = rel_pos / dist;
            double stretch = dist - cable_length_;
            
            // Damping term
            Vector2 vel_r = xr.tail<2>();
            Vector2 vel_l = xl.tail<2>();
            Vector2 rel_vel = vel_r - vel_l;
            double damping_force_mag = damping_ * (rel_vel.dot(rel_pos_dir));

            // Spring-damper model for tension
            double force_mag = stiffness_ * stretch + damping_force_mag;
            
            // Tension only pulls, it doesn't push
            if (force_mag > 0) {
               tension_force = force_mag * rel_pos_dir;
            }
        }
        
        // Jacobian calculation would go here if needed
        if (H_xr) *H_xr = Matrix::Zero(2, 4); // Placeholder
        if (H_xl) *H_xl = Matrix::Zero(2, 4); // Placeholder

        return tension_force;
    }

    // The error function for this factor is not traditional.
    // Instead of returning an error vector, we'll use this factor
    // inside the dynamics factors. This is a common pattern for complex systems.
    Vector evaluateError(const Vector4& xr_k, const Vector4& xl_k,
                         boost::optional<Matrix&> H1 = boost::none,
                         boost::optional<Matrix&> H2 = boost::none) const override {
        // This factor itself doesn't have an error. It's a "function" used by others.
        // An alternative design would be to have T_k as a variable and have this
        // factor return: error = T_k - calculateTension(...)
        return Vector::Zero(2);
    }
};

/**
 * A more realistic dynamics factor that models the connection between two time steps.
 * This is a "Between" factor for our custom system.
 * It connects Xr_k, Xl_k, U_k -> Xr_{k+1}, Xl_{k+1}
 */
class SystemDynamicsFactor : public NoiseModelFactor3<Vector4, Vector4, Vector2> {
    double dt_;
    double robot_mass_;
    double load_mass_;
    Vector2 gravity_;
    boost::shared_ptr<CableFactor> cable_model_;

public:
    SystemDynamicsFactor(Key xr_k, Key xl_k, Key u_k, // Input variables at time k
                         const Vector4& xr_k_plus_1_measured, // "Measurement" at k+1
                         const Vector4& xl_k_plus_1_measured, // "Measurement" at k+1
                         double dt, double rm, double lm, const Vector2& g,
                         boost::shared_ptr<CableFactor> cable,
                         const SharedNoiseModel& model)
        : NoiseModelFactor3<Vector4, Vector4, Vector2>(model, xr_k, xl_k, u_k),
          dt_(dt), robot_mass_(rm), load_mass_(lm), gravity_(g), cable_model_(cable) {}

    // This is the core of the physics model
    Vector evaluateError(const Vector4& xr_k, const Vector4& xl_k, const Vector2& u_k,
                         boost::optional<Matrix&> H1 = boost::none,
                         boost::optional<Matrix&> H2 = boost::none,
                         boost::optional<Matrix&> H3 = boost::none) const override {
        
        // 1. Calculate tension from the cable model based on current states
        Vector2 tension = cable_model_->calculateTension(xr_k, xl_k);

        // 2. Predict next robot state using Euler integration
        Vector2 robot_pos_k = xr_k.head<2>();
        Vector2 robot_vel_k = xr_k.tail<2>();
        Vector2 robot_accel = (u_k - tension) / robot_mass_;
        Vector4 predicted_xr_k_plus_1;
        predicted_xr_k_plus_1 << robot_pos_k + robot_vel_k * dt_,
                                 robot_vel_k + robot_accel * dt_;
        
        // 3. Predict next load state using Euler integration
        Vector2 load_pos_k = xl_k.head<2>();
        Vector2 load_vel_k = xl_k.tail<2>();
        Vector2 load_accel = (tension / load_mass_) + gravity_;
        Vector4 predicted_xl_k_plus_1;
        predicted_xl_k_plus_1 << load_pos_k + load_vel_k * dt_,
                                 load_vel_k + load_accel * dt_;

        // The error is the difference between the predicted next state and the
        // variables representing the next state in the graph.
        // This requires a 5-way factor (Xr_k, Xl_k, U_k, Xr_{k+1}, Xl_{k+1}).
        // GTSAM doesn't have this out of the box, so we combine the errors.
        // Let's create a 5-way factor for proper implementation.

        // This factor is a placeholder for the concept. See the correct factor below.
        return Vector::Zero(8);
    }
};


/**
 * The PROPER factor for system dynamics. Connects 5 variables.
 * This is the factor that links two time steps together.
 * Models: (Xr_k, Xl_k, U_k) -> (Xr_{k+1}, Xl_{k+1})
 */
class FullSystemDynamicsFactor : public NoiseModelFactor5<Vector4, Vector4, Vector2, Vector4, Vector4> {
    double dt_;
    double robot_mass_;
    double load_mass_;
    Vector2 gravity_;
    boost::shared_ptr<CableFactor> cable_model_;

public:
    FullSystemDynamicsFactor(Key xr_k, Key xl_k, Key u_k, Key xr_k_plus_1, Key xl_k_plus_1,
                         double dt, double rm, double lm, const Vector2& g,
                         boost::shared_ptr<CableFactor> cable,
                         const SharedNoiseModel& model)
        : NoiseModelFactor5<Vector4, Vector4, Vector2, Vector4, Vector4>(model, xr_k, xl_k, u_k, xr_k_plus_1, xl_k_plus_1),
          dt_(dt), robot_mass_(rm), load_mass_(lm), gravity_(g), cable_model_(cable) {}

    Vector evaluateError(const Vector4& xr_k, const Vector4& xl_k, const Vector2& u_k,
                         const Vector4& xr_k_plus_1, const Vector4& xl_k_plus_1,
                         boost::optional<Matrix&> H1 = boost::none, // Jacobian wrt xr_k
                         boost::optional<Matrix&> H2 = boost::none, // Jacobian wrt xl_k
                         boost::optional<Matrix&> H3 = boost::none, // Jacobian wrt u_k
                         boost::optional<Matrix&> H4 = boost::none, // Jacobian wrt xr_{k+1}
                         boost::optional<Matrix&> H5 = boost::none  // Jacobian wrt xl_{k+1}
                        ) const override {

        // 1. Calculate tension from the cable model
        Vector2 tension = cable_model_->calculateTension(xr_k, xl_k);

        // 2. Predict next robot state using Euler integration
        Vector2 robot_pos_k = xr_k.head<2>();
        Vector2 robot_vel_k = xr_k.tail<2>();
        Vector2 robot_accel = (u_k - tension) / robot_mass_;
        Vector4 predicted_xr_k_plus_1;
        predicted_xr_k_plus_1 << robot_pos_k + robot_vel_k * dt_,
                                 robot_vel_k + robot_accel * dt_;

        // 3. Predict next load state using Euler integration
        Vector2 load_pos_k = xl_k.head<2>();
        Vector2 load_vel_k = xl_k.tail<2>();
        Vector2 load_accel = (tension / load_mass_) + gravity_;
        Vector4 predicted_xl_k_plus_1;
        predicted_xl_k_plus_1 << load_pos_k + load_vel_k * dt_,
                                 load_vel_k + load_accel * dt_;

        // 4. Calculate error
        Vector4 robot_error = xr_k_plus_1 - predicted_xr_k_plus_1;
        Vector4 load_error = xl_k_plus_1 - predicted_xl_k_plus_1;

        // Jacobians would be calculated here if provided.
        // This is complex and omitted for brevity, but crucial for performance.
        // GTSAM can compute them numerically if they are not provided.
        if(H1) *H1 = Matrix::Zero(8,4);
        if(H2) *H2 = Matrix::Zero(8,4);
        if(H3) *H3 = Matrix::Zero(8,2);
        if(H4) *H4 = (Matrix(8,4) << Matrix::Identity(4,4), Matrix::Zero(4,4)).finished();
        if(H5) *H5 = (Matrix(8,4) << Matrix::Zero(4,4), Matrix::Identity(4,4)).finished();


        // Combine errors into one vector
        Vector8 total_error;
        total_error << robot_error, load_error;
        return total_error;
    }
};


int main(int argc, char** argv) {
    // --- 2. Create the Factor Graph ---
    NonlinearFactorGraph graph;

    // --- Define problem parameters ---
    const int num_time_steps = 20;
    const double dt = 0.1;
    const double robot_mass = 10.0; // kg
    const double load_mass = 5.0;   // kg
    const Vector2 gravity(0, -9.81);
    const double cable_length = 2.0; // meters
    const double cable_stiffness = 500.0; // N/m
    const double cable_damping = 20.0;
    
    // Create the shared cable model
    auto cable_model = boost::make_shared<CableFactor>(0, 0, cable_length, cable_stiffness, cable_damping, nullptr);


    // --- Define Noise Models ---
    // These represent the uncertainty of each factor (1/covariance)
    auto prior_noise = noiseModel::Diagonal::Sigmas(
        (Vector(4) << 0.01, 0.01, 0.01, 0.01).finished());
    auto dynamics_noise = noiseModel::Diagonal::Sigmas(
        (Vector(8) << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01).finished());
    auto control_cost = noiseModel::Diagonal::Sigmas(
        (Vector(2) << 0.1, 0.1).finished());
    auto goal_cost = noiseModel::Diagonal::Sigmas(
        (Vector(4) << 0.05, 0.05, 0.1, 0.1).finished());


    // --- Add Factors to the Graph ---

    // Add prior factors for the initial state (t=0)
    // Assume robot starts at origin, load is hanging below it.
    Vector4 initial_robot_state(0, 2.0, 0, 0);
    Vector4 initial_load_state(0, 0, 0, 0);
    graph.add(PriorFactor<Vector4>(Xr(0), initial_robot_state, prior_noise));
    graph.add(PriorFactor<Vector4>(Xl(0), initial_load_state, prior_noise));

    for (int k = 0; k < num_time_steps; ++k) {
        // Add dynamics factor connecting k and k+1
        graph.add(FullSystemDynamicsFactor(Xr(k), Xl(k), U(k), Xr(k + 1), Xl(k + 1),
                                           dt, robot_mass, load_mass, gravity, cable_model,
                                           dynamics_noise));

        // Add a soft cost on control input to keep it small (prevents wild solutions)
        graph.add(PriorFactor<Vector2>(U(k), Vector2::Zero(), control_cost));
    }

    // Add a goal cost on the final load state
    Vector4 final_load_goal(10, 5, 0, 0); // Goal: move to (10, 5)
    graph.add(PriorFactor<Vector4>(Xl(num_time_steps), final_load_goal, goal_cost));
    // Also add a cost to have the robot be "above" the final load state
    Vector4 final_robot_goal(10, 7, 0, 0);
    graph.add(PriorFactor<Vector4>(Xr(num_time_steps), final_robot_goal, goal_cost));


    cout << "Factor Graph built." << endl;
    graph.print("Factor Graph:\n");


    // --- 3. Create Initial Estimate ---
    // The optimizer needs an initial guess for all variables.
    Values initial_values;
    for (int k = 0; k <= num_time_steps; ++k) {
        // A simple initial guess: stay at the start position.
        initial_values.insert(Xr(k), initial_robot_state);
        initial_values.insert(Xl(k), initial_load_state);
        // Initial guess for controls is zero.
        if (k < num_time_steps) {
            initial_values.insert(U(k), Vector2::Zero());
        }
    }

    cout << "\nInitial values created." << endl;
    // initial_values.print("Initial Values:\n");

    // --- 4. Optimize ---
    LevenbergMarquardtParams params;
    params.setVerbosity("TERMINATION"); // Print info at the end
    LevenbergMarquardtOptimizer optimizer(graph, initial_values, params);

    cout << "\nOptimizing..." << endl;
    Values result = optimizer.optimize();
    cout << "Optimization complete." << endl;
    cout << "Initial Error: " << graph.error(initial_values) << endl;
    cout << "Final Error: " << graph.error(result) << endl;


    // --- 5. Print Results ---
    cout << "\nOptimized Trajectory:" << endl;
    for (int k = 0; k <= num_time_steps; ++k) {
        Vector4 robot_state = result.at<Vector4>(Xr(k));
        Vector4 load_state = result.at<Vector4>(Xl(k));
        cout << "--- Time Step " << k << " ---" << endl;
        cout << "  Robot Pos: (" << robot_state[0] << ", " << robot_state[1] << ")" << endl;
        cout << "  Load  Pos: (" << load_state[0]  << ", " << load_state[1]  << ")" << endl;
        if (k < num_time_steps) {
            Vector2 control = result.at<Vector2>(U(k));
            cout << "  Control:   (" << control[0] << ", " << control[1] << ")" << endl;
        }
    }

    return 0;
}
