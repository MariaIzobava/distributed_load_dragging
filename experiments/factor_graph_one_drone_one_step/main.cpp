/* ----------------------------------------------------------------------------
 * A C++ example for GTSAM that implements the factor graph for a robot
 * controlling a load with a cable.
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

// // My Factor Graph classes
#include "factor_graph_lib/cable_factors.hpp"
#include "factor_graph_lib/control_factors.hpp"
#include "factor_graph_lib/dynamics_factors.hpp"
#include "factor_graph_lib/factor_executor.hpp"

using namespace std;
using namespace gtsam;

using symbol_t = gtsam::Symbol;

void FindAllValues(const Vector4& robot_state, const Vector6& load_state, const Vector6& load_goal) {
    int num = 20;
    double dt = 0.005;
    double load_mass = 0.005;
    double mu = 0.01;
    double g = 9.81;
    Vector6 diff = (load_goal - load_state) / num;
    cout << diff << std::endl;
    Vector2 prev_load_pos(2), prev_load_speed(2);
    prev_load_pos << load_state(0), load_state(1);
    prev_load_speed << load_state(3), load_state(4);
    for (int k = 1; k <= num; k++) {
        Vector2 cur_load_pos(2);
        cur_load_pos << load_state(0) + k * diff(0),  load_state(1) + k * diff(1);
        Vector2 cur_speed = (cur_load_pos - prev_load_pos) /dt;
        Vector2 cur_a = (cur_speed - prev_load_speed) / dt;

        double norm = prev_load_speed.norm();
        Vector2 normed_prev_vel = prev_load_speed / norm;

        cout << "normed_prev_vel " << normed_prev_vel << std::endl;
        Vector2 cur_te(2); 
        cur_te << cur_a(0) * load_mass + mu * g * load_mass * normed_prev_vel(0), cur_a(1) * load_mass + mu * g * load_mass * normed_prev_vel(1);
        double t = cur_te.norm();


        Vector2 e = cur_te / t;

        cout << "T: " << t << "; E: " << e(0) << ' ' << e(1) << endl;
        
        prev_load_pos = cur_load_pos;
        prev_load_speed = cur_speed;

    }

    prev_load_pos << load_state(0), load_state(1);
    prev_load_speed << load_state(3), load_state(4);

    Vector2 cur_load_pos(2);
    cur_load_pos << load_state(0) + 20 * diff(0),  load_state(1) + 20 * diff(1);
    
    Vector2 cur_speed = (cur_load_pos - prev_load_pos) /dt;
    Vector2 cur_a = (cur_speed - prev_load_speed) / dt;
    cout << "cur load pos " << cur_a << std::endl;

    double norm = prev_load_speed.norm();
    Vector2 normed_prev_vel(2);

    normed_prev_vel << -0.992497, 0.122265;
    Vector2 cur_te(2); 
    cur_te << cur_a(0) * load_mass + mu * g * load_mass * normed_prev_vel(0), cur_a(1) * load_mass + mu * g * load_mass * normed_prev_vel(1);
    double t = cur_te.norm();


    Vector2 e = cur_te / t;

    cout << "T: " << t << "; E: " << e(0) << ' ' << e(1) << endl;

}


int main(int argc, char** argv) {

    bool WITH_ANGLE = false;
    bool TWO_ROBOTS = false;

    if (!WITH_ANGLE && !TWO_ROBOTS) {
        
        Vector4 initial_load_state(-0.721754, 0.122561, 0, 0);
        Vector4 initial_robot_state(-1.61632, 0.494984, -0.000491759, 0.000247113);
        Vector4 final_load_goal(-1.00181, 0.268997, 0, 0);
        double robot_height = 0.503174;
        Vector2 last_u(0.000870843, 0.00589633);
        Vector1 last_t(0.000194949);

        auto executor = FactorExecutorFactory::create(1, initial_load_state, initial_robot_state, final_load_goal, robot_height, last_u, last_t);
        auto reult = executor->run();

    } else if (WITH_ANGLE && !TWO_ROBOTS){

        Vector6 initial_load_state(0.149135, 0.0409068, -0.0269265, 0, 0, 0);
        Vector4 initial_robot_state(-0.920369, 0.00136737, -0.00229409, 0.000568685);
        Vector6 final_load_goal(-0.0219907, 0.000120901, -0.0109956, 0, 0, 0);
        double robot_height = 0.493962;

        auto executor = FactorExecutorFactory::create(1, initial_load_state, initial_robot_state, final_load_goal, robot_height);
        auto reult = executor->run();

    } else if (TWO_ROBOTS && !WITH_ANGLE) {

        Vector4 initial_load_state(0.147999, 0.377569, 0.0132869, 0.0711125);
        Vector4 initial_robot1_state( -1.08944, -0.172567, 0.00471183, -0.00134805);
        Vector4 initial_robot2_state(-0.646896, 1.40409, -0.0689179, 0.0239214);
        Vector4 final_load_goal(-1, 0.267949, 0, 0);
        double robot_height1 =  0.446799;
        double robot_height2 =  0.397307;

        auto executor = FactorExecutorFactory::create(1, initial_load_state, initial_robot1_state, initial_robot2_state, final_load_goal, robot_height1, robot_height2);
        auto reult = executor->run();

    } else {

        Vector6 initial_load_state(0.150256, 0.0502197, 0.00282714, 0, 0, 0);
        Vector4 initial_robot1_state(-0.949938, 0.00168648, -0.00929439, 0.00128902);
        Vector4 initial_robot2_state(0.201326, 1.14887, -0.00175323, -0.0184762);
        Vector6 final_load_goal(-1.22465e-16, 0, 0, 0, 0, 0);
        double robot_height1 = 0.500701;
        double robot_height2 =0.50312;

        auto executor = FactorExecutorFactory::create(1, initial_load_state, initial_robot1_state, initial_robot2_state, final_load_goal, robot_height1, robot_height2);
        auto reult = executor->run();
    }

    return 0;
}
