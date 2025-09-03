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

// #include "factor_graph_lib/cable_factors.hpp"
// #include "factor_graph_lib/control_factors.hpp"
// #include "factor_graph_lib/dynamics_factors.hpp"
#include "factor_graph_lib/factor_executor_factory.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>

using namespace std;
using namespace gtsam;
using json = nlohmann::json;
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

const string DOUBLE_PARAMS = "double_params";
const string BOOLEAN_PARAMS = "boolean_params";

const string ONE_ROBOT_NO_ORI_POINTS = "/home/maryia/legacy/experiments/factor_graph_one_drone_one_step/one_drone_no_ori_points.json";
const string ONE_ROBOT_NO_ORI_PARAMS = "/home/maryia/legacy/experiments/factor_graph_one_drone_one_step/one_drone_no_ori_params.json";

const string ONE_ROBOT_WITH_ORI_POINTS = "/home/maryia/legacy/experiments/factor_graph_one_drone_one_step/one_drone_with_ori_points.json";
const string ONE_ROBOT_WITH_ORI_PARAMS = "/home/maryia/legacy/experiments/factor_graph_one_drone_one_step/one_drone_with_ori_params.json";

const string TWO_ROBOTS_NO_ORI_POINTS = "/home/maryia/legacy/experiments/factor_graph_one_drone_one_step/two_drones_no_ori_points.json";
const string TWO_ROBOTS_NO_ORI_PARAMS = "/home/maryia/legacy/experiments/factor_graph_one_drone_one_step/two_drones_no_ori_params.json";

const string TWO_ROBOTS_WITH_ORI_POINTS = "/home/maryia/legacy/experiments/factor_graph_one_drone_one_step/two_drones_with_ori_points.json";
const string TWO_ROBOTS_WITH_ORI_PARAMS = "/home/maryia/legacy/experiments/factor_graph_one_drone_one_step/two_drones_with_ori_params.json";

json read_json_file(string filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open the JSON file: " << filename << std::endl;
        return 1;
    }

    json jsonData;
    try {
        file >> jsonData;
    } catch (json::parse_error& e) {
        std::cerr << "Error parsing JSON: " << e.what() << std::endl;
        return 1;
    }

    return jsonData;
}

json get_initial_params(string mode, string filename, map<string, double>& tune_d, map<string, bool>& tune_b) {
    if (mode != "auto") {
        return {};
    }

    json jsonData = read_json_file(filename);

    for (auto& [key, value] : jsonData[DOUBLE_PARAMS].items()) {
        tune_d[key] = value["lb"].get<double>();
    }

    for (auto& [key, value] : jsonData[BOOLEAN_PARAMS].items()) {
        if (!value.get<bool>()) continue;
        tune_b[key] = false;
    }

    return jsonData;
}

bool get_next_params(const json& js_params, map<string, double>& tune_d, map<string, bool>& tune_b) {
    if (js_params.is_null()) return false;

    string key_to_change = "";
    for (auto& [key, value] : js_params[BOOLEAN_PARAMS].items()) {
        if (!value.get<bool>()) continue;
        if (tune_b[key] != true) {
            key_to_change = key;
        }
    }
    if (key_to_change == "") {
        for (auto& [key, value] : js_params[DOUBLE_PARAMS].items()) {
            if (value["ub"].get<double>() - tune_d[key] > 1e-3) {
                key_to_change = key;
            }
        }
        if (key_to_change  == "") return false;

        bool keys_after_changed = false;
        for (auto& [key, value] : js_params[DOUBLE_PARAMS].items()) {
            if (!keys_after_changed && key != key_to_change) continue;
            if (key == key_to_change) {
                keys_after_changed = true;
                tune_d[key] = min(tune_d[key] * value["step"].get<double>(), value["ub"].get<double>());
            }
            else {
                tune_d[key] = value["lb"].get<double>();
            }
        }

        for (auto& [key, value] : js_params[BOOLEAN_PARAMS].items()) {
            if (!value.get<bool>()) continue;
            tune_b[key] = false;
        }
        return true;
    }

    bool keys_after_changed = false;
    for (auto& [key, value] : js_params[BOOLEAN_PARAMS].items()) {
        if (!value.get<bool>()) continue;
        if (!keys_after_changed && key != key_to_change) { continue; }
        if (key == key_to_change) {
            keys_after_changed = true;
            tune_b[key] = true;
        }
        else {
            tune_b[key] = false;
        }
    }
    return true;
}

struct TuneResult {
    double pos_error;
    map<string, double> factor_errors;
    map<string, double> tune_d;
    map<string, bool> tune_b;
};

bool compareTuneResultsByPosError(const std::vector<TuneResult>& a, const std::vector<TuneResult>& b) {
    double kas = 0.0, kbs = 0.0;
    for (int i = 0; i < a.size(); i++) {
        kas += a[i].pos_error;
        kbs += b[i].pos_error;
    }
    kas /= a.size();
    kbs /= a.size();
    return (kas < kbs);
}

bool compareTuneResultsByLoadDynError(const std::vector<TuneResult>& a, const std::vector<TuneResult>& b) {
    double kal = 0.0, kbl = 0.0;
    for (int i = 0; i < a.size(); i++) {
        kal += a[i].factor_errors.at("[l, x, t, l]"); //"[l, x, t, l]"
        kbl += b[i].factor_errors.at("[l, x, t, l]"); // "[l, x, t, X, T, l]"
    }
    kal /= a.size();
    kbl /= a.size();
    return (kal < kbl);
}

void printTuneResult(const std::vector<std::vector<TuneResult> >& results, int num = 20, bool outfile = false) {

    std::ofstream outputFile("/home/maryia/legacy/experiments/factor_graph_one_drone_one_step/output.txt");
    std::streambuf* originalCoutBuffer = std::cout.rdbuf();
    if (outfile) {
        std::cout.rdbuf(outputFile.rdbuf());
    }
    for (int i = min(num, int(results.size())) - 1; i >= 0; i--) {
        const auto& ts = results[i];
        double sum = 0.0;
        double load_err = 0.0;
        for (const auto& t: ts) {
            sum += t.pos_error;
            load_err += t.factor_errors.at("[l, x, t, l]");
            //cout << t.factor_errors.at("[l, x, t, X, T, l]") << endl;
        }
        for (const auto& [k, v] : ts[0].tune_d) {
            cout << k << ' '<< v << endl;
        }
        for (const auto& [k, v] : ts[0].tune_b) {
            cout << k << ' '<< v << endl;
        }
        cout << endl;
        cout << " Position mean: " << sum / ts.size() << endl;
        cout << " Load error mean: " << load_err / ts.size() << endl;
        cout << "\n=======================\n";
    }

    outputFile.close();
}

void printSeparator() {
    cout << "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n";
    cout << "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n";
    cout << "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n";
    cout << "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n";
    cout << "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n";
}

struct OneRobotNoOriSpec{
    Vector4 initial_load_state;
    Vector4 initial_robot_state;
    Vector4 final_load_goal;
    double robot_height;
    Vector2 last_u;

    OneRobotNoOriSpec(
        const Vector4& initial_load_state,
        const Vector4& initial_robot_state,
        const Vector4& final_load_goal,
        double robot_height,
        const Vector2& last_u
    ) : initial_load_state(initial_load_state),
        initial_robot_state(initial_robot_state),
        final_load_goal(final_load_goal),
        robot_height(robot_height),
        last_u(last_u) {}
};

std::vector<OneRobotNoOriSpec> getOneRobotNoOriSpec() {
    json jsonData = read_json_file(ONE_ROBOT_NO_ORI_POINTS);

    std::vector<OneRobotNoOriSpec> res = {};
    for (const auto& f: jsonData) {
        Vector4 a(0,0,0,0);
        a << f["init_load"][0] ,f["init_load"][1], f["init_load"][2], f["init_load"][3];
        Vector4 b(0,0,0,0);
        b << f["init_robot"][0] ,f["init_robot"][1], f["init_robot"][2], f["init_robot"][3];
        Vector4 c(0,0,0,0);
        c << f["goal_load"][0] ,f["goal_load"][1], f["goal_load"][2], f["goal_load"][3];
        Vector2 d(0,0);
        d << f["last_u"][0] ,f["last_u"][1];
        res.push_back(OneRobotNoOriSpec(
            a, b, c, f["height"].get<double>(), d));
    }
    return res;
}


std::vector<OneRobotNoOriSpec> one_robot_no_ori = {
    // OneRobotNoOriSpec(
    //     Vector4(0.198132, -5.40177e-06, -1.80715e-08, 1.47314e-09),
    //     Vector4(-0.983182, -1.98885e-05, -0.00649756, -3.71512e-06),
    //     Vector4(-0.0376969, 0.000355295, 0, 0),
    //     0.513059,
    //     Vector2(-0.0243281, 0.00018109),
    //     Vector1(0.0016)
    // )
    // ,
    // OneRobotNoOriSpec(
    //     Vector4(-0.290209, 0.000137091, -0.151018, -0.00627848),
    //     Vector4(-1.48005, 0.161958, -0.0941003, 0.0791552),
    //     Vector4(-0.272469, 0.0186467, 0, 0),
    //     0.437046,
    //     Vector2(0.58836, 0.905764)
    // ),
    // OneRobotNoOriSpec(
    //     Vector4(-1.55103, 0.726305, -0.049898, 0.00501587),
    //     Vector4(-2.233, 1.57493, -0.0310213, 0.0113822),
    //     Vector4(-1.59861, 0.798155, 0, 0),
    //     0.493798,
    //     Vector2(-0.234963, -0.0759272)
    // ),
//      Cur load position: 0.200014, -1.38762e-05, 0, 0
// [mpc-6] Cur robot position: -0.971287, -0.000890108, 0.00627227, -3.6729e-05
// [mpc-6] Cur robot height: 0.50252
// [mpc-6] Next load position: -0.0345558, 0.000298548
// [mpc-6] Actual distance: 1.1713
// [mpc-6]    Max distance: 1.18457
// [mpc-6] Initial Error: 1.10046e+09
// [mpc-6] Final Error: 3700.95
// [mpc-6] Next velocity: 0.00605747 -0.000346017
// [mpc-6] Next control: -3.02468e-05 -2.21226e-08
// [mpc-6] Next tension: 1.05189e-07
// [mpc-6] 12 124
OneRobotNoOriSpec(
        Vector4(0.200012, -1.31332e-05, 0, 0),
        Vector4(-0.961359, -0.000971152, 0.0121107, -7.58946e-05),
        Vector4(0.101013, -1.31332e-05, 0, 0),
        0.454193,
        Vector2(0.0, 0.0)
    ),

};

// Cur load position: 0.200012, -1.31332e-05, 0, 0
// [mpc-6] Cur robot position: -0.961359, -0.000971152, 0.0121107, -7.58946e-05
// [mpc-6] Cur robot height: 0.454193
// [mpc-6] Next load position: -0.0691013, 0.0011941
// [mpc-6]   Cable length: 1.187



struct OneRobotWithOriSpec{
    Vector6 initial_load_state;
    Vector4 initial_robot_state;
    Vector6 final_load_goal;
    double robot_height;

    OneRobotWithOriSpec(
        const Vector6& initial_load_state,
        const Vector4& initial_robot_state,
        const Vector6& final_load_goal,
        double robot_height
    ) : initial_load_state(initial_load_state),
        initial_robot_state(initial_robot_state),
        final_load_goal(final_load_goal),
        robot_height(robot_height) {}
};

std::vector<OneRobotWithOriSpec> getOneRobotWithOriSpec() {
    json jsonData = read_json_file(ONE_ROBOT_WITH_ORI_POINTS);

    std::vector<OneRobotWithOriSpec> res = {};
    for (const auto& f: jsonData) {
        Vector6 a(0,0,0,0,0,0);
        a << f["init_load"][0] ,f["init_load"][1], f["init_load"][2], f["init_load"][3], f["init_load"][4], f["init_load"][5];
        Vector4 b(0,0,0,0);
        b << f["init_robot"][0] ,f["init_robot"][1], f["init_robot"][2], f["init_robot"][3];
        Vector6 c(0,0,0,0,0,0);
        c << f["goal_load"][0] ,f["goal_load"][1], f["goal_load"][2], f["goal_load"][3], f["goal_load"][4], f["goal_load"][5];
        res.push_back(OneRobotWithOriSpec(
            a, b, c, f["height"].get<double>()));
    }
    return res;
}

std::vector<OneRobotWithOriSpec> one_robot_with_ori = {
    OneRobotWithOriSpec(
        Vector6(-0.991435, -0.00103294, 0.00195432, 0, 0, 0),
        Vector4(-2.15588, -0.146596, 0.0344486, 0.0219257),
        Vector6(-1.20385, 0.402897, -0.645911, 0, 0, 0),
        0.52494
    ),
    OneRobotWithOriSpec(
        Vector6(0.197901, -1.38854e-05, 3.46041e-05, 0, 0, 0),
        Vector4(-0.470545, 0.0221688, 0.10711, -0.0311483),
        Vector6(-0.203224, 0.0103518, -0.101788, 0, 0, 0),
        0.550406
    ),
    OneRobotWithOriSpec(
        Vector6(0.197901, -1.38854e-05, 3.46041e-05, 0, 0, 0),
        Vector4(-0.470545, 0.0221688, 0.10711, -0.0311483),
        Vector6(-0.203224, 0.0103518, -0.101788, 0, 0, 0),
        0.550406
    )
};


struct TwoRobotsNoOriSpec{
    Vector4 initial_load_state;
    Vector4 initial_robot1_state;
    Vector4 initial_robot2_state;
    Vector4 final_load_goal;
    double robot_height1;
    double robot_height2;

    TwoRobotsNoOriSpec(
        const Vector4& initial_load_state,
        const Vector4& initial_robot1_state,
        const Vector4& initial_robot2_state,
        const Vector4& final_load_goal,
        double robot_height1,
        double robot_height2
    ) : initial_load_state(initial_load_state),
        initial_robot1_state(initial_robot1_state),
        initial_robot2_state(initial_robot2_state),
        final_load_goal(final_load_goal),
        robot_height1(robot_height1),
        robot_height2(robot_height2) {}
};

std::vector<TwoRobotsNoOriSpec> get_two_robots_no_ori_points() {
    json jsonData = read_json_file(TWO_ROBOTS_NO_ORI_POINTS);

    std::vector<TwoRobotsNoOriSpec> res = {};
    for (const auto& f: jsonData) {
        Vector4 a(0,0,0,0);
        a << f["init_load"][0] ,f["init_load"][1], f["init_load"][2], f["init_load"][3];
        Vector4 b(0,0,0,0);
        b << f["init_robot1"][0] ,f["init_robot1"][1], f["init_robot1"][2], f["init_robot1"][3];
        Vector4 bb(0,0,0,0);
        bb << f["init_robot2"][0] ,f["init_robot2"][1], f["init_robot2"][2], f["init_robot2"][3];
        Vector4 c(0,0,0,0);
        c << f["goal_load"][0] ,f["goal_load"][1], f["goal_load"][2], f["goal_load"][3];
        
        res.push_back(TwoRobotsNoOriSpec(
            a, b, bb, c, f["height1"].get<double>(), f["height2"].get<double>()));
    }
    return res;
}

std::vector<TwoRobotsNoOriSpec> two_robots_no_ori = {
    TwoRobotsNoOriSpec(
        Vector4(0.1521526983365722,
            0.044316165224553915,
            0.0,
            0.0),
        Vector4( -0.8686248817783563,
            0.00723303249848686,
            0.012991947392871961,
            -0.005504996218551037),
        Vector4(0.1921693692811136,
            0.9775868734025891,
            -0.033460592896584505,
            -0.08442394079106005),
        Vector4(-0.2475493389058867,0.015379299511551814,0.0,0.0),
        0.5188737100435936,
        0.5387850304077783
    ),
//     Cur load position: -0.392348, 0.158378, 0, 0
// [mpc_two_drones-7] Cur robot 1 position: -1.3874, -0.092516, -0.00178325, 0.00109391
// [mpc_two_drones-7] Cur robot 2 position: -0.191496, 0.61114, -0.0635253, -0.0285721
// [mpc_two_drones-7] Robots height: 0.584132 0.609334
// [mpc_two_drones-7] Next load position: -1.32027, 0.497702
// [mpc_two_drones-7] Cable lengths: 1.04835 1.03043
// [mpc_two_drones-7]   Actual dist: 1.0262 0.495313
// [mpc_two_drones-7] Optimization complete.
// [mpc_two_drones-7] Initial Error: 1.95252e+08
// [mpc_two_drones-7] Final Error: 30287.5
// [mpc_two_drones-7] Cur tension: 0.000159172 0.000133441
// [mpc_two_drones-7] Cur controls: 0.0235581 -0.0186711 -0.0642181 -0.109227
// [mpc_two_drones-7] Next controls: 0.0237487 -0.0185392 -0.0479587 -0.0866343
// [mpc_two_drones-7] Next velocity drone 1: 0.00295937 -0.00263239
// [mpc_two_drones-7] Next velocity drone 2: -0.0764071 -0.0504446
TwoRobotsNoOriSpec(
        Vector4(-0.392348, 0.158378, 0, 0),
        Vector4( -1.3874, -0.092516, -0.00178325, 0.00109391),
        Vector4(-0.191496, 0.61114, -0.0635253, -0.0285721),
        Vector4(-1.32027, 0.497702, 0, 0),
        0.584132, 0.609334
    )
};


struct TwoRobotsWithOriSpec{
    Vector6 initial_load_state;
    Vector4 initial_robot1_state;
    Vector4 initial_robot2_state;
    Vector6 final_load_goal;
    double robot_height1;
    double robot_height2;

    TwoRobotsWithOriSpec(
        const Vector6& initial_load_state,
        const Vector4& initial_robot1_state,
        const Vector4& initial_robot2_state,
        const Vector6& final_load_goal,
        double robot_height1,
        double robot_height2
    ) : initial_load_state(initial_load_state),
        initial_robot1_state(initial_robot1_state),
        initial_robot2_state(initial_robot2_state),
        final_load_goal(final_load_goal),
        robot_height1(robot_height1),
        robot_height2(robot_height2) {}
};

std::vector<TwoRobotsWithOriSpec> get_two_robots_with_ori_points() {
    json jsonData = read_json_file(TWO_ROBOTS_WITH_ORI_POINTS);

    std::vector<TwoRobotsWithOriSpec> res = {};
    for (const auto& f: jsonData) {
        Vector6 a(0,0,0,0,0,0);
        a << f["init_load"][0] ,f["init_load"][1], f["init_load"][2], f["init_load"][3], f["init_load"][4], f["init_load"][5];
        Vector4 b(0,0,0,0);
        b << f["init_robot1"][0] ,f["init_robot1"][1], f["init_robot1"][2], f["init_robot1"][3];
        Vector4 bb(0,0,0,0);
        bb << f["init_robot2"][0] ,f["init_robot2"][1], f["init_robot2"][2], f["init_robot2"][3];
        Vector6 c(0,0,0,0,0,0);
        c << f["goal_load"][0] ,f["goal_load"][1], f["goal_load"][2], f["goal_load"][3], f["goal_load"][4], f["goal_load"][5];
        
        res.push_back(TwoRobotsWithOriSpec(
            a, b, bb, c, f["height1"].get<double>(), f["height2"].get<double>()));
    }
    return res;
}

std::vector<TwoRobotsWithOriSpec> two_robots_with_ori = {
    TwoRobotsWithOriSpec(
    //         Cur load position: -0.528534, 0.113513, 0.0139348, -1.8198e-05, -1.82078e-05, 9.09654e-05
    // [mpc_two_drones_with_orientation-7] Cur robot 1 position: -1.35897, -0.572823, 0.00941458, -0.0887321
    // [mpc_two_drones_with_orientation-7] Cur robot 2 position: -0.017592, 0.903962, -0.0119463, 0.0222254
    // [mpc_two_drones_with_orientation-7] Robots height: 0.443523 0.60519
    // [mpc_two_drones_with_orientation-7] Next load position: -0.774066, 0.205868, -0.397411
    // [mpc_two_drones_with_orientation-7] Cable lengths: 0.929617 0.833454
    // [mpc_two_drones_with_orientation-7]   Actual dist: 0.9299 0.782668
    // [mpc_two_drones_with_orientation-7] Optimization complete.
    // [mpc_two_drones_with_orientation-7] Initial Error: 5.07077e+07
    // [mpc_two_drones_with_orientation-7] Final Error: 2704.54
    // [mpc_two_drones_with_orientation-7] Next controls: -0.359776 -0.174827 -0.18933 0.0981818
    // [mpc_two_drones_with_orientation-7] Next velocity drone 1: -0.0241124 -0.15099
    // [mpc_two_drones_with_orientation-7] Next velocity drone 2: 0.0213395 0.0950121 ??????????????????
        Vector6(-0.528534, 0.113513, 0.0139348, -1.8198e-05, -1.82078e-05, 9.09654e-05),
        Vector4(-1.35897, -0.572823, 0.00941458, -0.0887321),
        Vector4(-0.017592, 0.903962, -0.0119463, 0.0222254),
        Vector6(-0.774066, 0.205868, -0.397411, 0, 0, 0),
        0.443523,
        0.60519
    ),
    //     Cur load position: -0.862215, 0.148695, -0.0640435, -0.0130398, 0.0123373, -0.0703703
    // [mpc_two_drones_with_orientation-7] Cur robot 1 position: -1.87853, -0.256012, -0.0208214, 0.00274339
    // [mpc_two_drones_with_orientation-7] Cur robot 2 position: -0.175577, 0.924015, 0.0112545, 0.0469924
    // [mpc_two_drones_with_orientation-7] Robots height: 0.469234 0.523021
    // [mpc_two_drones_with_orientation-7] Next load position: -1.06368, 0.356313, -0.560774
    // [mpc_two_drones_with_orientation-7] Cable lengths: 0.916907 0.887327
    // [mpc_two_drones_with_orientation-7]   Actual dist: 0.917248 0.886297
    // [mpc_two_drones_with_orientation-7] Optimization complete.
    // [mpc_two_drones_with_orientation-7] Initial Error: 6.9196e+07
    // [mpc_two_drones_with_orientation-7] Final Error: 3008.53
    // [mpc_two_drones_with_orientation-7] Next controls: -0.379089 0.131928 0.222225 0.332605
    // [mpc_two_drones_with_orientation-7] Next velocity drone 1: -0.0916177 0.0173701
    // [mpc_two_drones_with_orientation-7] Next velocity drone 2: 0.0598472 0.105672 ???

    TwoRobotsWithOriSpec(
        Vector6(-0.862215, 0.148695, -0.0640435, -0.0130398, 0.0123373, -0.0703703),
        Vector4(-1.87853, -0.256012, -0.0208214, 0.00274339),
        Vector4(-0.175577, 0.924015, 0.0112545, 0.0469924),
        Vector6(-1.06368, 0.356313, -0.560774, 0, 0, 0), 
        0.469234, 
        0.523021
    ),
    //     Cur load position: -0.534556, 0.121316, -0.21805, -0.0335872, -0.00675096, -0.0018817
    // [mpc_two_drones_with_orientation-7] Cur robot 1 position: -1.66291, 0.0633607, -0.0302595, -0.00968158
    // [mpc_two_drones_with_orientation-7] Cur robot 2 position: 0.110523, 0.941871, -0.000551268, -0.00977029
    // [mpc_two_drones_with_orientation-7] Robots height: 0.42577 0.554698
    // [mpc_two_drones_with_orientation-7] Next load position: -0.597082, 0.141206, -0.303164
    // [mpc_two_drones_with_orientation-7] Cable lengths: 0.93788 0.867876
    // [mpc_two_drones_with_orientation-7]   Actual dist: 0.938565 0.867852
    // [mpc_two_drones_with_orientation-7] Optimization complete.
    // [mpc_two_drones_with_orientation-7] Initial Error: 3.99047e+06
    // [mpc_two_drones_with_orientation-7] Final Error: 3.99047e+06
    // [mpc_two_drones_with_orientation-7] Next controls: 0 0 0 0
    // [mpc_two_drones_with_orientation-7] Next velocity drone 1: -0.0302595 -0.00968158
    // [mpc_two_drones_with_orientation-7] Next velocity drone 2: -0.000551268 -0.00977029

    TwoRobotsWithOriSpec(
        Vector6(-0.534556, 0.121316, -0.21805, -0.0335872, -0.00675096, -0.0018817),
        Vector4(-1.66291, 0.0633607, -0.0302595, -0.00968158),
        Vector4(0.110523, 0.941871, -0.000551268, -0.00977029),
        Vector6(-0.597082, 0.141206, -0.303164, 0, 0, 0),
        0.42577, 
        0.554698
    )
    // TwoRobotsWithOriSpec(
    //     Vector6(0.150256, 0.0502197, 0.00282714, 0, 0, 0),
    //     Vector4(-0.949938, 0.00168648, -0.00929439, 0.00128902),
    //     Vector4(0.201326, 1.14887, -0.00175323, -0.0184762),
    //     Vector6(-1.22465e-16, 0, 0, 0, 0, 0),
    //     0.500701,
    //     0.50312
    // )
};


struct OneRobotWithHeightSpec{
    Vector4 initial_load_state;
    Vector6 initial_robot_state;
    Vector4 final_load_goal;
    double robot_height;
    Vector3 last_u;

    OneRobotWithHeightSpec(
        const Vector4& initial_load_state,
        const Vector6& initial_robot_state,
        const Vector4& final_load_goal,
        double robot_height,
        const Vector3& last_u
    ) : initial_load_state(initial_load_state),
        initial_robot_state(initial_robot_state),
        final_load_goal(final_load_goal),
        robot_height(robot_height),
        last_u(last_u) {}
};


std::vector<OneRobotWithHeightSpec> one_robot_with_height = {
    OneRobotWithHeightSpec(

//  Next control: 0.491335 0.111334 0.485999
// [mpc_with_height-6] Next tension: 0.364558
// [mpc_with_height-6] 890 994
// [mpc_with_height-6] Cur load position: -1.03071, 1.63742, -5.59785e-08, 0.0559649
// [mpc_with_height-6] Cur robot position: -0.979516, 2.82353, 0.475023, 0.0691242, 0.0306609, 0.0709556
// [mpc_with_height-6] Cur robot height: 0.475023
// [mpc_with_height-6] Next load position: -0.978197, 1.78
// [mpc_with_height-6] Actual distance: 1.20368
// [mpc_with_height-6] Initial Error: 6.09917e+06
// [mpc_with_height-6] Final Error: 54.8858

  
        Vector4(-1.03071, 1.63742, -5.59785e-08, 0.0559649),
        Vector6(-0.979516, 2.82353, 0.475023, 0.0691242, 0.0306609, 0.0709556),
        Vector4(-0.978197, 1.78, 0, 0),
        0.475023,
        Vector3(0.491335, 0.111334, 0.485999)
    ),
    OneRobotWithHeightSpec(

// Next control: 0.106894 0.662059 -0.2007
// [mpc_with_height-6] Next tension: 0.137658
// [mpc_with_height-6] 899 1005
// [mpc_with_height-6] Cur load position: -1.03217, 1.65812, 0, 0
// [mpc_with_height-6] Cur robot position: -0.879594, 2.79977, 0.576288, 0.0828811, -0.0301903, 0.0379491
// [mpc_with_height-6] Cur robot height: 0.576288
// [mpc_with_height-6] Next load position: -0.9743, 1.798
// [mpc_with_height-6] Actual distance: 1.19792
// [mpc_with_height-6] Initial Error: 5.57364e+06
// [mpc_with_height-6] Final Error: 2024.11

        Vector4(-1.03217, 1.65812, 0, 0),
        Vector6(-0.879594, 2.79977, 0.576288, 0.0828811, -0.0301903, 0.0379491),
        Vector4( -0.9743, 1.798, 0, 0),
        0.576288,
        Vector3(0.106894, 0.662059, -0.2007)
    )
};

int main(int argc, char** argv) {

    bool WITH_ANGLE = false;
    bool TWO_ROBOTS = false;
    bool WITH_HEIGHT = false;
    bool THREE_ROBOTS = true;
    string MODE = "one_of"; // "auto" or "one_of"
    int K = 0;

    map<string, double> tune_d = {};
    map<string, bool> tune_b = {};
    map<string, double> factor_errors = {};
    double pos_error = 0.0;

    std::vector<std::vector<TuneResult> > results;

    if (!WITH_ANGLE && !TWO_ROBOTS && !WITH_HEIGHT && !THREE_ROBOTS) {

        json tuneData = get_initial_params(MODE, ONE_ROBOT_NO_ORI_PARAMS, tune_d, tune_b);
        auto points = getOneRobotNoOriSpec();
        cout << "Num of points: " << points.size() << endl;

        while (true) {

            std::vector<TuneResult> results_per_params = {};
            
            for (const auto& s : points) {
                Vector1 tt(0.0);
                auto executor = FactorExecutorFactory::create(MODE, s.initial_load_state, s.initial_robot_state, s.final_load_goal, s.robot_height, s.last_u, tt, tune_d, tune_b);
                auto result = executor->run(factor_errors, pos_error);

                results_per_params.push_back({pos_error, factor_errors, tune_d, tune_b});

                K++;
                if (K % 1000 == 0) {
                    cout << K << endl;
                    std::sort(results.begin(), results.end(), compareTuneResultsByLoadDynError);
                    cout << "sorted: " << results.size() << endl;
                    printTuneResult(results, 50);
                }
            }
            results.push_back(results_per_params);
            if (!get_next_params(tuneData, tune_d, tune_b)) break;
        }

    } else if (WITH_ANGLE && !TWO_ROBOTS && !WITH_HEIGHT && !THREE_ROBOTS){

        json tuneData = get_initial_params(MODE, ONE_ROBOT_WITH_ORI_PARAMS, tune_d, tune_b);
        auto points = getOneRobotWithOriSpec();
        cout << "Num of points: " << points.size() << endl;

        while (true) {

            std::vector<TuneResult> results_per_params = {};
            
            for (const auto& s : points) {
                auto executor = FactorExecutorFactory::create(MODE, s.initial_load_state, s.initial_robot_state, s.final_load_goal, s.robot_height, tune_d, tune_b);
                auto result = executor->run(factor_errors, pos_error);

                results_per_params.push_back({pos_error, factor_errors, tune_d, tune_b});

                K++;
                if (K % 1000 == 0) {
                    cout << K << endl;
                    std::sort(results.begin(), results.end(), compareTuneResultsByPosError);
                    cout << "sorted: " << results.size() << endl;
                    printTuneResult(results, 1000);
                }
            }
            results.push_back(results_per_params);
            if (!get_next_params(tuneData, tune_d, tune_b)) break;
        }

    } else if (TWO_ROBOTS && !WITH_ANGLE && !WITH_HEIGHT && !THREE_ROBOTS) {

        json tuneData = get_initial_params(MODE, TWO_ROBOTS_NO_ORI_PARAMS, tune_d, tune_b);
        auto points = get_two_robots_no_ori_points();
        cout << "Num of points: " << points.size() << endl;

        while (true) {

            std::vector<TuneResult> results_per_params = {};
            
            for (const auto& s : points) {
                Vector2 last_u(0.0,0.0);
                auto executor = FactorExecutorFactory::create(
                    MODE, 
                    s.initial_load_state, 
                    s.initial_robot1_state, 
                    s.initial_robot2_state, 
                    s.final_load_goal, 
                    s.robot_height1, 
                    s.robot_height2, 
                    last_u,
                    last_u,
                    tune_d, 
                    tune_b
                );
                auto result = executor->run(factor_errors, pos_error);
                results_per_params.push_back({pos_error, factor_errors, tune_d, tune_b});

                K++;
                if (K % 1000 == 0) {
                    cout << K << endl;
                    std::sort(results.begin(), results.end(), compareTuneResultsByLoadDynError);
                    printTuneResult(results, 1000);
                }
            }
            results.push_back(results_per_params);
            if (!get_next_params(tuneData, tune_d, tune_b)) break;
        }

    } else if (TWO_ROBOTS && WITH_ANGLE && !WITH_HEIGHT && !THREE_ROBOTS) {

        json tuneData = get_initial_params(MODE, TWO_ROBOTS_WITH_ORI_PARAMS, tune_d, tune_b);
        auto points = get_two_robots_with_ori_points();
        cout << "Num of points: " << points.size() << endl;

        while (true) {

            std::vector<TuneResult> results_per_params = {};
            
            for (const auto& s : points) {
                auto executor = FactorExecutorFactory::create(
                    MODE, 
                    s.initial_load_state, 
                    s.initial_robot1_state, 
                    s.initial_robot2_state, 
                    s.final_load_goal, 
                    s.robot_height1, 
                    s.robot_height2, tune_d, tune_b);
                auto result = executor->run(factor_errors, pos_error);
                results_per_params.push_back({pos_error, factor_errors, tune_d, tune_b});

                if (MODE == "one_of") return 0;

                K++;
                if (K % 1000 == 0) {
                    cout << K << endl;
                    std::sort(results.begin(), results.end(), compareTuneResultsByPosError);
                    printTuneResult(results, 1000);
                }
            }
            results.push_back(results_per_params);
            if (!get_next_params(tuneData, tune_d, tune_b)) break;
        }
    } else if (WITH_HEIGHT && !THREE_ROBOTS) {
        
        json tuneData = get_initial_params(MODE, "", tune_d, tune_b);
        auto points = one_robot_with_height;
        cout << "Num of points: " << points.size() << endl;

        while (true) {

            std::vector<TuneResult> results_per_params = {};
            
            for (const auto& s : points) {
                auto executor = FactorExecutorFactory::create(
                    MODE, 
                    s.initial_load_state, 
                    s.initial_robot_state,
                    s.final_load_goal, 
                    s.robot_height, 
                    s.last_u, Vector1::Zero(),
                    tune_d, tune_b);
                auto result = executor->run(factor_errors, pos_error);
                results_per_params.push_back({pos_error, factor_errors, tune_d, tune_b});

                //if (MODE == "one_of") return 0;

                K++;
                if (K % 1000 == 0) {
                    cout << K << endl;
                    std::sort(results.begin(), results.end(), compareTuneResultsByPosError);
                    printTuneResult(results, 1000);
                }
            }
            results.push_back(results_per_params);
            if (!get_next_params(tuneData, tune_d, tune_b)) break;
        }
    } else {
//         Cur load position: 0.200014, -1.38803e-05, 3.47131e-05, 0, 0, 0
// [mpc_multi_drones_with_orientation-8] Cur robot 1 position: -0.971478, -0.000885069, 0.00380496, -0.000155451
// [mpc_multi_drones_with_orientation-8] Height: 0.502425
// [mpc_multi_drones_with_orientation-8] Cur robot 2 position: -0.68447, 0.684448, 0.000665602, -0.000748444
// [mpc_multi_drones_with_orientation-8] Height: 0.489528
// [mpc_multi_drones_with_orientation-8] Cur robot 3 position: -0.686952, -0.685664, 0.00151768, 0.00165828
// [mpc_multi_drones_with_orientation-8] Height: 0.488682
// [mpc_multi_drones_with_orientation-8] Next load position: -0.0314146, 0.000246735, -0.015708

        Vector6 initial_load_state(0.200014, -1.38803e-05, 3.47131e-05, 0, 0, 0);
        Vector4 r1(-0.971478, -0.000885069, 0.00380496, -0.000155451);
        Vector4 r2(-0.68447, 0.684448, 0.000665602, -0.000748444);
        Vector4 r3(-0.686952, -0.685664, 0.00151768, 0.00165828);
        Vector6 lg(-0.0314146, 0.000246735, -0.015708, 0, 0, 0);

        auto executor = FactorExecutorFactory::create(
            MODE, 3,
            initial_load_state, 
            {r1, r2, r3}, lg, {0.502425, 0.489528, 0.488682},
            tune_d, tune_b);
        auto result = executor->run(factor_errors, pos_error);
    }

    printSeparator();
    cout << "Total runs: " << K << endl;
 
    std::sort(results.begin(), results.end(), compareTuneResultsByLoadDynError);
    printTuneResult(results, 100000, true);

    return 0;
}
