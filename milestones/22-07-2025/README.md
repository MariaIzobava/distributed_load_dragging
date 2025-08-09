# 2 robots 0.01 kg

* POSITIVE GRAVITY
* Load mass in the code is 0.001 but in the model 0.01 !!!

# 1 robot with 0.001 kg

Works fine but I saw once the optimisation failed so probably not that stable.

const int num_time_steps = 20;
const double dt = 0.005;
const double robot_mass = 0.025; // kg
const double load_mass = 0.001;   // kg
const double gravity = 9.81;
const double mu = 0.3;
const double cable_length = 1.02; <-- smaller than the actual distance between the load center and the drone!
const double u_upper_bound = 0.4;
const double u_lower_bound = 0.003;
double weight_tension_lower_bound = 1000000.0; 
//double weight_cable_stretch = 1000000.0;    <-- was not used!
double weight_tension_slack = 50.0;

auto dynamics_cost = noiseModel::Diagonal::Sigmas(
    (Vector(4) << 0.001, 0.001, 0.001, 0.001).finished());
auto goal_cost = noiseModel::Diagonal::Sigmas(
    (Vector(4) << 0.00005, 0.00005, 1000.1, 1000.1).finished());
auto init_cost = noiseModel::Diagonal::Sigmas(
    (Vector(4) << 0.000005, 0.000005, 0.000005, 0.000005).finished());
auto tension_cost = noiseModel::Isotropic::Sigma(1, 1e-3);
auto control_cost = noiseModel::Isotropic::Sigma(1, 1e-3);


* trajectory points: 5000
* the load with attachement at the bottom
* only works so well when hte cable distance is wrong!

Same configuration but with cable length close to the correct one in in 0_001kg_no_orientation_correct_cable_length.png

