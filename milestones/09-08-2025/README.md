# 0.005 kg, one robot, no ori

const double u_upper_bound = 0.8;
const double u_lower_bound = 0.003;
double weight_tension_lower_bound = 1000000.0;
double weight_cable_stretch = 100.0;
double weight_tension_slack = 50.0;
double weight_tether_tension = 21.0;
const double cable_length = 0.20 + sqrt(1.03 * 1.03 - (robot_height_ - 0.2) * (robot_height_ - 0.2));
double cable_length_offset = 0.007;

bool have_u0_prior = false;
bool have_uk_prior = false;
bool have_t0_prior = false;
bool have_tension_slack_penalty_factor = true;
bool have_tether_tension_factor = false;
bool have_trajectory_reference_factor = false;

// STATE PRIORS
auto init_cost = noiseModel::Diagonal::Sigmas(
    (Vector(4) << 0.000005, 0.000005, 0.000005, 0.000005).finished());
auto goal_interim_cost = noiseModel::Diagonal::Sigmas(
    (Vector(4) << 0.01,    0.01,    1000.1, 1000.1).finished());
auto goal_cost = noiseModel::Diagonal::Sigmas(
    (Vector(4) << 0.00005, 0.00005, 1000.1, 1000.1).finished());

// DYNAMICS
auto dynamics_robot_cost = noiseModel::Diagonal::Sigmas(
    (Vector(4) << 0.001, 0.001, 0.001, 0.001).finished());
auto dynamics_load_cost = noiseModel::Diagonal::Sigmas(
    (Vector(4) << 0.001, 0.001, 0.001, 0.001).finished());

// CONTROL
auto control_init_cost = noiseModel::Diagonal::Sigmas(
    (Vector(2) << 10, 10).finished());
auto control_cost = noiseModel::Isotropic::Sigma(1, 1e-1);
auto control_interim_cost = noiseModel::Diagonal::Sigmas(
    (Vector(2) << 10, 10).finished());

// TENSION
auto tension_prior_cost = noiseModel::Isotropic::Sigma(1, 1e-2);
auto tension_cost = noiseModel::Isotropic::Sigma(1, 1e-3);

# 0.005 kg, one robot, with ori

const double u_upper_bound = 0.4;
const double u_lower_bound = 0.003;
double weight_tension_lower_bound = 1000000.0;
double weight_cable_stretch = 100.0;
double weight_tension_slack = 50.0;
double weight_tether_tension = 21.0;
const double cable_length = sqrt(1.03 * 1.03 - (robot_height_ - 0.2) * (robot_height_ - 0.2));

bool have_tension_slack_penalty_factor = true;
bool have_tether_tension_factor = false;
bool have_trajectory_reference_factor = true;

// STATE PRIORS
auto init_cost = noiseModel::Diagonal::Sigmas(
(Vector(4) << 0.000005, 0.000005, 0.000005, 0.000005).finished());
auto init_with_angle_cost = noiseModel::Diagonal::Sigmas(
(Vector(6) << 0.000005, 0.000005, 0.000005, 0.000005, 0.000005, 0.000005).finished());
// 0.00005 the best
auto goal_with_angle_cost = noiseModel::Diagonal::Sigmas(
(Vector(6) << 0.000005, 0.000005, 0.00005, 1000.1, 1000.1, 1000.1).finished());
auto interim_goal_with_angle_cost = noiseModel::Diagonal::Sigmas(
(Vector(6) << 0.000005, 0.000005, 0.00005,    1000.1, 1000.1, 1000.1).finished());

// DYNAMICS
auto dynamics_cost = noiseModel::Diagonal::Sigmas(
(Vector(4) << 0.001, 0.001, 0.001, 0.001).finished());
auto dynamics_with_angle_cost = noiseModel::Diagonal::Sigmas(
(Vector(6) << 0.001, 0.001, 0.01, 0.001, 0.001, 0.01).finished());

// CONTROL
auto control_cost = noiseModel::Isotropic::Sigma(1, 1e-2);

// TENSION
auto tension_cost = noiseModel::Isotropic::Sigma(1, 1e-3);

# 0.01 kg, two robots, no ori


const double u_upper_bound = 0.4; 
const double u_lower_bound = 0.03;
double weight_tension_lower_bound = 1000000.0;
double weight_cable_stretch = 100.0;
double weight_tension_slack = 50.0;
const double cable_length1 = 1.02; //0.20 + sqrt(1.03 * 1.03 - (robot_height1_) * (robot_height1_));
const double cable_length2 = 1.02; //0.20 + sqrt(1.03 * 1.03 - (robot_height2_) * (robot_height2_));
int cable_stretch_penalty_offset = 0; //0.002;
int cable_tension_penalty_offset = 0; //0.007;

bool have_trajectory_reference_factor = true;

// STATE PRIORS
auto init_cost = noiseModel::Diagonal::Sigmas(
    (Vector(4) << 0.000005, 0.000005, 0.000005, 0.000005).finished());
auto goal_cost = noiseModel::Diagonal::Sigmas(
    (Vector(4) << 0.00005, 0.00005, 1000.1, 1000.1).finished());

// DYNAMICS
auto dynamics_cost = noiseModel::Diagonal::Sigmas(
    (Vector(4) << 0.001, 0.001, 0.001, 0.001).finished());

// CONTROL
auto control_cost = noiseModel::Isotropic::Sigma(1, 1e-2);

// TENSION
auto tension_cost = noiseModel::Isotropic::Sigma(1, 1e-3);

Issue:

* cable lengths are fixed to 1.02
* no CableStretchPenaltyFactor 

# 0.01 kg, two robots, with ori
