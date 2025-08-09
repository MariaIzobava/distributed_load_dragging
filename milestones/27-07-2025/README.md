# 1 robot no ori, 0.005 kg

const int num_time_steps = 20;
const double dt = 0.005;
const double robot_mass = 0.025; // kg
const double load_mass = 0.005;   // kg
const double gravity = 9.81;
const double mu = 0.3;
const double u_upper_bound = 0.5;
const double u_lower_bound = 0.001;
double weight_tension_lower_bound = 1000000.0; <-- only worked with this high value!
// double weight_cable_stretch = 10.0;  <-- wasn't used
double weight_tension_slack = 500.0;

const double cable_length = 0.20 + sqrt(1.03 * 1.03 - (robot_height_ - 0.2) * (robot_height_ - 0.2));

auto dynamics_cost = noiseModel::Diagonal::Sigmas(
    (Vector(4) << 0.001, 0.001, 0.001, 0.001).finished());
auto goal_cost = noiseModel::Diagonal::Sigmas(
    (Vector(4) << 0.000005, 0.000005, 1000.1, 1000.1).finished());
auto init_cost = noiseModel::Diagonal::Sigmas(
    (Vector(4) << 0.000005, 0.000005, 0.000005, 0.000005).finished());
auto tension_cost = noiseModel::Isotropic::Sigma(1, 1e-3);
auto control_cost = noiseModel::Isotropic::Sigma(1, 1e-3);

* slack penalty -0.007 from cable length
* only final load trajectory point


# 1

const int num_time_steps = 20;
const double dt = 0.005;
const double robot_mass = 0.025; // kg
const double gravity = 9.81;

const double load_mass = 0.005;   // kg
const double inertia = 0.000133;
const double mu = 0.3;
const double mu2 = 0.3;
const double u_upper_bound = 0.8;
const double u_lower_bound = 0.1;
double weight_tension_lower_bound = 100.0; // High weight to strongly enforce T >= 0
double weight_cable_stretch = 100.0;     // Very high weight to strongly prevent cable from over-stretching
double weight_tension_slack = 50.0;

// State-dependent parameters
const double cable_length = sqrt(1.03 * 1.03 - (robot_height_ - 0.2) * (robot_height_ - 0.2));

// --- Define Noise Models ---
// These represent the uncertainty of each factor (1/covariance)
auto dynamics_cost = noiseModel::Diagonal::Sigmas(
(Vector(4) << 0.001, 0.001, 0.001, 0.001).finished());
auto dynamics_with_angle_cost = noiseModel::Diagonal::Sigmas(
(Vector(6) << 0.001, 0.001, 0.001, 0.001, 0.001, 0.001).finished());

auto init_cost = noiseModel::Diagonal::Sigmas(
(Vector(4) << 0.000005, 0.000005, 0.000005, 0.000005).finished());
auto init_with_angle_cost = noiseModel::Diagonal::Sigmas(
(Vector(6) << 0.000005, 0.000005, 0.000005, 0.000005, 0.000005, 0.000005).finished());

auto goal_cost = noiseModel::Diagonal::Sigmas(
(Vector(4) << 0.000005, 0.000005, 1000.1, 1000.1).finished());
auto goal_with_angle_cost = noiseModel::Diagonal::Sigmas(
(Vector(6) << 0.000005, 0.000005, 0.000005, 1000.1, 1000.1, 1000.1).finished());

auto tension_cost = noiseModel::Isotropic::Sigma(1, 1e-3);
auto control_cost = noiseModel::Isotropic::Sigma(1, 1e-3);

* load trajectory for each step
* both cable penalty factors, no changes


# 2, 2.1

const double load_mass = 0.005;   // kg
const double inertia = 0.000133;
const double mu = 0.01;
const double mu2 = 0.01;
const double u_upper_bound = 0.5;
const double u_lower_bound = 0.01;
double weight_tension_lower_bound = 100.0; // High weight to strongly enforce T >= 0
double weight_cable_stretch = 1.0;     // Very high weight to strongly prevent cable from over-stretching
double weight_tension_slack = 50.0;

// State-dependent parameters
const double cable_length = sqrt(1.03 * 1.03 - (robot_height_ - 0.2) * (robot_height_ - 0.2));

// --- Define Noise Models ---
// These represent the uncertainty of each factor (1/covariance)
auto dynamics_cost = noiseModel::Diagonal::Sigmas(
(Vector(4) << 0.001, 0.001, 0.001, 0.001).finished());
auto dynamics_with_angle_cost = noiseModel::Diagonal::Sigmas(
(Vector(6) << 0.001, 0.001, 0.001, 0.001, 0.001, 0.001).finished());

auto init_cost = noiseModel::Diagonal::Sigmas(
(Vector(4) << 0.000005, 0.000005, 0.000005, 0.000005).finished());
auto init_with_angle_cost = noiseModel::Diagonal::Sigmas(
(Vector(6) << 0.000005, 0.000005, 0.000005, 0.000005, 0.000005, 0.000005).finished());

auto goal_with_angle_cost = noiseModel::Diagonal::Sigmas(
(Vector(6) << 0.000005, 0.000005, 0.05, 1000.1, 1000.1, 1000.1).finished());

auto tension_cost = noiseModel::Isotropic::Sigma(1, 1e-3);
auto control_cost = noiseModel::Isotropic::Sigma(1, 1e-3);


* load trajectory for the last step
* both cable penalty factotors, no changes
* init for tension is 0.01
* 2 uses numerical derivatives, 2.1 - analytical


# 3

const double load_mass = 0.005;   // kg
const double inertia = 0.000133;
const double mu = 0.01;
const double mu2 = 0.01;
const double u_upper_bound = 0.5;
const double u_lower_bound = 0.01;
double weight_tension_lower_bound = 100.0; // High weight to strongly enforce T >= 0
double weight_cable_stretch = 1.0;     // Very high weight to strongly prevent cable from over-stretching
double weight_tension_slack = 50.0;

// State-dependent parameters
const double cable_length = sqrt(1.03 * 1.03 - (robot_height_ - 0.2) * (robot_height_ - 0.2));
cout << "EXEC: Cable length: " << cable_length << std::endl;

// --- Define Noise Models ---
// These represent the uncertainty of each factor (1/covariance)
auto dynamics_cost = noiseModel::Diagonal::Sigmas(
(Vector(4) << 0.001, 0.001, 0.001, 0.001).finished());
auto dynamics_with_angle_cost = noiseModel::Diagonal::Sigmas(
(Vector(6) << 0.001, 0.001, 0.01, 0.001, 0.001, 0.01).finished());

auto init_cost = noiseModel::Diagonal::Sigmas(
(Vector(4) << 0.000005, 0.000005, 0.000005, 0.000005).finished());
auto init_with_angle_cost = noiseModel::Diagonal::Sigmas(
(Vector(6) << 0.000005, 0.000005, 0.000005, 0.000005, 0.000005, 0.000005).finished());

auto goal_with_angle_cost = noiseModel::Diagonal::Sigmas(
(Vector(6) << 0.000005, 0.000005, 0.00005, 1000.1, 1000.1, 1000.1).finished());

auto tension_cost = noiseModel::Isotropic::Sigma(1, 1e-3);
auto control_cost = noiseModel::Isotropic::Sigma(1, 1e-3);


* anaytical 
* 0.01 for tension
* + 0.001 on tension penalty factor


# 4

const double load_mass = 0.005;   // kg
const double inertia = 0.000133;
const double mu = 0.01;
const double mu2 = 0.01;
const double u_upper_bound = 0.5;
const double u_lower_bound = 0.01;
double weight_tension_lower_bound = 100.0; // High weight to strongly enforce T >= 0
double weight_cable_stretch = 10.0;     // Very high weight to strongly prevent cable from over-stretching
double weight_tension_slack = 50.0;

// State-dependent parameters
const double cable_length = sqrt(1.03 * 1.03 - (robot_height_ - 0.2) * (robot_height_ - 0.2));
cout << "EXEC: Cable length: " << cable_length << std::endl;

// --- Define Noise Models ---
// These represent the uncertainty of each factor (1/covariance)
auto dynamics_cost = noiseModel::Diagonal::Sigmas(
(Vector(4) << 0.001, 0.001, 0.001, 0.001).finished());
auto dynamics_with_angle_cost = noiseModel::Diagonal::Sigmas(
(Vector(6) << 0.001, 0.001, 0.01, 0.001, 0.001, 0.01).finished());

auto init_cost = noiseModel::Diagonal::Sigmas(
(Vector(4) << 0.000005, 0.000005, 0.000005, 0.000005).finished());
auto init_with_angle_cost = noiseModel::Diagonal::Sigmas(
(Vector(6) << 0.000005, 0.000005, 0.000005, 0.000005, 0.000005, 0.000005).finished());

auto goal_with_angle_cost = noiseModel::Diagonal::Sigmas(
(Vector(6) << 0.000005, 0.000005, 0.00005, 1000.1, 1000.1, 1000.1).finished());

auto tension_cost = noiseModel::Isotropic::Sigma(1, 1e-3);
auto control_cost = noiseModel::Isotropic::Sigma(1, 1e-3);


* + 0.001 to tension penalty
* 0.00001 on init tension
* analytical