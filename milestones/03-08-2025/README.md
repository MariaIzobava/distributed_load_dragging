# 0.005 kg with ori

const double load_mass = 0.005;   // kg
const double inertia = 0.000133;
const double mu = 0.3;
const double mu2 = 0.3;
const double u_upper_bound = 0.4;
const double u_lower_bound = 0.003;
double weight_tension_lower_bound = 1000000.0; // High weight to strongly enforce T >= 0
double weight_cable_stretch = 100.0;     // Very high weight to strongly prevent cable from over-stretching
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

// 0.00005 the best
auto goal_with_angle_cost = noiseModel::Diagonal::Sigmas(
(Vector(6) << 0.000005, 0.000005, 0.00005, 1000.1, 1000.1, 1000.1).finished());

auto tension_cost = noiseModel::Isotropic::Sigma(1, 1e-3);
auto tension2_cost = noiseModel::Isotropic::Sigma(1, 1e-5);
auto control_cost = noiseModel::Isotropic::Sigma(1, 1e-2);


* Used new factor for controlling tension:
TethertensionFactor(symbol_t('t', k), symbol_t('x', k), symbol_t('l', k), cable_length - 0.0002, 3, tension2_cost)
* traj for each point