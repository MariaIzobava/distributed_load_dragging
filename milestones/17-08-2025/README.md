# 2 robots with ori (with traj)

const int num_time_steps = 20;
const double dt = 0.005;
const double robot_mass = 0.025; // kg
const double load_mass = 0.01;   // kg
const double inertia = 0.000266;
const double gravity = 9.81;
const double mu = 0.3;
const double mu2 = 0.3;
const double cable_length1 = sqrt(1.03 * 1.03 - (robot_height1_) * (robot_height1_));
const double cable_length2 = sqrt(1.03 * 1.03 - (robot_height2_) * (robot_height2_));

//         tether_tension_offset 0.28672
// weight_tether_tension 0.4096

// VALUES TO TUNE
// =============================
// =============================
const double u_upper_bound = getd("u_upper_bound", 0.6); 
const double u_lower_bound = getd("u_lower_bound", 0.003);

double weight_tension_lower_bound = getd("weight_tension_lower_bound", 1000000.0);
double weight_cable_stretch = getd("weight_cable_stretch", 100.0) ;
double weight_tension_slack = getd("weight_tension_slack", 50.0);
double weight_tether_tension = getd("weight_tether_tension", 0.0008096); //0.0008096

double cable_stretch_penalty_offset = getd("cable_stretch_penalty_offset", 0.0);
double tension_slack_penalty_offset = getd("tension_slack_penalty_offset", 0.2); 
double tether_tension_offset = getd("tether_tension_offset", 0.38672); //0.28672

bool have_uk_prior = getb("have_uk_prior", true);

bool have_tension_lower_bound_factor = getb("have_tension_lower_bound_factor", false);
bool have_cable_stretch_factor = getb("have_cable_stretch_factor", false);
bool have_tension_slack_penalty_factor = getb("have_tension_slack_penalty_factor", false); 
bool have_tether_tension_factor = getb("have_tether_tension_factor", true);

bool have_trajectory_reference_factor = getb("have_trajectory_reference_factor", true);

double goal_cost_v = getd("goal_cost", 0.00005); 

// STATE PRIORS
auto init_cost = noiseModel::Diagonal::Sigmas(
    (Vector(4) << 0.000005, 0.000005, 0.000005, 0.000005).finished());
auto init_cost_with_angle = noiseModel::Diagonal::Sigmas(
    (Vector(6) << 0.000005, 0.000005, 0.000005, 0.000005, 0.000005, 0.000005).finished());
auto goal_cost_with_angle = noiseModel::Diagonal::Sigmas(
    (Vector(6) << 0.000005, 0.000005, 0.00005, 1000.1, 1000.1, 1000.1).finished());

// DYNAMICS
auto dynamics_cost = noiseModel::Diagonal::Sigmas(
    (Vector(4) << 0.001, 0.001, 0.001, 0.001).finished());
auto dynamics_cost_with_angle = noiseModel::Diagonal::Sigmas(
    (Vector(6) << 0.001, 0.001, 0.01, 0.001, 0.001, 0.01).finished());

// CONTROL
auto control_cost = noiseModel::Isotropic::Sigma(1, 1e-2);
auto control_interim_cost = noiseModel::Diagonal::Sigmas(
    (Vector(2) << 10.0, 10.0).finished());

// TENSION
auto tension_cost = noiseModel::Isotropic::Sigma(1, 1e-3);
auto tension2_cost = noiseModel::Isotropic::Sigma(1, 1e-4);


# 2 robots with ori no traj 

Same as previous only have_trajectory_reference_factor is false.