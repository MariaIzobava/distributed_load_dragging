# 0.005kg 1 robot no ori (tether tension factor only!)

const int num_time_steps = 20;
const double dt = 0.005;
const double robot_mass = 0.025; // kg
const double load_mass = 0.005;   // kg
const double gravity = 9.81;
const double mu = 0.3;
const double cable_length = 0.20 + sqrt(1.03 * 1.03 - (robot_height_ - 0.2) * (robot_height_ - 0.2));

const double u_upper_bound = getd("u_upper_bound", 0.8);
const double u_lower_bound = getd("u_lower_bound", 0.003);

double weight_tension_lower_bound = getd("weight_tension_lower_bound", 1000000.0);
double weight_cable_stretch = getd("weight_cable_stretch", 1000.0);
double weight_tension_slack = getd("weight_tension_slack", 5000.0);
double weight_tether_tension = getd("weight_tether_tension", 12.5);

double cable_stretch_offset = getd("cable_stretch_offset", 0.0001);
double cable_length_offset = getd("cable_length_offset", 0.021875); 
    
bool have_u0_prior = getb("have_u0_prior", false);
bool have_uk_prior = getb("have_uk_prior", false); 
bool have_t0_prior = getb("have_t0_prior", false);
bool have_tk_prior = getb("have_tk_prior", false);

// Goal prior + tether_tension factor makes the full rank system
bool have_goal_prior = getb("have_goal_prior", true);

bool have_tension_lower_bound_factor = getb("have_tension_lower_bound_factor", false);
bool have_cable_stretch_factor = getb("have_cable_stretch_factor", false);
bool have_tension_slack_penalty_factor = getb("have_tension_slack_penalty_factor", false); 
bool have_tether_tension_factor = getb("have_tether_tension_factor", true);

bool have_trajectory_reference_factor = getb("have_trajectory_reference_factor", false);


// STATE PRIORS TUNED VALUES
double goal_cost_v = getd("goal_cost", 0.00005); 
double control_interim_cost_v = getd("control_interim_cost", 1.0);
double tension_cost_v = getd("tension_cost", 1e-3); 


// STATE PRIORS
auto init_cost = noiseModel::Diagonal::Sigmas(
    (Vector(4) << 0.000005, 0.000005, 0.000005, 0.000005).finished());
auto goal_interim_cost = noiseModel::Diagonal::Sigmas(
    (Vector(4) << 0.01,    0.01,    1000.1, 1000.1).finished());
auto goal_cost = noiseModel::Diagonal::Sigmas(
    (Vector(4) << goal_cost_v, goal_cost_v, 1000.1, 1000.1).finished());

// DYNAMICS
auto dynamics_robot_cost = noiseModel::Diagonal::Sigmas(
    (Vector(4) << 0.001, 0.001, 0.001, 0.001).finished());
auto dynamics_load_cost = noiseModel::Diagonal::Sigmas(
    (Vector(4) << 0.001, 0.001, 0.001, 0.001).finished());

// CONTROL
auto control_init_cost = noiseModel::Diagonal::Sigmas(
    (Vector(2) << 10.0, 10.0).finished());
auto control_interim_cost = noiseModel::Diagonal::Sigmas(
    (Vector(2) << control_interim_cost_v, control_interim_cost_v).finished());
auto control_cost = noiseModel::Isotropic::Sigma(1, 1e-3);

// TENSION
auto tension_prior_cost = noiseModel::Isotropic::Sigma(1, 1e-2);
auto tension_interim_cost = noiseModel::Isotropic::Sigma(1, 1e-2);
auto tension_cost = noiseModel::Isotropic::Sigma(1, tension_cost_v);