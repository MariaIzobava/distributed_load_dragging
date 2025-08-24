# 2 bots no ori

const double u_upper_bound = getd("u_upper_bound", 0.4); 
const double u_lower_bound = getd("u_lower_bound", 0.03);

double weight_tension_lower_bound = getd("weight_tension_lower_bound", 1000000.0);
double weight_cable_stretch = getd("weight_cable_stretch", 100.0) ;
double weight_tension_slack = getd("weight_tension_slack", 50.0);
double weight_tether_tension = getd("weight_tether_tension", .2768);

double cable_stretch_penalty_offset = getd("cable_stretch_penalty_offset", 0.022);
double tension_slack_penalty_offset = getd("tension_slack_penalty_offset", 0.0); 
double tether_tension_offset = getd("tether_tension_offset", 0.03584); 

bool have_tension_lower_bound_factor = getb("have_tension_lower_bound_factor", true);
bool have_cable_stretch_factor = getb("have_cable_stretch_factor", false);
bool have_tension_slack_penalty_factor = getb("have_tension_slack_penalty_factor", true); 
bool have_tether_tension_factor = getb("have_tether_tension_factor", false);

bool have_trajectory_reference_factor = getb("have_trajectory_reference_factor", true);

double goal_cost_v = getd("goal_cost", 0.00005); 

// STATE PRIORS
auto init_cost = noiseModel::Diagonal::Sigmas(
    (Vector(4) << 0.000005, 0.000005, 0.000005, 0.000005).finished());
auto goal_cost = noiseModel::Diagonal::Sigmas(
    (Vector(4) << goal_cost_v, goal_cost_v, 1000.1, 1000.1).finished());

// DYNAMICS
auto dynamics_cost = noiseModel::Diagonal::Sigmas(
    (Vector(4) << 0.001, 0.001, 0.001, 0.001).finished());

// CONTROL
auto control_cost = noiseModel::Isotropic::Sigma(1, 1e-2);
auto control_interim_cost = noiseModel::Diagonal::Sigmas(
    (Vector(2) << 10.0, 10.0).finished());

// TENSION
auto tension_cost = noiseModel::Isotropic::Sigma(1, 1e-3);