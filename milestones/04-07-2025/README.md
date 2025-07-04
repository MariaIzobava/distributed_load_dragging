## Setup

best control loop timer = 100 milliseconds

### CONSTANTS

```
const int num_time_steps = 20;
const double dt = 0.005;
const double robot_mass = 0.025; // kg
const double load_mass = 0.0001;   // kg
const double gravity = -9.81;
const double mu = 0.1;
const double cable_length = 1.0; // meters
const double u_upper_bound = 0.3;
const double u_lower_bound = 0.003;
double weight_tension_lower_bound = 1000000.0;
double weight_tension_slack = 50.0;
```

### COSTS

```
auto dynamics_cost = noiseModel::Diagonal::Sigmas(
(Vector(4) << 0.001, 0.001, 0.001, 0.001).finished());
auto goal_cost = noiseModel::Diagonal::Sigmas(
(Vector(4) << 0.00005, 0.00005, 1000.1, 1000.1).finished());
auto init_cost = noiseModel::Diagonal::Sigmas(
(Vector(4) << 0.000005, 0.000005, 0.000005, 0.000005).finished());
auto tension_cost = noiseModel::Isotropic::Sigma(1, 1e-3);
auto control_cost = noiseModel::Isotropic::Sigma(1, 1e-3);
```

### FACTORS

1. RobotDynamicsFactors for drone and load
2. PriorFactors for drone and load initial state
3. TensionLowerBoundFactor and TensionSlackPenaltyFactor 
4. MagnitudeUpperBoundFactor and MagnitudeLowerBoundFactor for drone control input
5. PriorFactor for load reference trajectory


## Next steps

1. Improve load and cable models
2. Fix CableStretchPenaltyFactor factor (it was not used in this milestone)
3. Add second robot