#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>

#include "factor_graph_lib/factor_executor_factory.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>


int main(int argc, char** argv) {

    cout << "HEEY!\n";
}