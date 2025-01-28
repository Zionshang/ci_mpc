#pragma once

#include "math_types.hpp"

namespace ci_mpc
{
    struct MpcSettings
    {
        // timestep in problem shooting nodes
        double timestep = 0.05;

        // prediciton horizon in mpc
        size_t horizon = 100;

        // Cost function weights
        MatrixXd w_x;      // State
        MatrixXd w_u;      // Control
        MatrixXd w_x_term; // Terminal state

        // Solver-related quantities
        double TOL = 1e-4;
        double mu_init = 1e-8;
        size_t max_iters = 1;
        size_t num_threads = 4;
        size_t first_iters = 100;
    };

} // namespace ci_mpc
