#pragma once

#include <string>
#include <iostream>
#include <yaml-cpp/yaml.h>

#include "math_types.hpp"
#include "mpc_settings.hpp"
#include "contact_parameter.hpp"

namespace ci_mpc
{
    struct ParamsLoader
    {
        ParamsLoader(std::string yaml_file)
        {
            YAML::Node config;
            try
            {
                config = YAML::LoadFile(yaml_file);
            }
            catch (YAML::BadFile &e)
            {
                throw "Read yaml file error!";
            }
            // load yaml parameters
            std::vector<double> q_init_vec = config["q_init"].as<std::vector<double>>();
            std::vector<double> v_init_vec = config["v_init"].as<std::vector<double>>();
            std::vector<double> w_q_vec = config["w_q"].as<std::vector<double>>();
            std::vector<double> w_v_vec = config["w_v"].as<std::vector<double>>();
            std::vector<double> w_u_vec = config["w_u"].as<std::vector<double>>();
            std::vector<double> w_q_term_vec = config["w_q_term"].as<std::vector<double>>();
            std::vector<double> w_v_term_vec = config["w_v_term"].as<std::vector<double>>();

            double contact_stiffness = config["contact_stiffness"].as<double>();
            double dissipation_velocity = config["dissipation_velocity"].as<double>();
            double smoothing_factor = config["smoothing_factor"].as<double>();
            double friction_coefficient = config["friction_coefficient"].as<double>();
            double stiction_velocity = config["stiction_velocity"].as<double>();

            double timestep = config["timestep"].as<double>();
            size_t horizon = config["horizon"].as<size_t>();
            double TOL = config["TOL"].as<double>();
            double mu_init = config["mu_init"].as<double>();
            size_t max_iters = config["max_iters"].as<size_t>();
            size_t num_threads = config["num_threads"].as<size_t>();
            size_t first_iters = config["first_iters"].as<size_t>();
            double finite_diff_step = config["finite_diff_step"].as<double>();

            // convert std::vector to Eigen::VectorXd
            VectorXd q_init = Eigen::Map<VectorXd>(q_init_vec.data(), q_init_vec.size());
            VectorXd v_init = Eigen::Map<VectorXd>(v_init_vec.data(), v_init_vec.size());
            VectorXd w_q_diag = Eigen::Map<VectorXd>(w_q_vec.data(), w_q_vec.size());
            VectorXd w_v_diag = Eigen::Map<VectorXd>(w_v_vec.data(), w_v_vec.size());
            VectorXd w_u_diag = Eigen::Map<VectorXd>(w_u_vec.data(), w_u_vec.size());
            VectorXd w_q_term_diag = Eigen::Map<VectorXd>(w_q_term_vec.data(), w_q_term_vec.size());
            VectorXd w_v_term_diag = Eigen::Map<VectorXd>(w_v_term_vec.data(), w_v_term_vec.size());

            // load mpc settings
            VectorXd w_x_diag = VectorXd::Zero(w_q_diag.size() + w_v_diag.size());
            VectorXd w_x_term_diag = VectorXd::Zero(w_q_diag.size() + w_v_diag.size());
            w_x_diag << w_q_diag, w_v_diag;
            w_x_term_diag << w_q_term_diag, w_v_term_diag;
            mpc_settings.timestep = timestep;
            mpc_settings.horizon = horizon;
            mpc_settings.TOL = TOL;
            mpc_settings.mu_init = mu_init;
            mpc_settings.max_iters = max_iters;
            mpc_settings.num_threads = num_threads;
            mpc_settings.first_iters = first_iters;
            mpc_settings.w_x = w_x_diag.asDiagonal();
            mpc_settings.w_x_term = w_x_term_diag.asDiagonal();
            mpc_settings.w_u = w_u_diag.asDiagonal();
            mpc_settings.finite_diff_step = finite_diff_step;
            
            // load contact parameters
            contact_param.contact_stiffness = contact_stiffness;
            contact_param.dissipation_velocity = dissipation_velocity;
            contact_param.smoothing_factor = smoothing_factor;
            contact_param.friction_coefficient = friction_coefficient;
            contact_param.stiction_velocity = stiction_velocity;

            // load initial state
            x0.setZero(q_init.size() + v_init.size());
            x0 << q_init, v_init;
        }

        MpcSettings mpc_settings;
        ContactParameter contact_param;
        VectorXd x0;
    };

} // namespace ci_mpc
