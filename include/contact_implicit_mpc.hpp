#pragma once
#include <proxsuite-nlp/modelling/spaces/multibody.hpp>
#include <aligator/core/stage-model.hpp>
#include <aligator/core/traj-opt-problem.hpp>
#include <aligator/modelling/costs/sum-of-costs.hpp>
#include <aligator/modelling/costs/quad-state-cost.hpp>
#include <aligator/modelling/dynamics/ode-abstract.hpp>
#include <aligator/modelling/dynamics/integrator-semi-euler.hpp>
#include <aligator/modelling/autodiff/finite-difference.hpp>
#include <aligator/modelling/dynamics/integrator-rk2.hpp>
#include <aligator/modelling/dynamics/integrator-euler.hpp>

#include <aligator/solvers/proxddp/solver-proxddp.hpp>

#include "compliant_contact_dynamics.hpp"
#include "robot_handler.hpp"
#include "mpc_settings.hpp"
#include "webots_interface.hpp"
namespace ci_mpc
{
    using StageModel = aligator::StageModelTpl<double>;
    using CostStack = aligator::CostStackTpl<double>;
    using QuadraticControlCost = aligator::QuadraticControlCostTpl<double>;
    using QuadraticStateCost = aligator::QuadraticStateCostTpl<double>;
    using IntegratorSemiImplEuler = aligator::dynamics::IntegratorSemiImplEulerTpl<double>;
    using DynamicsFiniteDifference = aligator::autodiff::DynamicsFiniteDifferenceHelper<double>;
    using TrajOptProblem = aligator::TrajOptProblemTpl<double>;
    using IntegratorRK2 = aligator::dynamics::IntegratorRK2Tpl<double>;
    using IntegratorEuler = aligator::dynamics::IntegratorEulerTpl<double>;

    using aligator::context::SolverProxDDP;

    /**
     * @brief Build a MPC object holding an instance of a trajectory optimization problem
     */

    class ContactImplicitMpc
    {
    public:
        ContactImplicitMpc(const ConstVectorRef &x0,
                           const MpcSettings &mpc_settings,
                           const ContactParameter &contact_param,
                           const RobotHandler &robot_handler);

        void iterate(const std::shared_ptr<WebotsInterface> &webots_interface);
        void iterate(const ConstVectorRef &x0,const std::vector<VectorXd> &x_ref);
        void updateStateReferences(const std::vector<VectorXd> &x_ref);

        void updateInitState(const ConstVectorRef &x0);

        const std::vector<VectorXd> &x_sol() const { return x_sol_; }
        const std::vector<VectorXd> &u_sol() const { return u_sol_; }
        const std::vector<MatrixXd> &K_sol() const { return K_sol_; }
        const double mpc_finish_time() const { return mpc_finish_time_; }

    private:
        StageModel createStage(const ConstVectorRef &x_ref);
        CostStack createTerminalCost(const ConstVectorRef &x_ref);
        void creatMpcProblem(const ConstVectorRef &x0, const std::vector<VectorXd> &x_ref);

        std::vector<VectorXd> x_ref_;
        // VectorXd x0_;

        // Solution vectors for state and control
        std::vector<VectorXd> x_sol_;
        std::vector<VectorXd> u_sol_;
        // Riccati gains
        std::vector<MatrixXd> K_sol_;

        std::vector<VectorXd> x_guess_;
        std::vector<VectorXd> u_guess_;

        double mpc_finish_time_;

        std::unique_ptr<TrajOptProblem> problem_;
        std::unique_ptr<SolverProxDDP> solver_;

        MpcSettings mpc_settings_;
        ContactParameter contact_param_;

        MatrixXd actuation_matrix_; // TODO: 定义在这里是否合适？

        RobotHandler robot_handler_;
        int nq_; // TODO: 定义在这里是否合适？
        int nv_;
        int nu_;
    };
} // namespace ci_mpc
