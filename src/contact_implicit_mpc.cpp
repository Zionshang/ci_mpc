#include "contact_implicit_mpc.hpp"

namespace ci_mpc
{
    ContactImplicitMpc::ContactImplicitMpc(const ConstVectorRef &x0,
                                           const MpcSettings &mpc_settings,
                                           const ContactParameter &contact_param,
                                           const RobotHandler &robot_handler)
        : mpc_settings_(mpc_settings),
          contact_param_(contact_param),
          robot_handler_(robot_handler)
    {
        nq_ = robot_handler_.nq();
        nv_ = robot_handler_.nv();
        nu_ = nv_ - 6;

        actuation_matrix_.resize(nv_, nu_);
        actuation_matrix_.setZero();
        actuation_matrix_.bottomRows(nu_).setIdentity();

        // solver settings
        solver_ = std::make_unique<SolverProxDDP>(mpc_settings_.TOL,
                                                  mpc_settings_.mu_init,
                                                  mpc_settings_.first_iters,
                                                  aligator::VerboseLevel::QUIET);
        solver_->rollout_type_ = aligator::RolloutType::LINEAR;
        if (mpc_settings_.num_threads > 1)
        {
            solver_->linear_solver_choice = aligator::LQSolverChoice::PARALLEL;
            solver_->setNumThreads(mpc_settings_.num_threads);
        }
        else
            solver_->linear_solver_choice = aligator::LQSolverChoice::SERIAL;
        solver_->force_initial_condition_ = true;

        // setup initial reference trajectory
        x_ref_.assign(mpc_settings_.horizon, x0);

        // create mpc problem
        const auto &model = robot_handler_.model();
        const auto &geom_model = robot_handler_.geom_model();
        MultibodyPhaseSpace space(model);
        CompliantContactDynamics dynamics(space, actuation_matrix_, geom_model, contact_param_);
        VectorXd u0 = VectorXd::Zero(nu_);
        createTrajOptProblem(dynamics, x_ref_, x0, u0);

        // solve initial problem
        x_sol_.assign(mpc_settings_.horizon + 1, x0);
        u_sol_.assign(mpc_settings_.horizon, u0);
        solver_->setup(*problem_);
        solver_->run(*problem_, x_sol_, u_sol_);
        std::cout << "Initial MPC solved" << std::endl;
        fmt::print("Results: {}\n", solver_->results_);

        // update guess for next iteration
        x_sol_ = solver_->results_.xs;
        u_sol_ = solver_->results_.us;

        // set max iterations to mpc settings
        solver_->max_iters = mpc_settings_.max_iters;
    }

    void ContactImplicitMpc::createTrajOptProblem(const CompliantContactDynamics &dynamics,
                                                  const std::vector<VectorXd> &x_ref,
                                                  const VectorXd &x0, const VectorXd &u0)
    {
        // todo: 现在dynamics只能靠传参进来，如果在这里创建的话，会报错，报错位置是pinocchio计算碰撞体距离的地方，暂时不知道原因
        const auto space = dynamics.space();

        IntegratorSemiImplEuler discrete_dyn = IntegratorSemiImplEuler(dynamics, mpc_settings_.timestep);
        DynamicsFiniteDifference finite_diff_dyn(space, discrete_dyn, mpc_settings_.timestep); // todo: 这里的timestep有待测试

        std::vector<xyz::polymorphic<StageModel>> stage_models;
        for (size_t i = 0; i < mpc_settings_.horizon; i++)
        {
            auto rcost = CostStack(space, nu_);
            rcost.addCost("state_cost", QuadraticStateCost(space, nu_, x_ref[i], mpc_settings_.w_x));
            rcost.addCost("control_cost", QuadraticControlCost(space, u0, mpc_settings_.w_u));

            StageModel stage(rcost, finite_diff_dyn);
            stage_models.push_back(std::move(stage));
        }

        auto term_cost = QuadraticStateCost(space, nu_, x_ref.back(), mpc_settings_.w_x_term);
        problem_ = std::make_unique<TrajOptProblem>(x0, std::move(stage_models), term_cost);
    }

    StageModel ContactImplicitMpc::createStage(const ConstVectorRef &x_ref)
    {
        const auto &model = robot_handler_.model();
        auto space = MultibodyPhaseSpace(model);
        auto rcost = CostStack(space, nu_);

        rcost.addCost("state_cost", QuadraticStateCost(space, nu_, x_ref, mpc_settings_.w_x));
        rcost.addCost("control_cost", QuadraticControlCost(space, VectorXd::Zero(nu_), mpc_settings_.w_u));

        CompliantContactDynamics dyn = CompliantContactDynamics(space, actuation_matrix_,
                                                                robot_handler_.geom_model(), contact_param_);

        IntegratorSemiImplEuler discrete_dyn = IntegratorSemiImplEuler(dyn, mpc_settings_.timestep);
        DynamicsFiniteDifference finite_diff_dyn(space, discrete_dyn, mpc_settings_.timestep); // TODO: 测试timestep
        StageModel stage_mdoel = StageModel(rcost, finite_diff_dyn);

        // TODO: 添加关节力矩约束
        return stage_mdoel;
    }

    CostStack ContactImplicitMpc::createTerminalCost(const ConstVectorRef &x_ref)
    {
        auto term_space = MultibodyPhaseSpace(robot_handler_.model());
        auto term_cost = CostStack(term_space, nu_);
        term_cost.addCost("term_state_cost", QuadraticStateCost(term_space, nu_, x_ref, mpc_settings_.w_x));

        return term_cost;
    }

    void ContactImplicitMpc::creatMpcProblem(const ConstVectorRef &x0, const std::vector<VectorXd> &x_ref)
    {
        std::vector<xyz::polymorphic<StageModel>> stage_models;
        for (size_t i = 0; i < mpc_settings_.horizon; i++)
        {
            StageModel stage = createStage(x_ref[i]);
            stage_models.push_back(std::move(stage));
        }
        auto term_cost = createTerminalCost(x_ref.back());
        problem_ = std::make_unique<TrajOptProblem>(x0, std::move(stage_models), term_cost);
    }

    void ContactImplicitMpc::iterate(const ConstVectorRef &x0,
                                     const std::vector<VectorXd> &pos_ref,
                                     const std::vector<VectorXd> &vel_ref)
    {
        // Update references
        updateStateReferences(pos_ref, vel_ref);
        // std::cout << "References updated" << std::endl;

        // Recede previous solutions
        x_sol_.erase(x_sol_.begin());
        x_sol_[0] = x0;
        x_sol_.push_back(x_sol_.back());

        u_sol_.erase(u_sol_.begin());
        u_sol_.push_back(u_sol_.back());
        // std::cout << "Previous solutions receded" << std::endl;

        // Update initial state
        problem_->setInitState(x0);
        // std::cout << "Initial state updated" << std::endl;

        // Run solver
        solver_->run(*problem_, x_sol_, u_sol_);

        // Collect results
        x_sol_ = solver_->results_.xs;
        u_sol_ = solver_->results_.us;
    }
    void ContactImplicitMpc::updateStateReferences(const std::vector<VectorXd> &pos_ref,
                                                   const std::vector<VectorXd> &vel_ref)
    {
        for (size_t i = 0; i < mpc_settings_.horizon; i++)
        {
            x_ref_[i].head(nq_) = pos_ref[i];
            x_ref_[i].tail(nv_) = vel_ref[i];

            CostStack *cs = dynamic_cast<CostStack *>(&*problem_->stages_[i]->cost_);
            QuadraticStateCost *qsc = cs->getComponent<QuadraticStateCost>("state_cost");
            qsc->setTarget(x_ref_[i]);
        }
    }

} // namespace mpc
