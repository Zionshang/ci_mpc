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

        mpc_finish_time_ = 0.0;

        // solver settings
        solver_ = std::make_unique<SolverProxDDP>(mpc_settings_.TOL,
                                                  mpc_settings_.mu_init,
                                                  mpc_settings_.first_iters,
                                                  aligator::VerboseLevel::QUIET);
        solver_->rollout_type_ = aligator::RolloutType::LINEAR;
        // solver_->sa_strategy_ = aligator::StepAcceptanceStrategy::FILTER;
        // solver_->filter_.beta_ = 1e-5;
        solver_->force_initial_condition_ = true;
        // solver_->reg_min = 1e-6;

        if (mpc_settings_.num_threads > 1)
        {
            solver_->linear_solver_choice = aligator::LQSolverChoice::PARALLEL;
            solver_->setNumThreads(mpc_settings_.num_threads);
        }
        else
            solver_->linear_solver_choice = aligator::LQSolverChoice::SERIAL;

        // setup initial reference trajectory
        x_ref_.assign(mpc_settings_.horizon, x0);

        // create mpc problem
        const auto &model = robot_handler_.model();
        const auto &geom_model = robot_handler_.geom_model();
        MultibodyPhaseSpace space(model);
        CompliantContactDynamics dynamics(space, actuation_matrix_, geom_model, contact_param_);
        VectorXd u0 = VectorXd::Zero(nu_);
        // createTrajOptProblem(dynamics, x_ref_, x0, u0);
        creatMpcProblem(x0, x_ref_);

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
        K_sol_ = solver_->results_.getCtrlFeedbacks();

        // for (size_t i = 0; i < x_sol_.size(); i++)
        // {
        //     std::cout << "x_sol[" << i << "]: " << x_sol_[i].transpose() << std::endl;
        // }

        // set max iterations to mpc settings
        solver_->max_iters = mpc_settings_.max_iters;
        x_guess_ = x_sol_;
        u_guess_ = u_sol_;
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
        // IntegratorRK2 discrete_dyn = IntegratorRK2(dyn, mpc_settings_.timestep);
        // IntegratorEuler discrete_dyn = IntegratorEuler(dyn, mpc_settings_.timestep);
        DynamicsFiniteDifference finite_diff_dyn(space, discrete_dyn, mpc_settings_.finite_diff_step); // TODO: 测试timestep
        StageModel stage_mdoel = StageModel(rcost, finite_diff_dyn);

        // TODO: 添加关节力矩约束
        return stage_mdoel;
    }

    CostStack ContactImplicitMpc::createTerminalCost(const ConstVectorRef &x_ref)
    {
        auto term_space = MultibodyPhaseSpace(robot_handler_.model());
        auto term_cost = CostStack(term_space, nu_);
        term_cost.addCost("term_state_cost", QuadraticStateCost(term_space, nu_, x_ref, mpc_settings_.w_x_term));
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

    void ContactImplicitMpc::iterate(const std::shared_ptr<WebotsInterface> &webots_interface)
    {
        std::cout << "---------------------------" << std::endl;
        auto start = std::chrono::high_resolution_clock::now();

        // Run solver
        solver_->run(*problem_, x_guess_, u_guess_);

        // Collect results
        x_sol_ = solver_->results_.xs;
        u_sol_ = solver_->results_.us;
        K_sol_ = solver_->results_.getCtrlFeedbacks();

        auto end = std::chrono::high_resolution_clock::now();
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        mpc_finish_time_ = webots_interface->current_time();
        // std::cout << "mpc finish time: " << mpc_finish_time_ << std::endl;
        std::cout << "iterate runtime: " << elapsed_ms << " ms" << std::endl;
        std::cout << "x0 : " << problem_->getInitState().transpose() << std::endl;

        for (size_t i = 0; i < 3; i++)
        {
            CostStack *cs = dynamic_cast<CostStack *>(&*problem_->stages_[i]->cost_);
            QuadraticStateCost *qsc = cs->getComponent<QuadraticStateCost>("state_cost");
            std::cout << "x_ref[" << i << "]: " << qsc->getTarget().transpose() << std::endl;
        }
        CostStack *cs = dynamic_cast<CostStack *>(&*problem_->term_cost_);
        QuadraticStateCost *qsc = cs->getComponent<QuadraticStateCost>("term_state_cost");
        std::cout << "x_ref_term" << qsc->getTarget().transpose() << std::endl;

        for (size_t i = 0; i < 3; i++)
        {
            std::cout << "body_sol[" << i << "]: " << x_sol_[i].head(7).transpose() << std::endl;
        }
        for (size_t i = 0; i < 3; i++)
        {
            std::cout << "leg_sol[" << i << "]: " << x_sol_[i].segment(7, 12).transpose() << std::endl;
        }
        for (size_t i = 0; i < 3; i++)
        {
            std::cout << "u_sol[" << i << "]: " << u_sol_[i].transpose() << std::endl;
        }
    }

    void ContactImplicitMpc::iterate(const ConstVectorRef &x0,
                                     const std::vector<VectorXd> &x_ref)
    {

        auto start = std::chrono::high_resolution_clock::now();

        // Update references
        updateStateReferences(x_ref);
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

        // for (int i = 0; i < 3; i++)
        // {
        //     std::cout << "x[" << i << "]: " << x_sol_[i].head(nq_).transpose() << std::endl;
        // }
        // for (size_t i = 0; i < 3; i++)
        // {
        //     CostStack *cs = dynamic_cast<CostStack *>(&*problem_->stages_[i]->cost_);
        //     QuadraticStateCost *qsc = cs->getComponent<QuadraticStateCost>("state_cost");
        //     std::cout << "x_ref[" << i << "]: " << qsc->getTarget().transpose() << std::endl;
        // }
        // for (int i = 0; i < 2; i++)
        // {
        //     std::cout << "u[" << i << "]: " << u_sol_[i].transpose() << std::endl;
        // }
        auto end = std::chrono::high_resolution_clock::now();
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        std::cout << "iterate runtime: " << elapsed_ms << " ms" << std::endl;
    }

    // TODO: 不使用传参，而是直接使用类成员变量
    void ContactImplicitMpc::updateStateReferences(const std::vector<VectorXd> &x_ref)
    {
        for (size_t i = 0; i < mpc_settings_.horizon; i++)
        {
            CostStack *cs = dynamic_cast<CostStack *>(&*problem_->stages_[i]->cost_);
            QuadraticStateCost *qsc = cs->getComponent<QuadraticStateCost>("state_cost");
            qsc->setTarget(x_ref[i]);
        }
        CostStack *cs = dynamic_cast<CostStack *>(&*problem_->term_cost_);
        QuadraticStateCost *qsc = cs->getComponent<QuadraticStateCost>("term_state_cost");
        qsc->setTarget(x_ref.back());
    }

    void ContactImplicitMpc::updateInitState(const ConstVectorRef &x0)
    {
        // Recede previous solutions
        x_guess_.erase(x_guess_.begin());
        x_guess_[0] = x0;
        x_guess_.push_back(x_guess_.back());

        u_guess_.erase(u_guess_.begin());
        u_guess_.push_back(u_guess_.back());

        // Update initial state
        problem_->setInitState(x0);
    }

} // namespace mpc
