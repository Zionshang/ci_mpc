#include <iostream>
#include <iomanip> // 新增头文件
#include "contact_implicit_mpc.hpp"
#include "utils/project_path.h"
#include "utils/load_config_yaml.h"
#include "utils/logger.hpp"
#include "utils/loop.hpp"
#include "webots_interface.hpp"

using namespace ci_mpc;

VectorXd lowLevelController(double current_time,
                            double mpc_finish_time,
                            const ParamsLoader &params,
                            const std::vector<VectorXd> &x_sol,
                            const std::vector<VectorXd> &u_sol,
                            const VectorXd &x_current)
{
    double dt = current_time - mpc_finish_time;

    // Find the correct interval for the given time
    double mpc_timestep = params.mpc_settings.timestep;
    int index = static_cast<int>(std::floor(dt / mpc_timestep));
    double alpha = (dt - index * mpc_timestep) / mpc_timestep;
    std::cout << "dt: " << dt << "   index: " << index << std::endl;

    // linear interpolation
    VectorXd x_des = (1 - alpha) * x_sol[index] + alpha * x_sol[index + 1];
    VectorXd u_des = (1 - alpha) * u_sol[index] + alpha * u_sol[index + 1];

    // extract actuated joint part
    int nq = 19;

    // PD controller
    VectorXd u_final = u_des +
                       params.Kp * (x_des.segment(6, 12) - x_current.segment(6, 12)) +
                       params.Kd * (x_des.segment(nq + 6, 12) - x_current.segment(nq + 6, 12));

    std::cout << "u_des: " << u_final.transpose() << std::endl;
    std::cout << "u_final: " << u_final.transpose() << std::endl;
    return u_final;
}

int main(int, char **)
{
    std::cout << std::fixed << std::setprecision(4); // 设置输出格式为固定小数点，四位精度
    // std::string urdf_filename = getProjectPath() + "/robot/mini_cheetah/urdf/mini_cheetah_ground.urdf";
    // std::string srdf_filename = getProjectPath() + "/robot/mini_cheetah/srdf/mini_cheetah.srdf";
    std::string urdf_filename = "/home/zishang/pinocchio_idto_drake_simulator/pinocchio_idto/robot/mini_cheetah/mini_cheetah_ground.urdf";
    std::string srdf_filename = "/home/zishang/pinocchio_idto_drake_simulator/pinocchio_idto/robot/mini_cheetah/mini_cheetah.srdf";
    std::string config_filename = getProjectPath() + "/config/config.yaml";

    RobotHandler robot_handler(urdf_filename, srdf_filename);

    int nq = robot_handler.nq();
    int nv = robot_handler.nv();

    ParamsLoader params_loader(config_filename);

    VectorXd x0 = params_loader.x0;
    const MpcSettings &mpc_settings = params_loader.mpc_settings;
    const ContactParameter &contact_param = params_loader.contact_param;
    std::shared_ptr<ContactImplicitMpc> mpc =
        std::make_shared<ContactImplicitMpc>(x0, mpc_settings, contact_param, robot_handler);

    saveVectorsToCsv("trajectory_initial.csv", mpc->x_sol());

    // std::vector<VectorXd> pos_ref(mpc_settings.horizon, x0.head(nq));
    // std::vector<VectorXd> vel_ref(mpc_settings.horizon, x0.tail(nv));
    // VectorXd pos_ref_start = x0.head(nq);
    // std::vector<VectorXd> x_result;
    // double vx = 0;
    // vel_ref[0](0) = vx;

    VectorXd pos_nom = VectorXd::Zero(nq);
    pos_nom.head(nq) << 0.0, 0.0, 0.29,
        0.0, 0.0, 0.0, 1.0,
        0.0, -0.8, 1.6,
        0.0, -0.8, 1.6,
        0.0, -0.8, 1.6,
        0.0, -0.8, 1.6;
    VectorXd vel_nom = VectorXd::Zero(nv);
    std::vector<VectorXd> x_ref(mpc_settings.horizon, x0);
    // for (int i = 0; i < mpc_settings.horizon; i++)
    // {
    //     std::cout << "x_ref[" << i << "]: " << x_ref[i].transpose() << std::endl;
    // }

    // used for test
    const auto &model = robot_handler.model();
    auto space = MultibodyPhaseSpace(model);
    MatrixXd actuation_matrix = MatrixXd::Zero(nv, nv - 6);
    actuation_matrix.bottomRows(nv - 6).setIdentity();

    CompliantContactDynamics dyn = CompliantContactDynamics(space, actuation_matrix,
                                                            robot_handler.geom_model(), params_loader.contact_param);
    auto dyn_data = dyn.createData();
    CompliantContactData &d = static_cast<CompliantContactData &>(*dyn_data);
    IntegratorSemiImplEuler discrete_dyn = IntegratorSemiImplEuler(dyn, mpc_settings.timestep);
    aligator::dynamics::IntegratorSemiImplDataTpl<double> discrete_data(&discrete_dyn);

    std::shared_ptr<WebotsInterface> webots_interface = std::make_shared<WebotsInterface>();

    auto mpc_callback = [mpc, webots_interface]()
    {
        mpc->iterate(webots_interface);
    };
    std::shared_ptr<loop::LoopFunc> mpc_loop = std::make_shared<loop::LoopFunc>(
        "MPC Loop", 1 / params_loader.mpc_frequency, mpc_callback);
    std::shared_ptr<loop::Timer> main_loop_timer = std::make_shared<loop::Timer>(0.001);
    webots_interface->resetSim();

    while (webots_interface->isRunning())
    {
        webots_interface->recvState(x0);
        // pin::integrate(robot_handler.model(), pos_ref_start, vel_ref[0] * mpc_settings.timestep, pos_ref[0]);
        // pos_ref_start = pos_ref[0];

        // for (int i = 1; i < mpc_settings.horizon; i++)
        // {
        //     vel_ref[i] = vel_ref[i - 1];
        //     pin::integrate(robot_handler.model(), pos_ref[i - 1], vel_ref[i - 1] * mpc_settings.timestep, pos_ref[i]);
        //     // std::cout << "vel_ref[" << i << "]: " << vel_ref[i].transpose() << std::endl;
        //     // std::cout << "pos_ref[" << i << "]: " << pos_ref[i].transpose() << std::endl;
        // }

        mpc->updateInitState(x0);
        mpc->updateStateReferences(x_ref);
        // mpc_loop->start();

        // std::cout << "x0: " << x0.transpose() << std::endl;
        // mpc->iterate(x0, pos_ref, vel_ref);
        mpc->iterate(webots_interface);

        const VectorXd &u = mpc->u_sol()[0];

        // std::cout << "mpc.u_sol().size() " << mpc->u_sol().size() << std::endl;
        // std::cout << "x0: " << x0.transpose() << std::endl;
        // for (int i = 0; i < 3; i++)
        // {
        //     std::cout << "pos_ref[" << i << "]: " << pos_ref[i].head(nq).transpose() << std::endl;
        // }

        // low-level controller
        double current_time = webots_interface->current_time();
        // std::cout << "test8" << std::endl;
        // VectorXd u = lowLevelController(current_time, mpc->mpc_finish_time(),
        //                                 params_loader, mpc->x_sol(), mpc->u_sol(), x0);
        // std::cout << "test9" << std::endl;
        webots_interface->sendCmd(u);
        main_loop_timer->sleep();

        // // test
        // dyn.forward(mpc->x_sol()[0], mpc->u_sol()[0], d);
        // auto a = dyn_data->xdot_.segment(nv, nv);
        // std::cout << "a_body: " << a.head(6).transpose() << std::endl;
        // std::cout << "a_leg: " << a.segment(6, 12).transpose() << std::endl;

        discrete_dyn.forward(mpc->x_sol()[0], mpc->u_sol()[0], discrete_data);
        auto x_next = discrete_data.xnext_;
        std::cout << "next pos body: " << x_next.head(7).transpose() << std::endl;
        std::cout << "next pos leg: " << x_next.segment(7, 12).transpose() << std::endl;

        // std::cout << "force[1]: " << d.f_ext_[4].linear() << std::endl;
        // std::cout << "force[2]: " << d.f_ext_[7].linear() << std::endl;
        // std::cout << "force[3]: " << d.f_ext_[10].linear() << std::endl;
        // std::cout << "force[4]: " << d.f_ext_[13].linear() << std::endl;
    }

    // for (int t = 0; t < 100; t++)
    // {
    //     pin::integrate(robot_handler.model(), pos_ref_start, vel_ref[0] * mpc_settings.timestep, pos_ref[0]);
    //     pos_ref_start = pos_ref[0];
    //     for (int i = 1; i < mpc_settings.horizon; i++)
    //     {
    //         vel_ref[i] = vel_ref[i - 1];
    //         pin::integrate(robot_handler.model(), pos_ref[i - 1], vel_ref[i - 1] * mpc_settings.timestep, pos_ref[i]);
    //         // std::cout << "vel_ref[" << i << "]: " << vel_ref[i].transpose() << std::endl;
    //         // std::cout << "pos_ref[" << i << "]: " << pos_ref[i].transpose() << std::endl;
    //     }
    //     mpc.iterate(x0, pos_ref, vel_ref);
    //     x0 = mpc.x_sol()[1];
    //     // std::cout << "t: " << t << std::endl;
    //     // std::cout << "q_ref0[" << t << "]: " << pos_ref[0].transpose() << std::endl;
    //     // std::cout << "q_0   [" << t << "]: " << x0.head(nq).transpose() << std::endl;

    //     x_result.push_back(x0);
    //     // x_result = mpc.x_sol();
    // }

    // saveVectorsToCsv("trajectory_mpc_result.csv", x_result);
}
