#include <iostream>
#include "contact_implicit_mpc.hpp"
#include "utils/project_path.h"
#include "utils/load_config_yaml.h"
#include "utils/logger.hpp"
#include "webots_interface.hpp"

using namespace ci_mpc;

int main(int, char **)
{
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
    ContactImplicitMpc mpc(x0, mpc_settings, contact_param, robot_handler);
    saveVectorsToCsv("trajectory_initial.csv", mpc.x_sol());

    std::vector<VectorXd> pos_ref(mpc_settings.horizon, x0.head(nq));
    std::vector<VectorXd> vel_ref(mpc_settings.horizon, x0.tail(nv));
    VectorXd pos_ref_start = x0.head(nq);
    std::vector<VectorXd> x_result;

    double vx = 0;
    vel_ref[0](0) = vx;


    // used for test
    const auto &model = robot_handler.model();
    auto space = MultibodyPhaseSpace(model);
    MatrixXd actuation_matrix = MatrixXd::Zero(nv, nv - 6);
    actuation_matrix.bottomRows(nv - 6).setIdentity();

    CompliantContactDynamics dyn = CompliantContactDynamics(space, actuation_matrix,
                                                            robot_handler.geom_model(), params_loader.contact_param);
    auto dyn_data = dyn.createData();
    CompliantContactData &d = static_cast<CompliantContactData &>(*dyn_data);

    // WebotsInterface webots_interface;
    // while (webots_interface.isRunning())
    // {
    //     std::cout << "---------------------------" << std::endl;
    //     webots_interface.recvState(x0);
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
    //     const VectorXd &u = mpc.u_sol()[0];
    //     std::cout << "mpc.u_sol().size() " << mpc.u_sol().size() << std::endl;
    //     std::cout << "x0: " << x0.transpose() << std::endl;
    //     // for (int i = 0; i < 3; i++)
    //     // {
    //     //     std::cout << "pos_ref[" << i << "]: " << pos_ref[i].head(nq).transpose() << std::endl;
    //     // }
    //     webots_interface.sendCmd(u);

    //     // test
    //     dyn.forward(x0, u, d);
    //     auto a = dyn_data->xdot_.segment(nv, nv);
    //     std::cout << "a: " << a.transpose() << std::endl;
    //     for (int i = 0; i < 4; i++)
    //     {
    //         std::cout << "force[" << i << "]: " << d.contact_forces_[i].transpose() << std::endl;
    //     }
    // }

    for (int t = 0; t < 100; t++)
    {
        pin::integrate(robot_handler.model(), pos_ref_start, vel_ref[0] * mpc_settings.timestep, pos_ref[0]);
        pos_ref_start = pos_ref[0];
        for (int i = 1; i < mpc_settings.horizon; i++)
        {
            vel_ref[i] = vel_ref[i - 1];
            pin::integrate(robot_handler.model(), pos_ref[i - 1], vel_ref[i - 1] * mpc_settings.timestep, pos_ref[i]);
            // std::cout << "vel_ref[" << i << "]: " << vel_ref[i].transpose() << std::endl;
            // std::cout << "pos_ref[" << i << "]: " << pos_ref[i].transpose() << std::endl;
        }
        mpc.iterate(x0, pos_ref, vel_ref);
        x0 = mpc.x_sol()[1];
        // std::cout << "t: " << t << std::endl;
        // std::cout << "q_ref0[" << t << "]: " << pos_ref[0].transpose() << std::endl;
        // std::cout << "q_0   [" << t << "]: " << x0.head(nq).transpose() << std::endl;

        x_result.push_back(x0);
        // x_result = mpc.x_sol();
    }

    saveVectorsToCsv("trajectory_mpc_result.csv", x_result);
}
