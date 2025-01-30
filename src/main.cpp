#include <iostream>
#include "contact_implicit_mpc.hpp"
#include "utils/project_path.h"
#include "utils/load_config_yaml.h"
#include "utils/logger.hpp"

using namespace ci_mpc;

int main(int, char **)
{
    std::string urdf_filename = getProjectPath() + "/robot/mini_cheetah/urdf/mini_cheetah_ground.urdf";
    std::string srdf_filename = getProjectPath() + "/robot/mini_cheetah/srdf/mini_cheetah.srdf";
    std::string config_filename = getProjectPath() + "/config/config.yaml";

    RobotHandler robot_handler(urdf_filename, srdf_filename);

    int nq = robot_handler.nq();
    int nv = robot_handler.nv();

    ParamsLoader params_loader(config_filename);

    VectorXd x0 = params_loader.x0;
    const MpcSettings &mpc_settings = params_loader.mpc_settings;
    const ContactParameter &contact_param = params_loader.contact_param;
    ContactImplicitMpc mpc(x0, mpc_settings, contact_param, robot_handler);

    std::vector<VectorXd> pos_ref(mpc_settings.horizon, x0.head(nq));
    std::vector<VectorXd> vel_ref(mpc_settings.horizon, x0.tail(nv));
    VectorXd pos_ref_start = x0.head(nq);
    std::vector<VectorXd> x_result;

    double vx = 0.1;
    vel_ref[0](0) = vx;
    for (int t = 0; t < 1; t++)
    {
        pin::integrate(robot_handler.model(), pos_ref_start, vel_ref[0] * mpc_settings.timestep, pos_ref[0]);
        pos_ref_start = pos_ref[0];
        for (int i = 1; i < mpc_settings.horizon; i++)
        {
            vel_ref[i] = vel_ref[i - 1];
            pin::integrate(robot_handler.model(), pos_ref[i - 1], vel_ref[i - 1] * mpc_settings.timestep, pos_ref[i]);
            std::cout << "vel_ref[" << i << "]: " << vel_ref[i].transpose() << std::endl;
            std::cout << "pos_ref[" << i << "]: " << pos_ref[i].transpose() << std::endl;
        }
        mpc.iterate(x0, pos_ref, vel_ref);
        x0 = mpc.x_sol()[1];
        // std::cout << "t: " << t << std::endl;
        // std::cout << "q_ref0[" << t << "]: " << pos_ref[0].transpose() << std::endl;
        // std::cout << "q_0   [" << t << "]: " << x0.head(nq).transpose() << std::endl;

        // x_result.push_back(x0);
        x_result = mpc.x_sol();
    }

    saveVectorsToCsv("trajectory_results2.csv", x_result);
}
