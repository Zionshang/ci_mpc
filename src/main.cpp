#include <iostream>
#include "contact_implicit_mpc.hpp"
#include "utils/project_path.h"
#include "utils/load_config_yaml.h"

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

    std::vector<VectorXd> x_result;

    double vx = 0.5;
    vel_ref[0](0) = vx * mpc_settings.timestep;

    for (int t = 0; t < 100; t++)
    {
        for (int i = 1; i < mpc_settings.horizon; i++)
        {
            vel_ref[i] = vel_ref[i - 1];
            pin::integrate(robot_handler.model(), pos_ref[i - 1], vel_ref[i - 1], pos_ref[i]);
        }
        mpc.iterate(x0, pos_ref, vel_ref);
        x0 = mpc.x_sol()[1];
        std::cout << "t: " << t << std::endl;
        std::cout << "x0: " << x0.transpose() << std::endl;
        x_result.push_back(x0);
    }

    std::ofstream outFile("trajectory_results.csv");
    if (outFile.is_open())
    {
        for (size_t i = 0; i < x_result.size(); i++)
        {
            VectorXd q = x_result[i].head(nq);
            for (int j = 0; j < nq; ++j)
            {
                outFile << q[j];
                if (j < nq - 1)
                    outFile << ",";
            }
            outFile << "\n";
        }
        outFile.close();
        std::cout << "Results saved to trajectory_results.csv" << std::endl;
    }
    else
    {
        std::cerr << "Unable to open file for writing" << std::endl;
    }
}
