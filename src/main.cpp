#include <iostream>
#include "contact_implicit_mpc.hpp"
#include "utils/project_path.h"
#include "utils/load_config_yaml.h"

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

    VectorXd x_ref_start = VectorXd::Zero(nq + nv);
    x_ref_start.head(nq) << 0.0, 0.0, 0.29,
        0.0, 0.0, 0.0, 1.0,
        0.0, -0.8, 1.6,
        0.0, -0.8, 1.6,
        0.0, -0.8, 1.6,
        0.0, -0.8, 1.6;
    VectorXd x_ref_end = VectorXd::Zero(nq + nv);
    x_ref_end.head(nq) << 0.5, 0.0, 0.29,
        0.0, 0.0, 0.0, 1.0,
        0.0, -0.8, 1.6,
        0.0, -0.8, 1.6,
        0.0, -0.8, 1.6,
        0.0, -0.8, 1.6;

    const VectorXd &x0 = params_loader.x0;
    const MpcSettings &mpc_settings = params_loader.mpc_settings;
    const ContactParameter &contact_param = params_loader.contact_param;
    ContactImplicitMpc mpc(x0, mpc_settings, contact_param, robot_handler);
}
