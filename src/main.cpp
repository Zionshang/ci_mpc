#include <iostream>
#include "contact_implicit_mpc.hpp"

using namespace ci_mpc;

int main(int, char **)
{
    std::string urdf_filename = "/home/zishang/pinocchio_idto_drake_simulator/pinocchio_idto/robot/mini_cheetah/mini_cheetah_ground.urdf";
    std::string srdf_filename = "/home/zishang/pinocchio_idto_drake_simulator/pinocchio_idto/robot/mini_cheetah/mini_cheetah.srdf";

    RobotHandler robot_handler(urdf_filename, srdf_filename);

    int nq = robot_handler.nq();
    int nv = robot_handler.nv();

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
    VectorXd x0 = x_ref_start;

    MpcSettings mpc_settings;
    ContactParameter contact_param;
    ContactImplicitMpc mpc(x0, mpc_settings, contact_param, robot_handler);

    // Define stage state weights
    VectorXd w_pos_diag = VectorXd::Zero(space.getModel().nv);
    VectorXd w_vel_diag = VectorXd::Zero(space.getModel().nv);
    w_pos_diag << 10, 10, 50, // linear part
        1, 1, 1,              // angular part
        0, 0, 0,              // leg part
        0, 0, 0,
        0, 0, 0,
        0, 0, 0;
    w_vel_diag << 1, 1, 1, // linear part
        1, 1, 1,           // angular part
        0.1, 0.1, 0.1,     // leg part
        0.1, 0.1, 0.1,
        0.1, 0.1, 0.1,
        0.1, 0.1, 0.1;
    VectorXd w_x_diag = VectorXd::Zero(ndx);
    w_x_diag << w_pos_diag, w_vel_diag;
    MatrixXd w_x = w_x_diag.asDiagonal();

    // Define terminal state weights
    w_pos_diag << 10, 10, 50,
        10, 10, 10,
        1, 1, 1,
        1, 1, 1,
        1, 1, 1,
        1, 1, 1;
    w_vel_diag << 1, 1, 1,
        1, 1, 1,
        0.1, 0.1, 0.1,
        0.1, 0.1, 0.1,
        0.1, 0.1, 0.1,
        0.1, 0.1, 0.1;
    w_x_diag << w_pos_diag, w_vel_diag;
    MatrixXd w_x_term = w_x_diag.asDiagonal();

    // Define input state weights
    MatrixXd w_u = MatrixXd::Identity(nu, nu);
    w_u.diagonal().array() = 0.01;
}
