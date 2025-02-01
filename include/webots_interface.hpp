#pragma once

#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Gyro.hpp>
#include <webots/Supervisor.hpp>
#include <Eigen/Dense>

inline Eigen::Matrix3d quat2RotMat(const Eigen::Vector4d &q)
{
    double e0 = q(0); // x
    double e1 = q(1); // y
    double e2 = q(2); // z
    double e3 = q(3); // w

    Eigen::Matrix3d R;
    R << 1 - 2 * (e1 * e1 + e2 * e2), 2 * (e0 * e1 - e3 * e2), 2 * (e0 * e2 + e3 * e1),
        2 * (e0 * e1 + e3 * e2), 1 - 2 * (e0 * e0 + e2 * e2), 2 * (e1 * e2 - e3 * e0),
        2 * (e0 * e2 - e3 * e1), 2 * (e1 * e2 + e3 * e0), 1 - 2 * (e0 * e0 + e1 * e1);
    return R;
}

class WebotsInterface
{
public:
    WebotsInterface();
    ~WebotsInterface();
    void recvState(Eigen::VectorXd &state_vector); // second
    void sendCmd(const Eigen::VectorXd &tau);
    bool isRunning();
    double current_time() { return current_time_; }
    void resetSim() {supervisor_->simulationReset();}
private:
    void initRecv();
    void initSend();

    int time_step_;
    double current_time_;

    Eigen::VectorXd last_q_;

    // webots interface
    webots::Supervisor *supervisor_;
    webots::Node *robot_node_;
    webots::Motor *joint_motor_[12];
    webots::PositionSensor *joint_sensor_[12];
    webots::InertialUnit *imu_;
    webots::Gyro *gyro_;

    std::string robot_name_ = "MiniCheetah";
    std::string imu_name_ = "trunk_imu_inertial";
    std::string gyro_name_ = "trunk_imu_gyro";
    std::vector<std::string> joint_sensor_name_ = {"torso_to_abduct_fl_j_sensor", "abduct_fl_to_thigh_fl_j_sensor", "thigh_fl_to_knee_fl_j_sensor",
                                                   "torso_to_abduct_fr_j_sensor", "abduct_fr_to_thigh_fr_j_sensor", "thigh_fr_to_knee_fr_j_sensor",
                                                   "torso_to_abduct_hl_j_sensor", "abduct_hl_to_thigh_hl_j_sensor", "thigh_hl_to_knee_hl_j_sensor",
                                                   "torso_to_abduct_hr_j_sensor", "abduct_hr_to_thigh_hr_j_sensor", "thigh_hr_to_knee_hr_j_sensor"};
    std::vector<std::string> joint_motor_name_ = {"torso_to_abduct_fl_j", "abduct_fl_to_thigh_fl_j", "thigh_fl_to_knee_fl_j",
                                                  "torso_to_abduct_fr_j", "abduct_fr_to_thigh_fr_j", "thigh_fr_to_knee_fr_j",
                                                  "torso_to_abduct_hl_j", "abduct_hl_to_thigh_hl_j", "thigh_hl_to_knee_hl_j",
                                                  "torso_to_abduct_hr_j", "abduct_hr_to_thigh_hr_j", "thigh_hr_to_knee_hr_j"};
};
