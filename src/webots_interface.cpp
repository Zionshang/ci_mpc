#include "webots_interface.hpp"

WebotsInterface::WebotsInterface()
{
    supervisor_ = new webots::Supervisor();
    time_step_ = (int)supervisor_->getBasicTimeStep();
    std::cout << "timeStep in simulation is :" << time_step_ << std::endl;

    initRecv();
    initSend();

    last_q_.resize(joint_sensor_name_.size());
    // last_q_.setZero();
    last_q_ << 0.0, -1.3, 2.4,
        0.0, -1.3, 2.4,
        0.0, -1.3, 2.4,
        0.0, -1.3, 2.4;
}

WebotsInterface::~WebotsInterface()
{
    delete supervisor_;
}

void WebotsInterface::recvState(Eigen::VectorXd &state_vector)
{
    current_time_ = supervisor_->getTime();
    Eigen::VectorXd q(19), v(18);

    // sensor data
    Eigen::Vector4d quaternion = Eigen::Map<const Eigen::Vector4d>(imu_->getQuaternion());    // x,y,z,w
    Eigen::Vector3d angular_vel_B = Eigen::Map<const Eigen::Vector3d>(gyro_->getValues());    // expressed in BODY frame
    Eigen::Vector3d robotPos = Eigen::Map<const Eigen::Vector3d>(robot_node_->getPosition()); // expressed in WORLD frame
    Eigen::Vector3d robotVel = Eigen::Map<const Eigen::Vector3d>(robot_node_->getVelocity()); // expressed in WORLD frame
    Eigen::Vector3d robotVel_B = quat2RotMat(quaternion).transpose() * robotVel;              // expressed in BODY frame

    q.head(6) << robotPos, quaternion;
    v.head(6) << robotVel_B, angular_vel_B;

    for (int i = 0; i < 12; i++)
    {
        q(7 + i) = joint_sensor_[i]->getValue();
        v(6 + i) = (q(7 + i) - last_q_(i)) / double(time_step_) * 1000;
        last_q_(i) = q(7 + i);
    }

    state_vector << q, v;
}

void WebotsInterface::sendCmd(const Eigen::VectorXd &tau)
{
    for (int i = 0; i < 12; i++)
    {
        joint_motor_[i]->setTorque(tau(i));
    }
}

bool WebotsInterface::isRunning()
{
    if (supervisor_->step(time_step_) != -1)
        return true;
    else
        return false;
}

void WebotsInterface::initRecv()
{
    // supervisor init
    robot_node_ = supervisor_->getFromDef(robot_name_);
    if (robot_node_ == NULL)
    {
        printf("error supervisor");
        exit(1);
    }

    // sensor init
    imu_ = supervisor_->getInertialUnit(imu_name_);
    imu_->enable(time_step_);
    gyro_ = supervisor_->getGyro(gyro_name_);
    gyro_->enable(time_step_);
    for (int i = 0; i < 12; i++)
    {
        joint_sensor_[i] = supervisor_->getPositionSensor(joint_sensor_name_[i]);
        joint_sensor_[i]->enable(time_step_);
    }
}

void WebotsInterface::initSend()
{
    for (int i = 0; i < 12; i++)
        joint_motor_[i] = supervisor_->getMotor(joint_motor_name_[i]);
}