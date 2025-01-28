#include "robot_handler.hpp"

namespace ci_mpc
{
    RobotHandler::RobotHandler(std::string urdf_filename, std::string srdf_filename)
    {
        pin::urdf::buildModel(urdf_filename, model_);
        pin::urdf::buildGeom(model_, urdf_filename, pinocchio::COLLISION, geom_model_);
        geom_model_.addAllCollisionPairs();
        pin::srdf::removeCollisionPairs(model_, geom_model_, srdf_filename);
    }

} // namespace ci_mpc
