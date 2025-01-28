#pragma once

#include <string>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/geometry.hpp>

namespace ci_mpc
{
    namespace pin = pinocchio;

    // TODO： 改名成RobotModelHanlder更好
    class RobotHandler
    {
    public:
        RobotHandler(std::string urdf_filename, std::string srdf_filename);
        const pin::Model &model() { return model_; }
        const pin::GeometryModel &geom_model() { return geom_model_; }
        int nq() const { return model_.nq; }
        int nv() const { return model_.nv; }
        
    private:
        pin::Model model_;
        pin::GeometryModel geom_model_;
    };

} // namespace ci_mpc
