#pragma once
#include <aligator/modelling/dynamics/ode-abstract.hpp>
#include <proxsuite-nlp/modelling/spaces/multibody.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/collision/collision.hpp>
#include <pinocchio/algorithm/aba.hpp>

#include "contact_parameter.hpp"
#include "math_types.hpp"

namespace ci_mpc
{
    namespace pin = pinocchio;
    using ODEAbstract = aligator::dynamics::ODEAbstractTpl<double>;
    using ODEData = aligator::dynamics::ContinuousDynamicsDataTpl<double>;
    using MultibodyPhaseSpace = proxsuite::nlp::MultibodyPhaseSpace<double>;

    struct CompliantContactData;

    struct CompliantContactDynamics : ODEAbstract
    {
        using ODEData = aligator::dynamics::ContinuousDynamicsDataTpl<double>;

        CompliantContactDynamics(const MultibodyPhaseSpace &space, const MatrixXd &actuation,
                                 const pin::GeometryModel &geom_model,
                                 const ContactParameter &param);

        const MultibodyPhaseSpace &space() const { return space_; }

        void forward(const ConstVectorRef &x, const ConstVectorRef &u,
                     ODEData &data) const override;

        void dForward(const ConstVectorRef &x, const ConstVectorRef &u,
                      ODEData &data) const override;

        void CalcContactForceContribution(const pin::Model &rmodel, const pin::Data &rdata,
                                          const pin::GeometryModel &geom_model, pin::GeometryData &geom_data,
                                          pin::container::aligned_vector<pin::Force> &f_ext,
                                          CompliantContactData &contact_data) const;

        std::shared_ptr<ODEData> createData() const override;

        MultibodyPhaseSpace space_;
        MatrixXd actuation_matrix_;
        pin::GeometryModel geom_model_;
        ContactParameter contact_param_;
    };

    struct CompliantContactData : ODEData
    {
        CompliantContactData(const CompliantContactDynamics &dynamics);

        VectorXd tau_;
        pin::Data pin_data_;
        pin::GeometryModel geom_model_; // todo: 没有发生变化的数据是否需要存储？
        pin::GeometryData geom_data_;
        ContactParameter contact_param_; // todo: 没有发生变化的数据是否需要存储？
        pin::container::aligned_vector<pin::Force> f_ext_;
        std::vector<Vector3d> contact_forces_;
    };

} // namespace ci_mpc
