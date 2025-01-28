#pragma once

#include <Eigen/Core>

namespace ci_mpc
{
    using Eigen::MatrixXd;
    using Eigen::VectorXd;
    using MatrixRef = Eigen::Ref<MatrixXd>;
    using VectorRef = Eigen::Ref<VectorXd>;
    using ConstMatrixRef = Eigen::Ref<const MatrixXd>;
    using ConstVectorRef = Eigen::Ref<const VectorXd>;

    using Vector3d = Eigen::Matrix<double, 3, 1>;
    using Vector6d = Eigen::Matrix<double, 6, 1>;
    using Vector7d = Eigen::Matrix<double, 7, 1>;
} // namespace ci_mpc
