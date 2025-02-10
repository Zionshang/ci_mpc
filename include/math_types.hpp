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
    using Vector12d = Eigen::Matrix<double, 12, 1>;
    using Matrix34d = Eigen::Matrix<double, 3, 4>;
    
    // convert Vec34 to Vec12
    inline Matrix34d vec12ToVec34(const Vector12d &vec12)
    {
        Matrix34d mat34;
        for (int i = 0; i < 4; i++)
        {
            mat34.col(i) = vec12.segment(3 * i, 3);
        }
        return mat34;
    }

} // namespace ci_mpc
