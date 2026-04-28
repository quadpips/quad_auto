#pragma once

#include <Eigen/Dense>
#include <go2_wbc/PerceptiveLocomotionOCS2WBC/Task.hpp>

namespace switched_model {

using matrix_qp = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

class SqpSolver {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    scalar_t maxRecalculations_;

    vector_t solveSqp(const Task &weightedTasks, const Task &constraints, int & flag);
};

}  // namespace switched_model
