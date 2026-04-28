//
// Created by Max on 10/29/24.
//

#pragma once

#include <Eigen/Dense>

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>

namespace mmp {

static constexpr size_t STATE_DIM = 24;
static constexpr size_t CONFIG_DIM = 18;
static constexpr size_t INPUT_DIM = 24;

using scalar_t = ocs2::scalar_t;
using ad_scalar_t = ocs2::ad_scalar_t;
using scalar_array_t = ocs2::scalar_array_t;
using vector_t = ocs2::vector_t;
using vector_array_t = ocs2::vector_array_t;

using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;
using matrix3_t = Eigen::Matrix<scalar_t, 3, 3>;
using quaternion_t = Eigen::Quaternion<scalar_t>;

template <typename T>
using feet_array_t = std::array<T, 4>;
using contact_flag_t = feet_array_t<bool>;

}  // namespace mmp
