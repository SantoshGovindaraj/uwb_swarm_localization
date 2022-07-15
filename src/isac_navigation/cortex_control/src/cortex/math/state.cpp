/*
 * Copyright (c) 2019-2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#include "cortex/math/state.h"

#include <Eigen/Core>
#include <ros/assert.h>

namespace cortex {
namespace math {

State::State(int n) : state(Eigen::VectorXd::Zero(2 * n)) {}
State::State(const Eigen::VectorXd &x, const Eigen::VectorXd &xd) : State(x.size()) {
  ROS_ASSERT(x.size() == xd.size());
  pos() = x;
  vel() = xd;
}

}  // namespace math
}  // namespace cortex