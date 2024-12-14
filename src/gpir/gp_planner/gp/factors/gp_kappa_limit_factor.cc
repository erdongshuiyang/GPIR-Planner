/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "gp_planner/gp/factors/gp_kappa_limit_factor.h"

#include "gp_planner/gp/utils/curvature_utils.h"
#include "gp_planner/gp/utils/gp_utils.h"

namespace planning {

//它计算了当前节点状态 x 的曲率误差，以及误差的雅可比矩阵。
gtsam::Vector GPKappaLimitFactor::evaluateError(
    const gtsam::Vector3& x, boost::optional<gtsam::Matrix&> H) const {
  double gradient = 0.0, kappa = 0.0;
  if (H) {
    kappa = CurvatureUtils::GetKappaAndJacobian(x, kappa_r_, dkappa_r_, H); //GetKappaAndJacobian 函数还可以计算曲率的雅可比矩阵 H，用于表示曲率对状态的变化率。如果传入 H，则会计算雅可比矩阵，以帮助优化器在路径规划中更好地理解如何调整节点状态以降低误差。
  } else {
    kappa = CurvatureUtils::GetKappaAndJacobian(x, kappa_r_, dkappa_r_);
  }
  double error = penalty_.EvaluateHinge(kappa, &gradient); //调用了 PenaltyFunction::EvaluateHinge 函数来计算误差。该函数根据曲率是否超过了限制来决定误差的值。
                                                           //如果曲率超过了 kappa_limit_，则误差为曲率超出部分的线性或非线性代价，并返回对应的梯度 gradient，用于进一步的雅可比矩阵计算。
  if (H) (*H) *= gradient; //将曲率对输入状态的梯度和误差对曲率的梯度结合起来，得到完整的误差对节点状态的雅可比矩阵。
  return gtsam::Vector1(error);
}
}  // namespace planning
