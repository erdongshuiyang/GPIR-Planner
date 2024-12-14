/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "gp_planner/gp/factors/gp_prior_factor.h"

namespace planning {

using gtsam::Matrix;

//evaluateError 函数的目标是返回因子的误差
gtsam::Vector GPPriorFactor::evaluateError(
    const gtsam::Vector3& x1, const gtsam::Vector3& x2,
    boost::optional<gtsam::Matrix&> H1,
    boost::optional<gtsam::Matrix&> H2) const {
  //这意味着对 x1 的偏导数是状态转移矩阵 phi_，对 x2 的偏导数是负单位矩阵。雅可比矩阵是用于梯度计算的，用于优化器快速迭代逼近最优解。
  if (H1) *H1 = phi_;
  if (H2) *H2 = -gtsam::Matrix33::Identity();
  return phi_ * x1 - x2;//这个公式意味着预测的下一状态 (phi_ * x1) 与实际下一状态 (x2) 之间的差异，即误差。
  //在实际求解中，这个误差用于构建目标函数，优化器通过最小化所有因子的误差来求解最优路径。
}

}  // namespace planning
