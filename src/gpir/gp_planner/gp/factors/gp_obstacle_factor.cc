/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "gp_planner/gp/factors/gp_obstacle_factor.h"

#include "gp_planner/gp/utils/gp_utils.h"

namespace planning {

//该函数用于计算点的雅可比矩阵，用来描述特定点的位置如何影响误差值。
//CheckPointJacobian 函数的实现依赖于路径的曲率等信息，通过对点的位置求导，得到误差对于状态的敏感性。
gtsam::Matrix23 GPObstacleFactor::CheckPointJacobian(const gtsam::Vector3& x,
                                                     const double l) const {
   gtsam::Matrix23 jacobian = gtsam::Matrix23::Zero();

  const double one_minus_kappar_d = 1 - kappa_r_ * x(0); //表示当前节点横向位置和曲率的关系，用于计算该节点在路径上的变化如何影响避障误差。
  const double denominator =
      1 / (one_minus_kappar_d * one_minus_kappar_d + x(1) * x(1));//用于归一化的因子

  jacobian(0, 0) = l * kappa_r_ * x(1) * sin_theta_ * denominator;
  jacobian(0, 1) = -l * one_minus_kappar_d * sin_theta_ * denominator;
  jacobian(1, 0) = 1 - l * kappa_r_ * x(1) * cos_theta_ * denominator;
  jacobian(1, 1) = l * one_minus_kappar_d * cos_theta_ * denominator;

  return jacobian;
}

//evaluateError 函数计算轨迹节点到障碍物的误差，确保轨迹远离障碍物。
//雅可比矩阵的可选引用（boost::optional），表示误差对输入变量的梯度。
gtsam::Vector GPObstacleFactor::evaluateError(
    const gtsam::Vector3& x1, boost::optional<gtsam::Matrix&> H1) const {
  static gtsam::Matrix23 J_x =
      (gtsam::Matrix(2, 3) << 0, 0, 0, 1, 0, 0).finished();

  const double theta = std::atan2(x1(1), 1 - kappa_r_ * x1(0));
  sin_theta_ = std::sin(theta);
  cos_theta_ = std::cos(theta);

  gtsam::Matrix12 J_err1, J_err2;
  //计算轨迹节点的两个参考点 point1 和 point2
  gtsam::Vector2 point1(param_ + ls_ / 2 * cos_theta_,
                        x1(0) + ls_ / 2 * sin_theta_);
  gtsam::Vector2 point2(param_ + ls_ * cos_theta_, x1(0) + ls_ * sin_theta_);
  
  //使用 GPUtils::HingeLoss2 计算这些参考点相对于障碍物的误差 err1 和 err2。
  //HingeLoss2 可以理解为一种度量距离的方法，当轨迹节点离障碍物太近时，返回一个大的误差值，迫使优化器在后续优化中增加节点与障碍物的距离。
  const double err1 = GPUtils::HingeLoss2(point1, *sdf_, epsilon_, J_err1);
  const double err2 = GPUtils::HingeLoss2(point2, *sdf_, epsilon_, J_err2);

  //传入了 H1，则需要计算误差对于输入状态的导数 H1 是一个 2x3 的矩阵（对应 err1 和 err2 对于 x1 的导数）。
  //使用 CheckPointJacobian 计算参考点相对于节点位置的雅可比矩阵，这对于后续优化求解是必不可少的。
  //这些梯度信息帮助优化器在下一步调整节点的位置时，决定如何变化能更有效地减少误差。
  //如果路径节点距离障碍物太近，则梯度的方向会指向远离障碍物的方向，使得优化后的路径能够有效地避开障碍物。
  if (H1) {
    *H1 = gtsam::Matrix23::Zero();
    H1->row(0) = J_err1 * CheckPointJacobian(x1, ls_ / 2);//使用 J_err1 和 J_err2 乘以调用 CheckPointJacobian 返回的结果来生成梯度：
    H1->row(1) = J_err2 * CheckPointJacobian(x1, ls_);
  }
  return gtsam::Vector2(err1, err2);
}
}  // namespace planning
