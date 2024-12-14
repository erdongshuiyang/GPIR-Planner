/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "gp_planner/gp/utils/gp_utils.h"

namespace planning {

double GPUtils::HingeLoss(const gtsam::Vector2& point,
                          const SignedDistanceField2D& sdf, const double eps,
                          gtsam::OptionalJacobian<1, 2> H_point) {
  gtsam::Vector2 grad;
  const double signed_distance = sdf.SignedDistance(point, &grad);

  if (signed_distance > eps) {
    if (H_point) *H_point = gtsam::Matrix12::Zero();
    return 0.0;
  } else {
    if (H_point) *H_point = -grad.transpose();
    return eps - signed_distance;
  }
}

double GPUtils::HingeLoss2(const gtsam::Vector2& point,
                           const SignedDistanceField2D& sdf, const double eps,
                           gtsam::OptionalJacobian<1, 2> H_point) {
  gtsam::Vector2 grad; //grad：梯度，用于记录障碍物距离函数在当前点上的方向变化。
  const double signed_distance = sdf.SignedDistance(point, &grad);//通过调用 SignedDistance 函数计算当前节点与最近障碍物之间的距离
  double error = eps - signed_distance;//计算出的误差值，用于衡量当前节点与障碍物的距离相对于阈值 eps 的差距。如果节点距离小于 eps，则 error 为正值，表示需要避开障碍物。

  if (error < 0) { //当 error 为负值时，意味着 signed_distance > eps，即节点与障碍物之间的距离大于阈值，足够安全
    if (H_point) *H_point = gtsam::Matrix12::Zero();//雅可比矩阵为零，因为不需要对节点施加任何影响。
    return 0.0;//此时返回误差为 0.0，意味着当前节点不会因避障而增加代价
  } else if (0 < error && error <= eps) { //当 error 在 0 到 eps 之间时，意味着当前节点离障碍物较近，需要增加代价来远离障碍物。
    if (H_point) (*H_point) = -3 * error * error * grad.transpose(); //用于表示当前节点在沿梯度方向移动时对损失值的影响。负号是为了引导路径远离障碍物。
    return error * error * error;//此时返回误差值为 error^3，即以三次方的形式增加代价。当距离障碍物越近，代价会急剧增加。
  } else if (error > eps) { //当 error > eps 时，意味着路径节点距离障碍物非常近，甚至已经侵入了障碍物的空间
    if (H_point)
      (*H_point) = -(6 * eps * error - 3 * eps * eps) * grad.transpose();//通过计算梯度来表示误差随节点位置变化的关系。
    return 3 * eps * error * error - 3 * eps * eps * error + eps * eps * eps; //以三次函数的形式表示，确保在靠近障碍物的情况下，代价呈现出较高的增长速率，从而强制路径远离障碍物。
  }
}

double GPUtils::HingeKappaLimitLoss(
    const gtsam::Vector3& x, const double kappa_r, const double dkappa_r,
    const double kappa_limit,
    gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H) {
  const double one_minus_kappar_d = 1 - kappa_r * x(0);
  const double one_minus_kappar_d_inv = 1.0 / one_minus_kappar_d;
  const double theta = std::atan2(x(1), one_minus_kappar_d);

  const double tan_theta = x(1) / one_minus_kappar_d;
  const double sin_theta = std::sin(theta);
  const double cos_theta = std::cos(theta);
  const double cos_thete_sqr = cos_theta * cos_theta;

  const double kappa =
      ((x(2) - (dkappa_r * x(0) + kappa_r * x(1)) * tan_theta) * cos_thete_sqr *
           one_minus_kappar_d_inv +
       kappa_r) *
      cos_theta * one_minus_kappar_d_inv;

  if (std::fabs(kappa) < kappa_limit) {
    if (H) *H = gtsam::Matrix13::Zero();
    return 0.0;
  }

  if (H) {
    std::array<double, 2> partial_theta;
    std::array<double, 3> partial_f;

    const double denominator =
        1.0 / (one_minus_kappar_d * one_minus_kappar_d + x(1) * x(1));
    partial_theta[0] = -kappa_r * x(1) * denominator;
    partial_theta[1] = one_minus_kappar_d * denominator;

    const double tmp0 = -3 * cos_thete_sqr * sin_theta;
    const double tmp1 = (dkappa_r * x(0) + kappa_r * x(1)) * cos_theta *
                        (1 - 3 * sin_theta * sin_theta);
    partial_f[0] = (tmp0 - dkappa_r * tmp1) * partial_theta[0] -
                   dkappa_r * sin_theta * cos_thete_sqr;
    partial_f[1] = (tmp0 - kappa_r * tmp1) * partial_theta[1] -
                   kappa_r * sin_theta * cos_thete_sqr;
    partial_f[2] = cos_thete_sqr * cos_theta;

    const double f = (x(2) - (dkappa_r * x(0) + kappa_r * x(1)) * tan_theta) *
                     cos_theta * cos_thete_sqr;

    *H = gtsam::Matrix13::Zero();
    (*H)(0, 0) =
        (partial_f[0] * one_minus_kappar_d +
         2 * kappa_r * f * one_minus_kappar_d) *
            std::pow(one_minus_kappar_d_inv, 4) +
        (-kappa_r * cos_thete_sqr * partial_theta[0] * one_minus_kappar_d +
         kappa_r * kappa_r * cos_theta) *
            one_minus_kappar_d_inv;
    (*H)(0, 1) =
        partial_f[1] * one_minus_kappar_d_inv -
        kappa_r * sin_theta * partial_theta[1] * one_minus_kappar_d_inv;
    (*H)(0, 2) = partial_f[2] * one_minus_kappar_d_inv * one_minus_kappar_d_inv;
  }
  if (kappa > 0) {
    return kappa - kappa_limit;
  } else {
    if (H) *H = -(*H);
    return kappa_limit - kappa;
  }
}

}  // namespace planning
