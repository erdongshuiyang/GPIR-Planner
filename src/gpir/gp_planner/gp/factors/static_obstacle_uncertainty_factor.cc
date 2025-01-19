/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "gp_planner/gp/factors/static_obstacle_uncertainty_factor.h"

namespace planning {

gtsam::Matrix23 StaticObstacleUncertaintyFactor::CheckPointJacobian(
    const gtsam::Vector3& x,
    const double l) const {
  gtsam::Matrix23 jacobian = gtsam::Matrix23::Zero();

  const double one_minus_kappar_d = 1 - kappa_r_ * x(0);
  const double denominator = 
      1 / (one_minus_kappar_d * one_minus_kappar_d + x(1) * x(1));

  jacobian(0, 0) = l * kappa_r_ * x(1) * sin_theta_ * denominator;
  jacobian(0, 1) = -l * one_minus_kappar_d * sin_theta_ * denominator;
  jacobian(1, 0) = 1 - l * kappa_r_ * x(1) * cos_theta_ * denominator;
  jacobian(1, 1) = l * one_minus_kappar_d * cos_theta_ * denominator;

  return jacobian;
}

double StaticObstacleUncertaintyFactor::computeCollisionProbability(
    const gtsam::Vector3& x,
    const gtsam::Vector2& point) const {
    
  // 获取SDF距离和梯度
  Eigen::Vector2d gradient;
  double distance = sdf_->SignedDistance(point, &gradient);
  
  // 避免零梯度
  if(gradient.norm() < 1e-6) {
    gradient = (point - Eigen::Vector2d(x(0), x(1))).normalized();
  }
  
  // 计算马氏距离
  double sigma = std::sqrt(
      gradient.transpose() * uncertainty_.position_covariance * gradient + 
      uncertainty_.length_variance + uncertainty_.width_variance);
  sigma = std::max(sigma, 1e-6);  // 避免除零
                          
  // 使用平滑的碰撞概率函数
  double normalized_distance = (distance - epsilon_) / sigma;
  
  // 限制数值范围
  normalized_distance = std::clamp(normalized_distance, -10.0, 10.0);
  
  if(normalized_distance > 1.0) {
    return kMinProb_;
  } else if(normalized_distance > 0.0) {
    return kMinProb_ + (kMaxProb_ - kMinProb_) * 
           std::pow(1.0 - normalized_distance, 3);
  } else {
    return kMaxProb_ - 0.1 * normalized_distance;  // 线性惩罚
  }
}

gtsam::Vector StaticObstacleUncertaintyFactor::evaluateError(
    const gtsam::Vector3& x1,
    boost::optional<gtsam::Matrix&> H1) const {
    
  const double theta = std::atan2(x1(1), 1 - kappa_r_ * x1(0));
  sin_theta_ = std::sin(theta);
  cos_theta_ = std::cos(theta);

  // 计算前后两个检查点
  gtsam::Vector2 front_point(param_ + ls_ / 2 * cos_theta_,
                            x1(0) + ls_ / 2 * sin_theta_);
  gtsam::Vector2 rear_point(param_ - ls_ / 2 * cos_theta_,
                           x1(0) - ls_ / 2 * sin_theta_);

  // 计算基础碰撞代价
  gtsam::Matrix12 J_front, J_rear;
  double front_cost = GPUtils::HingeLoss2(front_point, *sdf_, epsilon_, 
                                         H1 ? &J_front : nullptr);
  double rear_cost = GPUtils::HingeLoss2(rear_point, *sdf_, epsilon_, 
                                        H1 ? &J_rear : nullptr);
  
  // 计算碰撞概率
  double front_prob = kMinProb_;
  double rear_prob = kMinProb_;
  
  if (front_cost > 0 || rear_cost > 0) {
    front_prob = computeCollisionProbability(x1, front_point);
    rear_prob = computeCollisionProbability(x1, rear_point);
  }
  
  // 平滑概率变化
  front_prob = kSmoothFactor_ * front_prob + 
               (1 - kSmoothFactor_) * kMinProb_;
  rear_prob = kSmoothFactor_ * rear_prob + 
              (1 - kSmoothFactor_) * kMinProb_;

  // 计算梯度
  if (H1) {
    *H1 = gtsam::Matrix23::Zero();
    H1->row(0) = front_prob * J_front * CheckPointJacobian(x1, ls_ / 2);
    H1->row(1) = rear_prob * J_rear * CheckPointJacobian(x1, -ls_ / 2);
  }

  // 返回加权误差
  return (gtsam::Vector(2) << front_cost * front_prob, 
                             rear_cost * rear_prob).finished();
}

}  // namespace planning