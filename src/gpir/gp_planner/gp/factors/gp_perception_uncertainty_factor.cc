#include "gp_planner/gp/factors/gp_perception_uncertainty_factor.h"

#include <cmath>

namespace planning {

GPPerceptionUncertaintyFactor::GPPerceptionUncertaintyFactor(
    gtsam::Key key,
    const std::shared_ptr<SignedDistanceField2D>& sdf,
    const PerceptionUncertainty& uncertainty,
    double weight,
    double epsilon,
    double s,
    double kappa_r)
    : NoiseModelFactor1(gtsam::noiseModel::Isotropic::Sigma(1, weight), key),
      sdf_(sdf),
      uncertainty_(uncertainty),
      epsilon_(epsilon),
      s_(s),
      kappa_r_(kappa_r) {}

gtsam::Vector GPPerceptionUncertaintyFactor::evaluateError(
    const gtsam::Vector3& x,
    boost::optional<gtsam::Matrix&> H) const {
  
  // 计算考虑感知不确定性的碰撞概率
  double collision_prob = computeCollisionProbability(x);
  
  // 将碰撞概率转换为代价值 (使用对数变换保证数值稳定性)
  constexpr double kMinProb = 1e-6;  // 避免取对数时出现数值问题
  double cost = -std::log(std::max(1.0 - collision_prob, kMinProb));
  
  // 如果需要计算雅可比矩阵
  if (H) {
    *H = computeJacobian(x);
  }
  
  // 返回1维的代价向量
  return (gtsam::Vector(1) << cost).finished();
}

double GPPerceptionUncertaintyFactor::computeCollisionProbability(
    const gtsam::Vector3& x) const {
  
  // 计算Frenet坐标系下的航向角
  const double theta = std::atan2(x(1), 1 - kappa_r_ * x(0));
  cos_theta_ = std::cos(theta);
  sin_theta_ = std::sin(theta);
  
  // 计算车辆前后两个特征点的位置
  gtsam::Vector2 front_point(s_ + ls_ / 2 * cos_theta_, 
                            x(0) + ls_ / 2 * sin_theta_);
  gtsam::Vector2 rear_point(s_ - ls_ / 2 * cos_theta_, 
                           x(0) - ls_ / 2 * sin_theta_);

  // 获取SDF距离和梯度信息
  Eigen::Vector2d front_grad, rear_grad;
  double front_dist = sdf_->SignedDistance(front_point, &front_grad);
  double rear_dist = sdf_->SignedDistance(rear_point, &rear_grad);

  // 计算总体不确定性协方差
  Eigen::Matrix2d total_cov = uncertainty_.position_covariance;
  
  // 考虑航向角不确定性导致的位置不确定性
  if (uncertainty_.heading_variance > 0) {
    // 近似计算由航向角不确定性引起的位置不确定性
    double pos_var_from_heading = ls_ * ls_ * uncertainty_.heading_variance / 4.0;
    total_cov(0,0) += pos_var_from_heading;
    total_cov(1,1) += pos_var_from_heading;
  }

  // 计算前后点的马氏距离
  double front_sigma = std::sqrt(front_grad.transpose() * total_cov * front_grad);
  double rear_sigma = std::sqrt(rear_grad.transpose() * total_cov * rear_grad);

  // 使用误差函数计算碰撞概率
  const double sqrt_2 = std::sqrt(2.0);
  double front_prob = 0.5 * (1 + std::erf((epsilon_ - front_dist)/(sqrt_2 * front_sigma)));
  double rear_prob = 0.5 * (1 + std::erf((epsilon_ - rear_dist)/(sqrt_2 * rear_sigma)));

  // 返回最大碰撞概率
  return std::max(front_prob, rear_prob);
}

gtsam::Matrix23 GPPerceptionUncertaintyFactor::computeCheckPointJacobian(
    const gtsam::Vector3& x, double l) const {
    
  gtsam::Matrix23 jacobian = gtsam::Matrix23::Zero();
  
  const double one_minus_kappar_d = 1 - kappa_r_ * x(0);
  const double denominator = 1 / (one_minus_kappar_d * one_minus_kappar_d + x(1) * x(1));

  // 计算特征点位置对状态的雅可比矩阵
  jacobian(0, 0) = l * kappa_r_ * x(1) * sin_theta_ * denominator;
  jacobian(0, 1) = -l * one_minus_kappar_d * sin_theta_ * denominator;
  jacobian(1, 0) = 1 - l * kappa_r_ * x(1) * cos_theta_ * denominator;
  jacobian(1, 1) = l * one_minus_kappar_d * cos_theta_ * denominator;

  return jacobian;
}

gtsam::Matrix GPPerceptionUncertaintyFactor::computeJacobian(
    const gtsam::Vector3& x) const {
  
  // 使用数值差分计算雅可比矩阵
  constexpr double delta = 1e-6;
  gtsam::Matrix H(1, 3);  // 1x3的雅可比矩阵
  
  // 对每个维度计算偏导数
  for (int i = 0; i < 3; ++i) {
    gtsam::Vector3 x_plus = x;
    gtsam::Vector3 x_minus = x;
    x_plus(i) += delta;
    x_minus(i) -= delta;
    
    // 计算前向和后向的碰撞概率
    double prob_plus = computeCollisionProbability(x_plus);
    double prob_minus = computeCollisionProbability(x_minus);
    
    // 使用中心差分计算偏导数
    const double prob = computeCollisionProbability(x);
    constexpr double kMinProb = 1e-6;
    
    double cost_plus = -std::log(std::max(1.0 - prob_plus, kMinProb));
    double cost_minus = -std::log(std::max(1.0 - prob_minus, kMinProb));
    
    // 计算代价对状态的偏导数
    H(0,i) = (cost_plus - cost_minus) / (2.0 * delta);
  }
  
  return H;
}

}  // namespace planning