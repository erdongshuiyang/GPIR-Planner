// static_obstacle_uncertainty_factor.cc
#include "gp_planner/gp/factors/static_obstacle_uncertainty_factor.h"

#include <cmath>

namespace planning {

StaticObstacleUncertaintyFactor::StaticObstacleUncertaintyFactor(
    gtsam::Key key,
    const std::shared_ptr<SignedDistanceField2D>& sdf,
    const PerceptionUncertainty& uncertainty,
    const double cost,
    const double epsilon,
    const double param,
    const double kappa_r)
    // 改为 2 维的噪声模型，因为我们返回 2 维的误差向量
    : NoiseModelFactor1(gtsam::noiseModel::Isotropic::Sigma(2, cost), key),
      sdf_(sdf),
      uncertainty_(uncertainty),
      epsilon_(epsilon),
      param_(param),
      kappa_r_(kappa_r) {}

gtsam::Vector StaticObstacleUncertaintyFactor::evaluateError(
    const gtsam::Vector3& x1,
    boost::optional<gtsam::Matrix&> H1) const {
    
  // 计算Frenet坐标系下的航向角
  const double theta = std::atan2(x1(1), 1 - kappa_r_ * x1(0));
  cos_theta_ = std::cos(theta);
  sin_theta_ = std::sin(theta);

  // 添加数值保护
  constexpr double kMaxProb = 0.95;

  // 计算车辆前后两个特征点
  gtsam::Vector2 front_point(param_ + ls_ / 2 * cos_theta_,
                            x1(0) + ls_ / 2 * sin_theta_);
  gtsam::Vector2 rear_point(param_ - ls_ / 2 * cos_theta_,
                           x1(0) - ls_ / 2 * sin_theta_);

  // 计算考虑不确定性的碰撞概率
  // 限制概率范围
  double front_prob = std::min(computeCollisionProbability(x1, front_point), kMaxProb);
  double rear_prob = std::min(computeCollisionProbability(x1, rear_point), kMaxProb);
  // LOG(INFO) << "front_prob:"<<front_prob;
  // LOG(INFO) << "rear_prob:"<<rear_prob;


  // 取最大碰撞概率作为代价
  double max_prob = std::max(front_prob, rear_prob);
  
  // 转换为代价值
  double cost_front = -std::log(std::max(1.0 - front_prob, kMinProbability_));
  double cost_rear = -std::log(std::max(1.0 - rear_prob, kMinProbability_));
  // LOG(INFO) << "cost_front:"<<cost_front;
  // LOG(INFO) << "cost_rear:"<<cost_rear;
 
 

  // 如果需要计算雅可比矩阵
  if (H1) {
    // 修改为2x3的雅可比矩阵
    *H1 = gtsam::Matrix::Zero(2, 3);
    // 计算前后点的梯度
    gtsam::Matrix13 front_jacobian = computeCollisionProbJacobian(x1);
    gtsam::Matrix13 rear_jacobian = computeCollisionProbJacobian(x1);

    // 添加数值保护
    double front_denom = std::max(1.0 - front_prob, kMinProbability_);
    double rear_denom = std::max(1.0 - rear_prob, kMinProbability_);

    // 填充雅可比矩阵
    H1->row(0) = front_jacobian * (1.0 / (1.0 - front_prob));
    H1->row(1) = rear_jacobian * (1.0 / (1.0 - rear_prob));
  }

  // 返回2维误差向量
  return (gtsam::Vector(2) << cost_front, cost_rear).finished();
}

double StaticObstacleUncertaintyFactor::computeCollisionProbability(
    const gtsam::Vector3& x,
    const gtsam::Vector2& point) const {

  // LOG(INFO) << "=== Computing collision probability ===";
  // LOG(INFO) << "Query point: " << point.transpose();

  Eigen::Vector2d gradient;
  double distance = sdf_->SignedDistance(point, &gradient);

  // LOG(INFO) << "SDF distance: " << distance;
  // LOG(INFO) << "SDF gradient: " << gradient.transpose();
  

   // 添加梯度保护
  constexpr double kMinGradNorm = 1e-6;
  // if (gradient.norm() < kMinGradNorm) {
  //   return distance < epsilon_ ? 0.999 : 0.001;
  // }
    // 添加梯度保护
  if (gradient.norm() < 1e-6) {
    // 如果梯度太小，使用到查询点的方向作为梯度
    Eigen::Vector2d direction = point - Eigen::Vector2d(x[0], x[1]);
    if (direction.norm() > 1e-6) {
      gradient = direction.normalized();
    } else {
      gradient << 1.0, 0.0;  // 默认方向
    }
  }

  // // 归一化梯度
  // gradient.normalize();

  // 计算总体不确定性
  // 限制协方差最大值
  constexpr double kMaxVariance = 4.0;
  Eigen::Matrix2d total_cov = uncertainty_.position_covariance;
  total_cov = total_cov.cwiseMin(Eigen::Matrix2d::Identity() * kMaxVariance);

  double size_variance = std::min(
      uncertainty_.length_variance + uncertainty_.width_variance, 
      kMaxVariance);
  
  // 计算梯度方向上的不确定性
  double sigma = std::sqrt(gradient.transpose() * total_cov * gradient + size_variance);
  // LOG(INFO) << "Origin uncertainty: " << sigma;
  // LOG(INFO) << "kMinGradNorm: " << kMinGradNorm;
  // sigma = std::max(sigma, kMinGradNorm);
  // LOG(INFO) << "Combined uncertainty: " << sigma;

  // 使用 3σ 规则计算安全裕度
  double safety_distance = kSafetyScaling_ * sigma;
  
  // 计算归一化距离
  double normalized_distance = (distance - epsilon_ - safety_distance) / sigma;

  // 限制normalized_distance范围
  normalized_distance = std::clamp(normalized_distance, -10.0, 10.0);
  
  // 使用误差函数计算碰撞概率

  // 打印最终概率
  double prob = 0.5 * (1.0 + std::erf(-normalized_distance / std::sqrt(2.0)));
  // LOG(INFO) << "Collision probability: " << prob;
  return prob;
}

gtsam::Matrix23 StaticObstacleUncertaintyFactor::computeCheckPointJacobian(
    const gtsam::Vector3& x,
    const double l) const {
    
  gtsam::Matrix23 jacobian = gtsam::Matrix23::Zero();
  
  const double one_minus_kappar_d = 1 - kappa_r_ * x(0);
  const double denominator = 1 / (one_minus_kappar_d * one_minus_kappar_d + x(1) * x(1));

  jacobian(0, 0) = l * kappa_r_ * x(1) * sin_theta_ * denominator;
  jacobian(0, 1) = -l * one_minus_kappar_d * sin_theta_ * denominator;
  jacobian(1, 0) = 1 - l * kappa_r_ * x(1) * cos_theta_ * denominator;
  jacobian(1, 1) = l * one_minus_kappar_d * cos_theta_ * denominator;

  return jacobian;
}

gtsam::Matrix13 StaticObstacleUncertaintyFactor::computeCollisionProbJacobian(
    const gtsam::Vector3& x) const {
    
  static constexpr double delta = 1e-6;
  gtsam::Matrix13 H = gtsam::Matrix13::Zero();
  
  // 对每个维度计算偏导数
  for (int i = 0; i < 3; ++i) {
    gtsam::Vector3 x_plus = x;
    gtsam::Vector3 x_minus = x;
    x_plus(i) += delta;
    x_minus(i) -= delta;
    
    // 计算前向差分
    const double theta_plus = std::atan2(x_plus(1), 1 - kappa_r_ * x_plus(0));
    gtsam::Vector2 point_plus(
        param_ + ls_ / 2 * std::cos(theta_plus),
        x_plus(0) + ls_ / 2 * std::sin(theta_plus));
    double prob_plus = computeCollisionProbability(x_plus, point_plus);
    
    // 计算后向差分
    const double theta_minus = std::atan2(x_minus(1), 1 - kappa_r_ * x_minus(0));
    gtsam::Vector2 point_minus(
        param_ + ls_ / 2 * std::cos(theta_minus),
        x_minus(0) + ls_ / 2 * std::sin(theta_minus));
    double prob_minus = computeCollisionProbability(x_minus, point_minus);
    
    // 中心差分
    H(0, i) = (prob_plus - prob_minus) / (2.0 * delta);
  }
  
  return H;
}

} // namespace planning