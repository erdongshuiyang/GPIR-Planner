#include "gp_planner/gp/factors/control_uncertainty_factor.h"

namespace planning {

ControlUncertaintyFactor::ControlUncertaintyFactor(
    gtsam::Key key1,
    gtsam::Key key2,
    const ControlUncertaintyModel& model,
    double dt,
    double weight)
    : NoiseModelFactor2<gtsam::Vector3, gtsam::Vector3>(
          gtsam::noiseModel::Isotropic::Sigma(3, 1.0),
          key1,
          key2),
      model_(model),
      dt_(dt),
      weight_(weight) {
  CHECK_GT(dt_, 0.0) << "Time step must be positive";
  CHECK(model_.IsValid()) << "Control uncertainty model is invalid";
}

gtsam::Vector ControlUncertaintyFactor::evaluateError(
    const gtsam::Vector3& x1,
    const gtsam::Vector3& x2,
    boost::optional<gtsam::Matrix&> H1,
    boost::optional<gtsam::Matrix&> H2) const {
  
  // 预测下一状态
  gtsam::Vector3 x2_pred;
  gtsam::Matrix jacobian;
  predictNextState(x1, &x2_pred, &jacobian);

  // 计算误差
  gtsam::Vector3 error = x2 - x2_pred;

  // 从Vector3构造State
  common::State current_state;
  current_state.position.x() = x1(0);  
  current_state.position.y() = x1(1);  
  current_state.heading = x1(2);      

  // 修改为Matrix3d类型
  Eigen::Matrix3d state_covariance = Eigen::Matrix3d::Identity() * kMinCovariance;
  model_.PredictUncertainty(current_state, dt_, &state_covariance);

  // 计算代价并使用它
  double mahalanobis_cost = computeCost(error, state_covariance);
  gtsam::Vector3 weighted_error = error * std::sqrt(mahalanobis_cost);

  // 计算雅可比矩阵
  if (H1) {
    *H1 = -weight_ * jacobian * std::sqrt(mahalanobis_cost);
  }
  if (H2) {
    *H2 = weight_ * Eigen::Matrix3d::Identity() * std::sqrt(mahalanobis_cost);
  }

  return weighted_error * weight_;
}

void ControlUncertaintyFactor::predictNextState(
    const gtsam::Vector3& x1,
    gtsam::Vector3* x2_pred,
    gtsam::Matrix* jacobian) const {
  
  CHECK_NOTNULL(x2_pred);
  const auto& sde_params = model_.GetSDEParams();

  // 直接获取速度和航向角
  const double v = x1(1);
  const double theta = x1(2);

  // 线性化点的系统矩阵
  Eigen::Matrix3d A = sde_params.A;
  A(0,2) = -v * sin(theta);
  A(1,2) = v * cos(theta);

  // 离散化
  Eigen::Matrix3d Ad = Eigen::Matrix3d::Identity() + A * dt_;

  // 预测下一状态
  *x2_pred = Ad * x1;

  if (jacobian) {
    *jacobian = Ad;
  }
}

double ControlUncertaintyFactor::computeCost(
    const gtsam::Vector3& error,
    const Eigen::Matrix3d& covariance) const {  // 改为Matrix3d
  
  // 添加数值保护
  Eigen::Matrix3d cov = covariance;
  cov.diagonal().array() += kMinCovariance;

  // 计算马氏距离
  double cost = error.transpose() * cov.inverse() * error;

  // 限制最大代价
  return std::min(cost, kMaxCost);
}

} // namespace planning