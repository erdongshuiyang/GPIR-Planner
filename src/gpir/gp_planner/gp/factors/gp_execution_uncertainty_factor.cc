#include "gp_planner/gp/factors/gp_execution_uncertainty_factor.h"

namespace planning {

GPExecutionUncertaintyFactor::GPExecutionUncertaintyFactor(
    gtsam::Key key1,
    gtsam::Key key2,
    const double delta,
    const SceneFeatures& scene_features,
    const gtsam::Matrix3& base_Qc,
    const ControllerParams& controller_params)
    : NoiseModelFactor2(
          ComputeNoiseModel(scene_features, base_Qc, controller_params),
          key1, key2),
      delta_(delta),
      params_(controller_params) {
  
  // 计算动力学雅可比矩阵
  J_ = ComputeDynamicsJacobian(delta, params_);

  // 基于场景特征计算执行不确定性缩放因子
  execution_scale_ = 1.0;
  // 曲率影响
  execution_scale_ *= (1.0 + std::abs(scene_features.curvature) * 0.3);
  // 速度影响
  execution_scale_ *= (1.0 + scene_features.velocity * 0.2);
  // 路面状况影响
  if (scene_features.is_wet) execution_scale_ *= 1.3;
  if (scene_features.is_snow) execution_scale_ *= 1.5;
}

gtsam::Vector GPExecutionUncertaintyFactor::evaluateError(
    const gtsam::Vector3& x1,
    const gtsam::Vector3& x2,
    boost::optional<gtsam::Matrix&> H1,
    boost::optional<gtsam::Matrix&> H2) const {
  
  // 计算期望的状态变化
  gtsam::Vector3 expected_delta = J_ * x1;
  gtsam::Vector3 actual_delta = x2 - x1;
  gtsam::Vector3 error = actual_delta - expected_delta;

  // 计算雅可比矩阵
  if (H1) *H1 = -J_;
  if (H2) *H2 = gtsam::Matrix33::Identity();

  // 应用执行不确定性缩放
  return error * execution_scale_;
}

gtsam::SharedNoiseModel GPExecutionUncertaintyFactor::ComputeNoiseModel(
    const SceneFeatures& features,
    const gtsam::Matrix3& base_Qc,
    const ControllerParams& params) {
  
  // 基础噪声
  gtsam::Matrix3 Q = base_Qc;

  // 考虑控制延迟
  double delay_factor = 1.0 + 
      (params.lat_delay + params.lon_delay) * 0.5;
  Q *= delay_factor;

  // 考虑控制误差
  double error_factor = 1.0 + 
      std::max(params.lat_error, params.lon_error);
  Q *= error_factor;

  // 考虑场景因素
  if (features.is_turn) Q *= 1.2;       // 转弯场景
  if (features.is_narrow) Q *= 1.3;     // 窄路场景
  if (features.is_crowded) Q *= 1.4;    // 拥挤场景

  return gtsam::noiseModel::Gaussian::Covariance(Q);
}

gtsam::Matrix33 GPExecutionUncertaintyFactor::ComputeDynamicsJacobian(
    const double delta,
    const ControllerParams& params) {
  
  // 使用简化的动力学模型
  gtsam::Matrix33 J = gtsam::Matrix33::Identity();
  
  // 考虑控制延迟的影响
  double effective_delta = delta * 
      (1.0 - (params.lat_delay + params.lon_delay) * 0.5);
  
  // 构建雅可比矩阵
  J(0,1) = effective_delta;
  J(0,2) = 0.5 * effective_delta * effective_delta;
  J(1,2) = effective_delta;

  return J;
}

} // namespace planning