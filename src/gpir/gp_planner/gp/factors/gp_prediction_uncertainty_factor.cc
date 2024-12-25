#include "gp_planner/gp/factors/gp_prediction_uncertainty_factor.h"

namespace planning {

gtsam::Vector GPPredictionUncertaintyFactor::evaluateError(
    const gtsam::Vector3& x1,
    const gtsam::Vector3& x2,
    boost::optional<gtsam::Matrix&> H1,
    boost::optional<gtsam::Matrix&> H2) const {
  
  // 计算预测误差
  gtsam::Vector3 predicted_x2 = phi_ * x1;
  gtsam::Vector3 error = x2 - predicted_x2;

  // 计算雅可比矩阵
  if (H1) *H1 = -phi_;
  if (H2) *H2 = gtsam::Matrix33::Identity();

  // 应用不确定性缩放
  return error * uncertainty_scale_;
}

gtsam::SharedNoiseModel GPPredictionUncertaintyFactor::ComputeNoiseModel(
    const SceneFeatures& features,
    const gtsam::Matrix3& base_Qc) {
  
  // 基于场景特征计算噪声缩放比例
  double noise_scale = 1.0;
  
  // 交通密度影响
  noise_scale *= (1.0 + features.traffic_density * 0.2);
  
  // 遮挡影响
  noise_scale *= (1.0 + features.occlusion_ratio * 0.3);
  
  // 道路几何影响
  noise_scale *= (1.0 + std::abs(features.curvature) * 0.2);
  
  // 速度影响
  noise_scale *= (1.0 + features.velocity * 0.1);

  // 创建噪声模型
  return gtsam::noiseModel::Gaussian::Covariance(
      base_Qc * noise_scale);
}

double GPPredictionUncertaintyFactor::ComputeUncertaintyScale(
    const SceneFeatures& features) {
  
  // 基础不确定性
  double scale = 1.0;

  // 场景复杂度影响
  scale *= (1.0 + features.scene_complexity * 0.3);
  
  // 交互复杂度影响
  scale *= (1.0 + features.interaction_complexity * 0.2);

  return scale;
}

gtsam::Matrix33 GPPredictionUncertaintyFactor::ComputeTransitionMatrix(
    const double delta) {
  // 使用常速度模型
  gtsam::Matrix33 phi;
  phi << 1, delta, 0.5 * delta * delta,
         0, 1,     delta,
         0, 0,     1;
  return phi;
}

} // namespace planning