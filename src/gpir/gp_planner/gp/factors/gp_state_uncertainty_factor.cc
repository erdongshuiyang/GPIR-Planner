#include "gp_planner/gp/factors/gp_state_uncertainty_factor.h"

namespace planning {

GPStateUncertaintyFactor::GPStateUncertaintyFactor(
    gtsam::Key key,
    const gtsam::Vector3& measurement, 
    const SceneFeatures& scene_features,
    const gtsam::Matrix3& base_R)
    : NoiseModelFactor1(
          ComputeNoiseModel(scene_features, base_R),
          key),
      measurement_(measurement) {
  // 初始化测量矩阵
  H_ = ComputeMeasurementMatrix(scene_features);
  
  // 计算测量可信度
  measurement_confidence_ = 
      ComputeMeasurementConfidence(scene_features);
}

gtsam::Vector GPStateUncertaintyFactor::evaluateError(
    const gtsam::Vector3& x,
    boost::optional<gtsam::Matrix&> H) const {
  
  // 计算测量残差
  gtsam::Vector3 error = H_ * (x - measurement_);
  
  if (H) {
    *H = H_;
  }
  
  // 应用测量可信度加权
  return error * measurement_confidence_;
}

gtsam::SharedNoiseModel GPStateUncertaintyFactor::ComputeNoiseModel(
    const SceneFeatures& features,
    const gtsam::Matrix3& base_R) {
  double scale = 1.0;
  
  // GPS信号质量影响
  scale *= (1.0 + (1.0 - features.gps_quality) * 0.5);
  
  // 视觉特征质量影响
  scale *= (1.0 + (1.0 - features.visual_quality) * 0.3);
  
  // 天气影响
  if (features.is_rain) scale *= 1.2;
  if (features.is_fog) scale *= 1.5;
  
  return gtsam::noiseModel::Gaussian::Covariance(base_R * scale);
}

gtsam::Matrix33 GPStateUncertaintyFactor::ComputeMeasurementMatrix(
    const SceneFeatures& features) {
  gtsam::Matrix33 H = gtsam::Matrix33::Identity();
  
  // 根据测量质量调整权重
  if (features.speed_measure_quality < 0.5) {
    H(1,1) *= 0.5;
  }
  
  return H;
}

double GPStateUncertaintyFactor::ComputeMeasurementConfidence(
    const SceneFeatures& features) {
  double confidence = 1.0;
  
  // 传感器状态影响
  confidence *= features.sensor_health;
  
  // 环境影响
  confidence *= (1.0 - features.occlusion_ratio * 0.2);
  confidence *= (1.0 - features.interference_level * 0.3);
  
  return std::max(0.1, std::min(1.0, confidence));
}

} // namespace planning