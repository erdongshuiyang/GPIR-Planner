#include "gp_planner/gp/uncertainty/uncertainty_evaluator.h"

namespace planning {

bool UncertaintyEvaluator::Init(const UncertaintyEvaluatorConfig& config) {
  config_ = config;
  return true;
}

UncertaintyState UncertaintyEvaluator::EvaluateUncertainty(
    const SceneFeatures& features,
    const gtsam::Matrix3& base_prediction_cov,
    const gtsam::Matrix3& base_state_cov,
    const gtsam::Matrix3& base_execution_cov) const {
  
  UncertaintyState state;

  // 评估各维度的不确定性
  EvaluatePredictionUncertainty(
      features, base_prediction_cov,
      &state.prediction_cov, &state.prediction_confidence);

  EvaluateStateUncertainty(
      features, base_state_cov,
      &state.state_cov, &state.state_confidence);

  EvaluateExecutionUncertainty(
      features, base_execution_cov,
      &state.execution_cov, &state.execution_confidence);

  // 计算综合不确定性指标
  ComputeTotalUncertainty(features, &state);

  return state;
}

void UncertaintyEvaluator::EvaluatePredictionUncertainty(
    const SceneFeatures& features,
    const gtsam::Matrix3& base_cov,
    gtsam::Matrix3* cov,
    double* confidence) const {
  
  // 基础不确定性
  *cov = base_cov;
  double scale = 1.0;

  // 交互复杂度影响
  scale *= (1.0 + features.interaction_complexity * 
            config_.interaction_weight);
  
  // 交通密度影响
  scale *= (1.0 + features.traffic_density * 
            config_.traffic_density_weight);
  
  // 特殊场景影响
  if (features.is_intersection) scale *= config_.intersection_factor;
  if (features.is_merge_area) scale *= config_.merge_area_factor;

  // 应用缩放因子
  *cov *= scale;
  
  // 计算可信度
  *confidence = 1.0 / scale;
}

void UncertaintyEvaluator::EvaluateStateUncertainty(
    const SceneFeatures& features,
    const gtsam::Matrix3& base_cov,
    gtsam::Matrix3* cov,
    double* confidence) const {
  
  // 基础不确定性
  *cov = base_cov;
  double scale = 1.0;

  // 传感器状态影响
  scale *= (2.0 - features.gps_quality * config_.gps_weight
                - features.visual_quality * config_.visual_weight
                - features.lidar_quality * config_.lidar_weight);
  
  // 遮挡影响
  scale *= (1.0 + features.occlusion_ratio * config_.occlusion_weight);
  
  // 环境影响
  if (features.is_rain) scale *= config_.rain_factor;
  if (features.is_fog) scale *= config_.fog_factor;
  if (features.is_night) scale *= config_.night_factor;

  // 应用缩放因子
  *cov *= scale;
  
  // 计算可信度
  *confidence = 1.0 / scale;
}

void UncertaintyEvaluator::EvaluateExecutionUncertainty(
    const SceneFeatures& features,
    const gtsam::Matrix3& base_cov,
    gtsam::Matrix3* cov,
    double* confidence) const {
  
  // 基础不确定性
  *cov = base_cov;
  double scale = 1.0;

  // 道路几何特征影响
  scale *= (1.0 + std::abs(features.curvature) * 
            config_.curvature_weight);
  
  // 车道宽度影响
  if (features.is_narrow) {
    scale *= config_.narrow_lane_factor;
  }
  
  // 环境状况影响
  scale *= (2.0 - features.visibility) * config_.visibility_weight;

  // 应用缩放因子
  *cov *= scale;
  
  // 计算可信度
  *confidence = 1.0 / scale;
}

void UncertaintyEvaluator::ComputeTotalUncertainty(
    const SceneFeatures& features,
    UncertaintyState* state) const {
  
  // 计算总体不确定性水平
  state->total_uncertainty = 
      state->prediction_confidence * config_.prediction_weight +
      state->state_confidence * config_.state_weight +
      state->execution_confidence * config_.execution_weight;

  // 判断系统是否可靠
  state->is_reliable = 
      state->total_uncertainty < config_.reliability_threshold;
}

} // namespace planning