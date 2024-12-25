// gp_planner/gp/uncertainty/uncertainty_propagator.cc
#include "gp_planner/gp/uncertainty/uncertainty_propagator.h"

namespace planning {

bool UncertaintyPropagator::Init(const UncertaintyPropagatorConfig& config) {
  config_ = config;
  return true;
}

UncertaintyState UncertaintyPropagator::PropagateUncertainty(
    const UncertaintyState& current_state,
    const SceneFeatures& features,
    double dt) const {
  
  UncertaintyState next_state;

  // 计算状态转移矩阵
  gtsam::Matrix33 F = ComputeTransitionMatrix(dt);
  
  // 计算过程噪声
  gtsam::Matrix3 Q = ComputeProcessNoise(features, dt);
  
  // 计算传播增益
  double gain = ComputePropagationGain(features);

  // 预测不确定性传播
  next_state.prediction_cov = 
      F * current_state.prediction_cov * F.transpose() + gain * Q;
  next_state.prediction_confidence = 
      current_state.prediction_confidence / (1.0 + gain);

  // 状态估计不确定性传播
  next_state.state_cov = 
      F * current_state.state_cov * F.transpose() + gain * Q;
  next_state.state_confidence = 
      current_state.state_confidence / (1.0 + gain);

  // 执行不确定性传播
  next_state.execution_cov = 
      F * current_state.execution_cov * F.transpose() + gain * Q;
  next_state.execution_confidence = 
      current_state.execution_confidence / (1.0 + gain);

  // 计算总体不确定性
  next_state.total_uncertainty = 
      next_state.prediction_confidence * config_.prediction_weight +
      next_state.state_confidence * config_.state_weight +
      next_state.execution_confidence * config_.execution_weight;

  next_state.is_reliable = 
      next_state.total_uncertainty < config_.reliability_threshold;

  return next_state;
}

std::vector<UncertaintyState> UncertaintyPropagator::PropagateAlongPath(
    const ReferenceLine& reference_line,
    const UncertaintyState& init_state,
    const UncertaintyEvaluator& evaluator) const {
  
  std::vector<UncertaintyState> uncertainty_states;
  uncertainty_states.push_back(init_state);

  double step = config_.propagation_step;
  UncertaintyState current_state = init_state;

  // 沿参考线传播不确定性
  for (double s = step; s < reference_line.length(); s += step) {
    // 获取当前位置的场景特征
    SceneFeatures features = 
        reference_line.GetSceneFeatures(s);
    
    // 传播不确定性
    double dt = step / config_.reference_speed;
    current_state = PropagateUncertainty(
        current_state, features, dt);
    
    uncertainty_states.push_back(current_state);
  }

  return uncertainty_states;
}

UncertaintyState UncertaintyPropagator::FuseUncertaintyStates(
    const std::vector<UncertaintyState>& states,
    const std::vector<double>& weights) const {
  
  CHECK_EQ(states.size(), weights.size());
  
  UncertaintyState fused_state;
  double total_weight = 0.0;

  // 加权融合各不确定性状态
  for (size_t i = 0; i < states.size(); ++i) {
    const auto& state = states[i];
    double w = weights[i];
    
    fused_state.prediction_cov += w * state.prediction_cov;
    fused_state.state_cov += w * state.state_cov;
    fused_state.execution_cov += w * state.execution_cov;
    
    fused_state.prediction_confidence += w * state.prediction_confidence;
    fused_state.state_confidence += w * state.state_confidence;
    fused_state.execution_confidence += w * state.execution_confidence;
    
    total_weight += w;
  }

  // 归一化
  if (total_weight > 0) {
    fused_state.prediction_cov /= total_weight;
    fused_state.state_cov /= total_weight;
    fused_state.execution_cov /= total_weight;
    
    fused_state.prediction_confidence /= total_weight;
    fused_state.state_confidence /= total_weight;
    fused_state.execution_confidence /= total_weight;
  }

  // 计算总体不确定性
  fused_state.total_uncertainty = 
      fused_state.prediction_confidence * config_.prediction_weight +
      fused_state.state_confidence * config_.state_weight +
      fused_state.execution_confidence * config_.execution_weight;

  fused_state.is_reliable = 
      fused_state.total_uncertainty < config_.reliability_threshold;

  return fused_state;
}

gtsam::Matrix33 UncertaintyPropagator::ComputeTransitionMatrix(
    double dt) const {
  // 使用线性系统模型
  gtsam::Matrix33 F;
  F << 1, dt, 0.5 * dt * dt,
       0, 1,  dt,
       0, 0,  1;
  return F;
}

gtsam::Matrix3 UncertaintyPropagator::ComputeProcessNoise(
    const SceneFeatures& features,
    double dt) const {
  // 基础过程噪声
  gtsam::Matrix3 Q = gtsam::Matrix3::Identity();
  
  // 时间相关
  double time_factor = std::sqrt(dt);
  
  // 场景相关
  double scene_noise = 
      features.traffic_density * config_.traffic_weight +
      features.occlusion_ratio * config_.occlusion_weight +
      std::abs(features.curvature) * config_.curvature_weight;
  
  return Q * scene_noise * time_factor;
}

double UncertaintyPropagator::ComputePropagationGain(
    const SceneFeatures& features) const {
  // 基础增益
  double gain = 1.0;
  
  // 场景复杂度影响
  gain *= (1.0 + features.interaction_complexity * 
           config_.interaction_weight);
  
  // 环境影响
  if (features.is_intersection) gain *= config_.intersection_factor;
  if (features.is_merge_area) gain *= config_.merge_area_factor;
  if (features.is_narrow) gain *= config_.narrow_area_factor;
  
  return gain;
}

} // namespace planning