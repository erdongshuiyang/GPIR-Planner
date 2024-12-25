#pragma once

#include <memory>
#include "gp_planner/gp/uncertainty/scene_analyzer.h"
#include "gtsam/linear/NoiseModel.h"

namespace planning {

/**
 * 不确定性状态结构体
 * 包含系统在各个维度上的不确定性评估结果
 */
struct UncertaintyState {
  // 预测相关不确定性
  gtsam::Matrix3 prediction_cov;      // 预测不确定性协方差
  double prediction_confidence;        // 预测可信度
  
  // 状态估计相关不确定性
  gtsam::Matrix3 state_cov;           // 状态估计不确定性协方差
  double state_confidence;            // 状态估计可信度
  
  // 执行相关不确定性
  gtsam::Matrix3 execution_cov;       // 执行不确定性协方差
  double execution_confidence;        // 执行可信度
  
  // 综合不确定性指标
  double total_uncertainty;           // 总体不确定性水平
  bool is_reliable;                   // 是否可靠
};

/**
 * 不确定性评估器类
 * 负责评估系统在不同维度上的不确定性水平
 */
class UncertaintyEvaluator {
 public:
  UncertaintyEvaluator() = default;
  ~UncertaintyEvaluator() = default;

  /**
   * 初始化评估器
   * @param config 配置参数
   */
  bool Init(const UncertaintyEvaluatorConfig& config);

  /**
   * 评估不确定性
   * @param features 场景特征
   * @param base_prediction_cov 基础预测不确定性
   * @param base_state_cov 基础状态估计不确定性
   * @param base_execution_cov 基础执行不确定性
   * @return 不确定性状态
   */
  UncertaintyState EvaluateUncertainty(
      const SceneFeatures& features,
      const gtsam::Matrix3& base_prediction_cov,
      const gtsam::Matrix3& base_state_cov,
      const gtsam::Matrix3& base_execution_cov) const;

 private:
  /**
   * 评估预测不确定性
   */
  void EvaluatePredictionUncertainty(
      const SceneFeatures& features,
      const gtsam::Matrix3& base_cov,
      gtsam::Matrix3* cov,
      double* confidence) const;

  /**
   * 评估状态估计不确定性
   */
  void EvaluateStateUncertainty(
      const SceneFeatures& features,
      const gtsam::Matrix3& base_cov,
      gtsam::Matrix3* cov,
      double* confidence) const;

  /**
   * 评估执行不确定性
   */
  void EvaluateExecutionUncertainty(
      const SceneFeatures& features,
      const gtsam::Matrix3& base_cov,
      gtsam::Matrix3* cov,
      double* confidence) const;

  /**
   * 计算综合不确定性指标
   */
  void ComputeTotalUncertainty(
      const SceneFeatures& features,
      UncertaintyState* state) const;

 private:
  UncertaintyEvaluatorConfig config_;
};

} // namespace planning