// gp_planner/gp/uncertainty/uncertainty_propagator.h
#pragma once

#include <memory>
#include <vector>
#include "gp_planner/gp/uncertainty/uncertainty_evaluator.h"
#include "gtsam/linear/NoiseModel.h"

namespace planning {

/**
 * 不确定性传播器类
 * 负责处理系统不确定性在时间和空间维度上的传播
 * 主要功能:
 * 1. 时间维度传播 - 预测未来时刻的不确定性
 * 2. 空间维度传播 - 计算不同位置的不确定性
 * 3. 多源不确定性融合 - 综合考虑各种不确定性来源
 */
class UncertaintyPropagator {
 public:
  UncertaintyPropagator() = default;
  ~UncertaintyPropagator() = default;

  /**
   * 初始化传播器
   * @param config 配置参数
   * @return 是否初始化成功
   */
  bool Init(const UncertaintyPropagatorConfig& config);

  /**
   * 预测未来时刻的不确定性状态
   * @param current_state 当前不确定性状态
   * @param features 场景特征
   * @param dt 预测时间步长
   * @return 预测的不确定性状态
   */
  UncertaintyState PropagateUncertainty(
      const UncertaintyState& current_state,
      const SceneFeatures& features,
      double dt) const;

  /**
   * 在路径上传播不确定性
   * @param reference_line 参考线
   * @param init_state 初始不确定性状态
   * @param evaluator 不确定性评估器
   * @return 路径上的不确定性状态序列
   */
  std::vector<UncertaintyState> PropagateAlongPath(
      const ReferenceLine& reference_line,
      const UncertaintyState& init_state,
      const UncertaintyEvaluator& evaluator) const;

  /**
   * 融合多个不确定性状态
   * @param states 待融合的不确定性状态列表
   * @param weights 融合权重
   * @return 融合后的不确定性状态
   */
  UncertaintyState FuseUncertaintyStates(
      const std::vector<UncertaintyState>& states,
      const std::vector<double>& weights) const;

 private:
  /**
   * 计算状态转移矩阵
   * @param dt 时间步长
   * @return 状态转移矩阵
   */
  gtsam::Matrix33 ComputeTransitionMatrix(double dt) const;

  /**
   * 计算过程噪声协方差
   * @param features 场景特征
   * @param dt 时间步长
   * @return 过程噪声协方差
   */
  gtsam::Matrix3 ComputeProcessNoise(
      const SceneFeatures& features,
      double dt) const;

  /**
   * 根据场景特征调整不确定性传播
   * @param features 场景特征
   * @return 传播增益因子
   */
  double ComputePropagationGain(const SceneFeatures& features) const;

 private:
  UncertaintyPropagatorConfig config_;
};

} // namespace planning