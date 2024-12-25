// gp_planner/gp/factors/gp_prediction_uncertainty_factor.h
#pragma once

#include "gp_planner/gp/uncertainty/scene_analyzer.h"
#include "gtsam/nonlinear/NonlinearFactor.h"

namespace planning {

class GPPredictionUncertaintyFactor
    : public gtsam::NoiseModelFactor2<gtsam::Vector3, gtsam::Vector3> {
 public:
  /** 
   * @brief 构造预测不确定性因子
   * @param key1 前一个状态的key
   * @param key2 后一个状态的key
   * @param delta 时间间隔
   * @param scene_features 场景特征
   * @param base_Qc 基础不确定性协方差
   */
  GPPredictionUncertaintyFactor(
      gtsam::Key key1, gtsam::Key key2,
      const double delta,
      const SceneFeatures& scene_features,
      const gtsam::Matrix3& base_Qc)
      : NoiseModelFactor2(
            ComputeNoiseModel(scene_features, base_Qc),
            key1, key2),
        delta_(delta),
        phi_(ComputeTransitionMatrix(delta)) {
    
    // 基于场景特征计算不确定性缩放因子
    uncertainty_scale_ = ComputeUncertaintyScale(scene_features);
  }

  virtual ~GPPredictionUncertaintyFactor() = default;

  /** 
   * @brief 计算因子误差
   * @param x1 前一个状态
   * @param x2 后一个状态
   * @param H1 可选的雅可比矩阵
   * @param H2 可选的雅可比矩阵
   */
  gtsam::Vector evaluateError(
      const gtsam::Vector3& x1,
      const gtsam::Vector3& x2,
      boost::optional<gtsam::Matrix&> H1 = boost::none,
      boost::optional<gtsam::Matrix&> H2 = boost::none) const override;

 private:
  /** 
   * @brief 计算噪声模型
   * @param features 场景特征
   * @param base_Qc 基础不确定性协方差
   */
  static gtsam::SharedNoiseModel ComputeNoiseModel(
      const SceneFeatures& features,
      const gtsam::Matrix3& base_Qc);

  /**
   * @brief 计算不确定性缩放因子
   * @param features 场景特征
   */
  static double ComputeUncertaintyScale(
      const SceneFeatures& features);

  /**
   * @brief 计算状态转移矩阵
   * @param delta 时间间隔
   */
  static gtsam::Matrix33 ComputeTransitionMatrix(
      const double delta);

 private:
  double delta_;                // 时间间隔
  gtsam::Matrix33 phi_;        // 状态转移矩阵
  double uncertainty_scale_;    // 不确定性缩放因子
};

} // namespace planning