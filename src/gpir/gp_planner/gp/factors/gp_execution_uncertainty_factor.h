// gp_planner/gp/factors/gp_execution_uncertainty_factor.h
#pragma once

#include "gp_planner/gp/uncertainty/scene_analyzer.h"
#include "gtsam/nonlinear/NonlinearFactor.h"
#include "gtsam/linear/NoiseModel.h"

namespace planning {

/**
 * 执行不确定性因子
 * 这个因子建模控制执行过程中的各类不确定性,包括:
 * 1. 控制器响应延迟和误差
 * 2. 动力学模型参数不确定性
 * 3. 环境因素(如路面状况)带来的扰动
 */
class GPExecutionUncertaintyFactor
    : public gtsam::NoiseModelFactor2<gtsam::Vector3, gtsam::Vector3> {
 public:
  /**
   * @brief 构造执行不确定性因子
   * @param key1 前一个状态的key
   * @param key2 后一个状态的key
   * @param delta 时间间隔
   * @param scene_features 场景特征
   * @param base_Qc 基础执行不确定性协方差
   * @param controller_params 控制器参数
   */
  GPExecutionUncertaintyFactor(
      gtsam::Key key1, 
      gtsam::Key key2,
      const double delta,
      const SceneFeatures& scene_features, 
      const gtsam::Matrix3& base_Qc,
      const ControllerParams& controller_params);

  virtual ~GPExecutionUncertaintyFactor() = default;

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
   */
  static gtsam::SharedNoiseModel ComputeNoiseModel(
      const SceneFeatures& features,
      const gtsam::Matrix3& base_Qc,
      const ControllerParams& params);

  /**
   * @brief 计算动力学雅可比矩阵
   */
  static gtsam::Matrix33 ComputeDynamicsJacobian(
      const double delta,
      const ControllerParams& params);

  struct ControllerParams {
    double lat_delay;     // 横向控制延迟
    double lon_delay;     // 纵向控制延迟
    double lat_error;     // 横向控制误差
    double lon_error;     // 纵向控制误差
  };

 private:
  double delta_;              // 时间间隔
  gtsam::Matrix33 J_;        // 动力学雅可比矩阵
  ControllerParams params_;  // 控制器参数
  double execution_scale_;   // 执行不确定性缩放因子
};

} // namespace planning