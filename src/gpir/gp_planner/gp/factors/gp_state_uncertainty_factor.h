#pragma once

#include "gp_planner/gp/uncertainty/scene_analyzer.h"
#include "gtsam/nonlinear/NonlinearFactor.h"
#include "gtsam/linear/NoiseModel.h"

namespace planning {

/**
 * 状态估计不确定性因子
 * 用于建模定位、感知等模块带来的状态估计不确定性
 * 考虑传感器噪声特性和环境因素的影响
 */
class GPStateUncertaintyFactor 
    : public gtsam::NoiseModelFactor1<gtsam::Vector3> {
 public:
  GPStateUncertaintyFactor(
      gtsam::Key key,
      const gtsam::Vector3& measurement,
      const SceneFeatures& scene_features,
      const gtsam::Matrix3& base_R);

  virtual ~GPStateUncertaintyFactor() = default;

  /**
   * 计算因子误差 - 实现见.cc文件
   */
  gtsam::Vector evaluateError(
      const gtsam::Vector3& x,
      boost::optional<gtsam::Matrix&> H = boost::none) const override;

 private:
  /**
   * 计算噪声模型
   * @param features 场景特征
   * @param base_R 基础测量噪声
   */
  static gtsam::SharedNoiseModel ComputeNoiseModel(
      const SceneFeatures& features,
      const gtsam::Matrix3& base_R);

  /**
   * 计算测量矩阵
   * @param features 场景特征
   */
  static gtsam::Matrix33 ComputeMeasurementMatrix(
      const SceneFeatures& features);

  /**
   * 计算测量可信度
   * @param features 场景特征
   */  
  static double ComputeMeasurementConfidence(
      const SceneFeatures& features);

 private:
  gtsam::Vector3 measurement_;       // 测量值
  gtsam::Matrix33 H_;               // 测量矩阵
  double measurement_confidence_;    // 测量可信度
};

} // namespace planning