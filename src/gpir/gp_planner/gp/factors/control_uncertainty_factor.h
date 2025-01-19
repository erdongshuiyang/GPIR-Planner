#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>

#include "planning_core/planning_common/control_uncertainty_model.h"
#include "common/base/state.h"

namespace planning {

/**
 * @brief 控制不确定性因子
 * 
 * 该因子考虑了控制执行过程中的不确定性,包括:
 * - 控制误差的传播
 * - 执行延迟的影响
 * - 系统噪声的累积
 */
class ControlUncertaintyFactor : public gtsam::NoiseModelFactor2<gtsam::Vector3, gtsam::Vector3> {
public:
  ControlUncertaintyFactor(
      gtsam::Key key1,
      gtsam::Key key2,
      const ControlUncertaintyModel& model,
      double dt,
      double weight = 1.0);

  virtual ~ControlUncertaintyFactor() = default;

  /**
   * @brief 计算控制不确定性带来的误差
   * @param x1 当前状态
   * @param x2 下一状态
   * @param H1 关于x1的雅可比矩阵(可选)
   * @param H2 关于x2的雅可比矩阵(可选)
   */
  gtsam::Vector evaluateError(
      const gtsam::Vector3& x1,
      const gtsam::Vector3& x2,
      boost::optional<gtsam::Matrix&> H1 = boost::none,
      boost::optional<gtsam::Matrix&> H2 = boost::none) const override;

private:
  // 计算状态转移
  void predictNextState(
      const gtsam::Vector3& x1,
      gtsam::Vector3* x2_pred,
      gtsam::Matrix* jacobian) const;

  // 计算含不确定性的代价值
  double computeCost(
      const gtsam::Vector3& error,
      const Eigen::Matrix3d& covariance) const;  // 改为Matrix3d

  // 成员变量
  ControlUncertaintyModel model_;
  double dt_;
  double weight_;

  // 配置参数
  static constexpr double kMinCovariance = 1e-6;
  static constexpr double kMaxCost = 1e6;
};

} // namespace planning