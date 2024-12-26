#pragma once

#include <memory>

#include "gp_planner/sdf/signed_distance_field_2d.h"
#include "gtsam/nonlinear/NonlinearFactor.h"
#include "planning_core/planning_common/perception_uncertainty.h"

namespace planning {

/**
 * @brief 考虑感知不确定性的轨迹优化因子
 * 
 * 这个因子将感知系统的不确定性估计融入到轨迹优化中，主要考虑:
 * 1. 障碍物位置的不确定性 (通过协方差矩阵建模)
 * 2. 障碍物朝向的不确定性 (通过标量方差建模)
 * 3. 障碍物几何尺寸的不确定性 (通过长宽方差建模)
 */
class GPPerceptionUncertaintyFactor 
    : public gtsam::NoiseModelFactor1<gtsam::Vector3> {
 public:
  /**
   * @brief 构造函数
   * @param key 优化变量的键值
   * @param sdf 有向距离场指针
   * @param uncertainty 感知不确定性信息
   * @param weight 因子权重
   * @param epsilon 安全裕度
   * @param s 参考线弧长
   * @param kappa_r 参考线曲率
   */
  GPPerceptionUncertaintyFactor(
      gtsam::Key key,
      const std::shared_ptr<SignedDistanceField2D>& sdf,
      const PerceptionUncertainty& uncertainty,
      double weight,
      double epsilon,
      double s,
      double kappa_r);

  /**
   * @brief 计算因子误差
   * @param x 优化变量(Frenet坐标系下的横向状态)
   * @param H 可选的雅可比矩阵
   * @return 误差向量
   */
  gtsam::Vector evaluateError(
      const gtsam::Vector3& x,
      boost::optional<gtsam::Matrix&> H = boost::none) const override;

 private:
  /**
   * @brief 计算车辆与障碍物的碰撞概率
   * @param x 当前状态
   * @return 碰撞概率
   */
  double computeCollisionProbability(const gtsam::Vector3& x) const;

  /**
   * @brief 计算检查点的雅可比矩阵
   * @param x 当前状态
   * @param l 纵向偏移距离
   * @return 2x3的雅可比矩阵
   */
  gtsam::Matrix23 computeCheckPointJacobian(
      const gtsam::Vector3& x, double l) const;

  /**
   * @brief 计算误差对状态的雅可比矩阵
   * @param x 当前状态
   * @return 1x3的雅可比矩阵
   */
  gtsam::Matrix computeJacobian(const gtsam::Vector3& x) const;

 private:
  // 有向距离场
  std::shared_ptr<SignedDistanceField2D> sdf_;
  
  // 感知不确定性信息
  PerceptionUncertainty uncertainty_;
  
  // 安全裕度参数
  double epsilon_;
  
  // 参考线参数
  double s_;            // 弧长
  double kappa_r_;      // 曲率
  
  // 车辆参数
  double ls_{3.0};      // 车辆特征长度
  
  // 缓存变量(用于提高计算效率)
  mutable double cos_theta_{0.0};
  mutable double sin_theta_{0.0};
};

}  // namespace planning