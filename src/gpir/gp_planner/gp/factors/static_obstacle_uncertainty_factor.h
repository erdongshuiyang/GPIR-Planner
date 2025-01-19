/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include <memory>
#include "gp_planner/sdf/signed_distance_field_2d.h"
#include "gtsam/nonlinear/NonlinearFactor.h"
#include "planning_core/planning_common/perception_uncertainty.h"
#include "gp_planner/gp/utils/gp_utils.h"

namespace planning {

class StaticObstacleUncertaintyFactor 
    : public gtsam::NoiseModelFactor1<gtsam::Vector3> {
 public:
  /**
   * @brief 构造函数
   * @param key 优化变量键值
   * @param sdf SDF指针
   * @param uncertainty 感知不确定性
   * @param cost 基础代价权重
   * @param epsilon 安全裕度
   * @param param 参考线弧长参数
   * @param kappa_r 参考线曲率
   */
  StaticObstacleUncertaintyFactor(
      gtsam::Key key,
      const std::shared_ptr<SignedDistanceField2D>& sdf,
      const PerceptionUncertainty& uncertainty,  
      const double cost,
      const double epsilon, 
      const double param,
      const double kappa_r)
      : NoiseModelFactor1(gtsam::noiseModel::Isotropic::Sigma(2, cost), key),
        sdf_(sdf),
        uncertainty_(uncertainty),
        epsilon_(epsilon),
        param_(param),
        kappa_r_(kappa_r) {
    // 限制不确定性影响
    constexpr double kMaxUncertainty = 2.0;  
    uncertainty_.position_covariance = 
        uncertainty_.position_covariance.cwiseMin(kMaxUncertainty);
  }

  virtual ~StaticObstacleUncertaintyFactor() = default;

  /**
   * @brief 计算因子误差
   * @param x1 当前状态变量
   * @param H1 可选的雅可比矩阵
   * @return 误差向量
   */
  gtsam::Vector evaluateError(
      const gtsam::Vector3& x1,
      boost::optional<gtsam::Matrix&> H1 = boost::none) const override;

  /**
   * @brief 计算检查点的雅可比矩阵
   * @param x 状态变量
   * @param l 纵向偏移量
   * @return 2x3雅可比矩阵
   */
  gtsam::Matrix23 CheckPointJacobian(
      const gtsam::Vector3& x,
      const double l) const;

 protected:
  /**
   * @brief 计算考虑不确定性的碰撞概率
   * @param x 状态变量 
   * @param point 检查点位置
   * @return 碰撞概率[0,1]
   */
  double computeCollisionProbability(
      const gtsam::Vector3& x,
      const gtsam::Vector2& point) const;

 private:
  // SDF场指针
  std::shared_ptr<SignedDistanceField2D> sdf_;
  
  // 静态障碍物不确定性
  PerceptionUncertainty uncertainty_;
  
  // 基础参数
  double epsilon_{0.0};  // 安全裕度
  double param_{0.0};   // 参考线弧长参数
  double kappa_r_{0.0}; // 参考线曲率
  
  // 车辆特征长度(前后轮距)
  double ls_{3.0};  
  
  // 临时变量(用于提高计算效率)
  mutable double cos_theta_{0.0};
  mutable double sin_theta_{0.0};

  // 概率平滑参数
  static constexpr double kSmoothFactor_{0.95};
  static constexpr double kMinProb_{1e-3};
  static constexpr double kMaxProb_{0.95};
};

}  // namespace planning