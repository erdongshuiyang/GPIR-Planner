// static_obstacle_uncertainty_factor.h
#pragma once 

#include <memory>
#include "gp_planner/sdf/signed_distance_field_2d.h"
#include "gtsam/nonlinear/NonlinearFactor.h"
#include "planning_core/planning_common/perception_uncertainty.h"

namespace planning {

/**
 * @brief 静态障碍物不确定性因子,考虑静态障碍物的感知不确定性
 * 
 * 这个因子对传统的GPObstacleFactor进行扩展,通过考虑:
 * 1. 位置不确定性 (通过协方差矩阵建模)
 * 2. 几何尺寸不确定性 (通过尺寸方差建模)
 * 来生成更加保守和安全的路径
 */
class StaticObstacleUncertaintyFactor 
    : public gtsam::NoiseModelFactor1<gtsam::Vector3> {
 public:
  /**
   * @brief 构造函数
   * @param key 优化变量的键值 
   * @param sdf SDF指针,用于距离场查询
   * @param uncertainty 静态障碍物的不确定性信息
   * @param cost 因子权重
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
      const double kappa_r);

  // 虚析构函数
  virtual ~StaticObstacleUncertaintyFactor() = default;

  /**
   * @brief 计算因子误差
   * @param x1 优化变量(Frenet坐标系下的横向状态)
   * @param H1 可选的雅可比矩阵输出
   * @return 误差向量
   */
  gtsam::Vector evaluateError(
      const gtsam::Vector3& x1,
      boost::optional<gtsam::Matrix&> H1 = boost::none) const override;

 protected:
  /**
   * @brief 计算检查点的雅可比矩阵
   * @param x 当前状态
   * @param l 纵向偏移距离
   * @return 2x3雅可比矩阵
   */ 
  gtsam::Matrix23 computeCheckPointJacobian(
      const gtsam::Vector3& x,
      const double l) const;

  /**
   * @brief 计算碰撞概率的雅可比矩阵
   * @param x 当前状态
   * @return 1x3雅可比矩阵
   */
  gtsam::Matrix13 computeCollisionProbJacobian(
      const gtsam::Vector3& x) const;

  /**
   * @brief 计算考虑不确定性的碰撞概率
   * @param x 当前状态
   * @param point 检查点位置
   * @return 碰撞概率
   */
  double computeCollisionProbability(
      const gtsam::Vector3& x,
      const gtsam::Vector2& point) const;

 private:
  // 有向距离场
  static constexpr double kMinProbability_{1e-3}; 

  std::shared_ptr<SignedDistanceField2D> sdf_;
  
  // 静态障碍物的不确定性信息
  PerceptionUncertainty uncertainty_;
  
  // 因子参数
  double epsilon_{0.0};  // 安全裕度
  double param_{0.0};   // 参考线弧长参数  
  double kappa_r_{0.0}; // 参考线曲率
  
  // 车辆相关参数
  double ls_{3.0};  // 车辆特征长度
  
  // 临时变量(用于提高计算效率)
  mutable double cos_theta_{0.0};
  mutable double sin_theta_{0.0};
  
  // 不确定性相关参数
  static constexpr double kSafetyScaling_{1.1};  // 3σ规则的缩放系数 3.0
//   static constexpr double kMinProbability_{1e-6}; // 最小概率阈值
};

} // namespace planning