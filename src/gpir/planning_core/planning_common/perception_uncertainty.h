#pragma once

#include <Eigen/Dense>

namespace planning {

/**
 * @brief 表示感知系统对障碍物状态估计的不确定性
 * 
 * 这个类封装了障碍物状态估计中的各种不确定性，包括：
 * - 位置不确定性（2D协方差矩阵）
 * - 朝向不确定性（标量方差）
 * - 几何尺寸不确定性（长度和宽度的方差）
 */
class PerceptionUncertainty {
 public:
  PerceptionUncertainty() {
    // 初始化协方差矩阵为单位矩阵
    position_covariance.setIdentity();
  }

  // 位置的2x2协方差矩阵
  Eigen::Matrix2d position_covariance;
  
  // 朝向的不确定性（方差）
  double heading_variance{0.0};
  
  // 几何尺寸的不确定性
  double length_variance{0.0};
  double width_variance{0.0};

  /**
   * @brief 设置位置不确定性
   * @param xx x方向的方差
   * @param yy y方向的方差
   * @param xy xy的协方差
   */
  void SetPositionCovariance(double xx, double yy, double xy) {
    position_covariance << xx, xy, xy, yy;
  }
  
  /**
   * @brief 检查不确定性是否有效
   * @return bool 如果所有不确定性参数都是有限值则返回true
   */
  bool IsValid() const {
    return position_covariance.allFinite() &&
           std::isfinite(heading_variance) &&
           std::isfinite(length_variance) &&
           std::isfinite(width_variance);
  }
};

}  // namespace planning