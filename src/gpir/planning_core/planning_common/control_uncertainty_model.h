// src/gpir/planning_core/planning_common/control_uncertainty_model.h
#pragma once

#include <Eigen/Dense>
#include <memory>
#include <vector>

#include "planning_core/planning_common/control_error_analyzer.h"
#include "common/base/state.h"

namespace planning {

/**
 * @brief 控制不确定性模型
 * 
 * 该类封装了系统控制过程中的各种不确定性来源:
 * - 基于SDE的状态方程建模
 * - 基于历史数据的误差统计
 * - 控制延迟和执行误差
 */
class ControlUncertaintyModel {
public:
  // 默认构造函数，初始化所有矩阵
  ControlUncertaintyModel();

  /**
   * @brief 基于SDE的状态空间模型参数
   * dx = Ax + Bu + Fw
   * 其中w为高斯白噪声，w~N(0,Qc)
   */
  struct SDEParameters {
    Eigen::MatrixXd A;  // 系统矩阵
    Eigen::MatrixXd B;  // 输入矩阵
    Eigen::MatrixXd F;  // 噪声影响矩阵
    Eigen::MatrixXd Qc; // 噪声强度矩阵
    
    // 初始化所有矩阵为适当尺寸的零矩阵
    SDEParameters() {
      const int nx = 3;  // 状态维度[x, v, theta]
      const int nu = 2;  // 控制维度[a, delta]
      A = Eigen::MatrixXd::Zero(nx, nx);
      B = Eigen::MatrixXd::Zero(nx, nu);
      F = Eigen::MatrixXd::Zero(nx, nu);
      Qc = Eigen::MatrixXd::Zero(nu, nu);
    }
  };

  /**
   * @brief 误差统计信息
   */
  struct ErrorStatistics {
    // 轨迹跟踪误差
    double lateral_error_mean{0.0};
    double lateral_error_var{0.0};
    double longitudinal_error_mean{0.0};
    double longitudinal_error_var{0.0};
    
    // 控制误差
    double velocity_error_var{0.0};
    double steering_error_var{0.0};
    
    // 执行延迟
    double delay_mean{0.0};
    double delay_var{0.0};
    
    // 协方差矩阵
    Eigen::Matrix2d control_error_covariance{Eigen::Matrix2d::Zero()};
  };

  /**
   * @brief 从误差分析器更新模型参数
   * @param analyzer 控制误差分析器
   */
  void UpdateFromAnalyzer(const ControlErrorAnalyzer& analyzer);

  /**
   * @brief 预测给定状态的不确定性传播
   * @param current_state 当前状态
   * @param dt 时间步长
   * @param state_covariance 输出的状态协方差
   */
  void PredictUncertainty(
    const common::State& current_state,
    double dt,
    Eigen::Matrix3d* state_covariance) const;  // 改为Matrix3d

  // Getters
  const SDEParameters& GetSDEParams() const { return sde_params_; }
  const ErrorStatistics& GetErrorStats() const { return error_stats_; }
  
  // 检查模型是否有效
  bool IsValid() const {
    return sde_params_.Qc.allFinite() &&
           error_stats_.control_error_covariance.allFinite() &&
           std::isfinite(error_stats_.delay_mean);
  }

private:
  // 更新SDE参数
  void UpdateSDEParameters();
  
  // 配置参数
  static constexpr double kDefaultSystemNoise = 0.1;
  static constexpr double kMinVariance = 1e-6;
  
  SDEParameters sde_params_;
  ErrorStatistics error_stats_;
};

}  // namespace planning