// planning_core/planning_common/control_error_analyzer.h
#pragma once

#include <deque>
#include <vector>
#include <Eigen/Dense>
#include <ros/time.h>
#include <glog/logging.h>
#include "common/base/state.h"  // 添加这行
#include "common/base/trajectory.h" // 添加这行

namespace planning {

class ControlErrorAnalyzer {
public:
  // 简化构造函数
    ControlErrorAnalyzer() = default;

  struct TrackingError {
    double lateral_error{0.0};     // 横向跟踪误差
    double longitudinal_error{0.0}; // 纵向跟踪误差
    double heading_error{0.0};      // 航向角误差
    ros::Time stamp;
  };

  struct ControlError {
    double velocity_error{0.0};   // 速度控制误差
    double steering_error{0.0};   // 转向控制误差
    double execution_delay{0.0};  // 执行延迟
    ros::Time stamp;
  };
  //  // 修改构造函数实现
  // ControlErrorAnalyzer() {
  //   // 使用赋值初始化
  //   control_covariance_.setZero();  // 或使用 = Eigen::Matrix2d::Zero()
  //   avg_execution_delay_ = 0.0;
  //   max_execution_delay_ = 0.0;
  //   avg_lateral_error_ = 0.0;
  //   avg_longitudinal_error_ = 0.0;
  // }

  // 添加跟踪误差数据
  void AddTrackingError(const TrackingError& error);

  // 添加控制误差数据
  void AddControlError(const ControlError& error);

  // 获取控制误差协方差矩阵
  const Eigen::Matrix2d& GetControlCovariance() const { 
    return control_covariance_; 
  }

  // 获取统计数据
  double GetAverageExecutionDelay() const { 
    return avg_execution_delay_; 
  }
  
  double GetMaxExecutionDelay() const { 
    return max_execution_delay_; 
  }

  double GetAverageLateralError() const {
    return avg_lateral_error_;
  }

  double GetAverageLongitudinalError() const {
    return avg_longitudinal_error_;
  }

  // 获取误差序列
  const std::vector<double>& GetVelocityErrors() const {
    return velocity_errors_;
  }

  const std::vector<double>& GetSteeringErrors() const {
    return steering_errors_;
  }

  // 清除历史数据
  void Clear();

private:
  // 更新统计量
  void UpdateStatistics();

  // 更新误差序列
  void UpdateErrorSequence(const ControlError& error);

  // 控制误差缓存
  std::deque<ControlError> control_error_buffer_;
  std::deque<TrackingError> tracking_error_buffer_;

  // 统计量
    Eigen::Matrix2d control_covariance_{Eigen::Matrix2d::Zero()};
    double avg_execution_delay_{0.0};
    double max_execution_delay_{0.0};
    double avg_lateral_error_{0.0};
    double avg_longitudinal_error_{0.0};

  // 误差序列
  std::vector<double> velocity_errors_;
  std::vector<double> steering_errors_;

  // 配置参数
  static constexpr size_t kMaxBufferSize = 100;  // 缓存大小
  static constexpr size_t kMaxSequenceSize = 1000; // 序列最大长度
};

}  // namespace planning