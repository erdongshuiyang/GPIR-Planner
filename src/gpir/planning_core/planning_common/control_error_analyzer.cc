// planning_core/planning_common/control_error_analyzer.cc
#include "planning_core/planning_common/control_error_analyzer.h"

namespace planning {

void ControlErrorAnalyzer::AddTrackingError(const TrackingError& error) {
  tracking_error_buffer_.push_back(error);
  if (tracking_error_buffer_.size() > kMaxBufferSize) {
    tracking_error_buffer_.pop_front();
  }
  UpdateStatistics();
}

void ControlErrorAnalyzer::AddControlError(const ControlError& error) {
  control_error_buffer_.push_back(error);
  if (control_error_buffer_.size() > kMaxBufferSize) {
    control_error_buffer_.pop_front();
  }
  UpdateErrorSequence(error);
  UpdateStatistics();
}

void ControlErrorAnalyzer::Clear() {
  control_error_buffer_.clear();
  tracking_error_buffer_.clear();
  velocity_errors_.clear();
  steering_errors_.clear();
  
  control_covariance_ = Eigen::Matrix2d::Zero();
  avg_execution_delay_ = 0.0;
  max_execution_delay_ = 0.0;
  avg_lateral_error_ = 0.0;
  avg_longitudinal_error_ = 0.0;
}

void ControlErrorAnalyzer::UpdateErrorSequence(const ControlError& error) {
  velocity_errors_.push_back(error.velocity_error);
  steering_errors_.push_back(error.steering_error);

  if (velocity_errors_.size() > kMaxSequenceSize) {
    velocity_errors_.erase(velocity_errors_.begin());
  }
  if (steering_errors_.size() > kMaxSequenceSize) {
    steering_errors_.erase(steering_errors_.begin());
  }
}

void ControlErrorAnalyzer::UpdateStatistics() {
  if (control_error_buffer_.empty() && tracking_error_buffer_.empty()) {
    return;
  }

  // 更新控制误差统计
  if (!control_error_buffer_.empty()) {
    // 计算控制误差均值
    Eigen::Vector2d mean = Eigen::Vector2d::Zero();
    double total_delay = 0.0;
    max_execution_delay_ = 0.0;

    for (const auto& error : control_error_buffer_) {
      Eigen::Vector2d err(error.velocity_error, error.steering_error);
      mean += err;
      total_delay += error.execution_delay;
      max_execution_delay_ = std::max(max_execution_delay_, error.execution_delay);
    }

    mean /= control_error_buffer_.size();
    avg_execution_delay_ = total_delay / control_error_buffer_.size();

    // 计算协方差矩阵
    control_covariance_ = Eigen::Matrix2d::Zero();
    for (const auto& error : control_error_buffer_) {
      Eigen::Vector2d err(error.velocity_error, error.steering_error);
      Eigen::Vector2d centered = err - mean;
      control_covariance_ += centered * centered.transpose();
    }
    control_covariance_ /= (control_error_buffer_.size() - 1);

    // 添加数值保护
    for (int i = 0; i < 2; ++i) {
      for (int j = 0; j < 2; ++j) {
        if (std::abs(control_covariance_(i,j)) < 1e-10) {
          control_covariance_(i,j) = 0.0;
        }
      }
    }
  }

  // 更新轨迹跟踪误差统计
  if (!tracking_error_buffer_.empty()) {
    double total_lat_error = 0.0;
    double total_lon_error = 0.0;

    for (const auto& error : tracking_error_buffer_) {
      total_lat_error += error.lateral_error;
      total_lon_error += error.longitudinal_error;
    }

    avg_lateral_error_ = total_lat_error / tracking_error_buffer_.size();
    avg_longitudinal_error_ = total_lon_error / tracking_error_buffer_.size();
  }

  // 打印调试信息
  LOG_EVERY_N(INFO, 100) << "\nControl Error Statistics:"
                         << "\nAverage Execution Delay: " << avg_execution_delay_
                         << "\nMax Execution Delay: " << max_execution_delay_
                         << "\nAverage Lateral Error: " << avg_lateral_error_
                         << "\nAverage Longitudinal Error: " << avg_longitudinal_error_
                         << "\nControl Covariance:\n" << control_covariance_;
}

}  // namespace planning