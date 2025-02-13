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
     control_covariance_ = Eigen::Matrix2d::Identity() * 1e-6;
    return;
  }


  // 更新控制误差统计
  if (!control_error_buffer_.empty()) {
    // 计算控制误差均值
    Eigen::Vector2d mean = Eigen::Vector2d::Zero();
    double total_delay = 0.0;
    max_execution_delay_ = 0.0;

    for (const auto& error : control_error_buffer_) {
      mean(0) = error.velocity_error;
      mean(1) = error.steering_error;
      total_delay += error.execution_delay;
      max_execution_delay_ = std::max(max_execution_delay_, error.execution_delay);
    }
    mean /= control_error_buffer_.size();
    avg_execution_delay_ = total_delay / control_error_buffer_.size();

    // 重置协方差矩阵
    control_covariance_ = Eigen::Matrix2d::Zero();

    // 计算协方差
    if (control_error_buffer_.size() > 1) {
      for (const auto& error : control_error_buffer_) {
        Eigen::Vector2d err(error.velocity_error, error.steering_error);
        Eigen::Vector2d centered = err - mean;
        control_covariance_ += centered * centered.transpose();
      }
      control_covariance_ /= (control_error_buffer_.size() - 1);
    } else {
      // 如果只有一个样本，使用默认最小协方差
      control_covariance_ = Eigen::Matrix2d::Identity() * 1e-6;
    }

    // 确保协方差矩阵数值稳定
    control_covariance_ = control_covariance_.array().max(1e-6);
    
    // 确保对称性
    control_covariance_ = 0.5 * (control_covariance_ + control_covariance_.transpose());
  }

   // 打印调试信息
  LOG(INFO) << "Control Error Statistics:"
            // << "\nMean velocity error: " << mean(0)
            // << "\nMean steering error: " << mean(1)
            << "\nCovariance:\n" << control_covariance_;

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

   // 确保协方差矩阵非零
  control_covariance_.diagonal() = 
      control_covariance_.diagonal().array().max(1e-6);

  // 打印调试信息
  LOG_EVERY_N(INFO, 100) << "\nControl Error Statistics:"
                         << "\nAverage Execution Delay: " << avg_execution_delay_
                         << "\nMax Execution Delay: " << max_execution_delay_
                         << "\nAverage Lateral Error: " << avg_lateral_error_
                         << "\nAverage Longitudinal Error: " << avg_longitudinal_error_
                         << "\nControl Covariance:\n" << control_covariance_;
}

}  // namespace planning