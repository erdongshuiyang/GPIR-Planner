// src/gpir/planning_core/planning_common/control_uncertainty_model.cc
#include "planning_core/planning_common/control_uncertainty_model.h"

namespace planning {

ControlUncertaintyModel::ControlUncertaintyModel() {
  // 初始化SDE系统矩阵的基本结构
  sde_params_.A.block<2,2>(0,1) = Eigen::Matrix2d::Identity();
  sde_params_.F.block<2,2>(1,0) = Eigen::Matrix2d::Identity();
  
  // 设置默认噪声
  // sde_params_.Qc.diagonal() << 0.01 * kDefaultSystemNoise, 0.01 * kDefaultSystemNoise;
  sde_params_.Qc.diagonal() << 0.1 * kDefaultSystemNoise, 0.1 * kDefaultSystemNoise;

}

void ControlUncertaintyModel::UpdateFromAnalyzer(
    const ControlErrorAnalyzer& analyzer) {
  // 更新误差统计
  error_stats_.lateral_error_mean = analyzer.GetAverageLateralError();
  error_stats_.longitudinal_error_mean = analyzer.GetAverageLongitudinalError();
  error_stats_.delay_mean = analyzer.GetAverageExecutionDelay();
  error_stats_.control_error_covariance = analyzer.GetControlCovariance();

  // 提取速度和转向误差方差
  error_stats_.velocity_error_var = 
      std::max(error_stats_.control_error_covariance(0,0), kMinVariance);
  error_stats_.steering_error_var = 
      std::max(error_stats_.control_error_covariance(1,1), kMinVariance);

  // 更新SDE参数
  UpdateSDEParameters();
}

void ControlUncertaintyModel::UpdateSDEParameters() {
   // 添加防护
  if (!error_stats_.control_error_covariance.allFinite() ||
      error_stats_.control_error_covariance.isZero()) {
    // 使用默认最小值
    sde_params_.Qc.diagonal() << kMinVariance, kMinVariance;
    LOG(WARNING) << "Using minimum variance due to invalid covariance";
    return;
  }

  // 更新噪声协方差矩阵
  sde_params_.Qc(0,0) = error_stats_.velocity_error_var;
  sde_params_.Qc(1,1) = error_stats_.steering_error_var;

  // 确保不小于最小值
  sde_params_.Qc = sde_params_.Qc.array().max(kMinVariance);

   // 添加衰减因子
  const double decay_factor = 0.8;
  sde_params_.Qc *= decay_factor;

   // 打印SDE参数
  LOG(INFO) << "Updated SDE parameters:"
            << "\nSystem noise (velocity): " << sde_params_.Qc(0,0)
            << "\nSystem noise (steering): " << sde_params_.Qc(1,1);

  // 考虑执行延迟的影响
  // if (error_stats_.delay_mean > 0) {
  //   sde_params_.Qc *= (1.0 + 0.5*error_stats_.delay_mean); 
  // }

  // 添加衰减因子
  // const double decay_factor = 0.8;
  sde_params_.Qc(0,0) = decay_factor * error_stats_.velocity_error_var;
  sde_params_.Qc(1,1) = decay_factor * error_stats_.steering_error_var;
}

void ControlUncertaintyModel::PredictUncertainty(
    const common::State& current_state,
    double dt,
    Eigen::Matrix3d* state_covariance) const {  // 改为Matrix3d
  CHECK_NOTNULL(state_covariance);
  
  // 线性化点
  const double v = current_state.velocity;
  const double theta = current_state.heading;
  
  // 更新系统矩阵A
  Eigen::Matrix3d A = sde_params_.A;
  A(0,2) = -v * sin(theta);
  A(1,2) = v * cos(theta);
  
  // 离散化
  Eigen::Matrix3d Ad = Eigen::Matrix3d::Identity() + A * dt;
  Eigen::MatrixXd Fd = sde_params_.F * dt;
  
  // 计算离散时间噪声协方差
  *state_covariance = Ad * (*state_covariance) * Ad.transpose() +
                      Fd * sde_params_.Qc * Fd.transpose();

  // LOG(INFO) << "State before prediction: " << current_state.DebugString();
  // LOG(INFO) << "Predicted covariance:\n" << *state_covariance;
}

}  // namespace planning