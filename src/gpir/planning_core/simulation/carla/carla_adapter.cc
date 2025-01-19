/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "planning_core/simulation/carla/carla_adapter.h"

#include <ackermann_msgs/AckermannDrive.h>
#include <glog/logging.h>

#include <thread>

#include "common/utils/math.h"

namespace planning {
namespace simulation {

// CarlaAdapter::CarlaAdapter() {
//   error_analyzer_ = std::make_unique<ControlErrorAnalyzer>();
// }

void CarlaAdapter::Init() {
  error_analyzer_ = std::make_unique<ControlErrorAnalyzer>();

  carla_ego_info_.Init();
  carla_mock_perception_.Init();

  ros::NodeHandle node;
  control_cmd_pub_ = node.advertise<ackermann_msgs::AckermannDrive>(
      "carla/ego_vehicle/ackermann_cmd", 1);
  mpc_controller_.Init();

  // 添加检查
  CHECK(error_analyzer_) << "Error analyzer initialization failed!";
};

bool CarlaAdapter::InitVehicleParam(VehicleParam* vehicle_param) {
  for (int i = 0; i < 10 && ros::ok(); ++i) {
    ros::spinOnce();
    if (carla_ego_info_.GetVehicleParam(vehicle_param)) {
      wheel_base_ = vehicle_param->wheel_base;
      LOG(INFO) << "\n" << vehicle_param->DebugString();
      return true;
    }
    LOG(WARNING) << "fail to get vehicle param from Carla, Retry: " << i;
    std::this_thread::sleep_for(std::chrono::seconds(2));
  }
  return false;
}

bool CarlaAdapter::UpdateEgoState(common::State* state) {
  return carla_ego_info_.UpdateEgoState(state);
}

bool CarlaAdapter::UpdatePerceptionResults(std::vector<Obstacle>* obstacles) {
  return carla_mock_perception_.UpdateMockPerceptionResult(obstacles);
}

void CarlaAdapter::SetTrajectory(const common::Trajectory& trajectory) {
  if (trajectory.empty()) {
    LOG(WARNING) << "Empty trajectory received in SetTrajectory";
    return;
  }

  common::State ego_state;
  ackermann_msgs::AckermannDrive ackermann_drive;
  // 1. 设置车辆参数
  CHECK_GT(wheel_base_, 2);
  mpc_controller_.set_wheel_base(wheel_base_);

  // 2. 更新车辆状态
  carla_ego_info_.UpdateEgoState(&ego_state);

  // 3. 计算控制量
  mpc_controller_.CalculateAckermannDrive(ego_state, trajectory,
                                          &ackermann_drive);

  // 计算控制延迟
  double control_delay = CalculateControlDelay(ros::Time::now());
  LOG(INFO) << "control_delay " << control_delay;

  try {
    AnalyzeTrackingError(ego_state, trajectory);
  } catch (const std::exception& e) {
    LOG(ERROR) << "Exception in AnalyzeTrackingError: " << e.what();
  }
  
  // 更新误差统计
  UpdateControlErrorStatistics(ego_state, ackermann_drive, control_delay);

  // 保存历史控制指令
  ControlCommand cmd;
  cmd.cmd = ackermann_drive;
  cmd.timestamp = ros::Time::now();
  historical_commands_.push_back(cmd);
  if (historical_commands_.size() > kMaxHistorySize) {
    historical_commands_.pop_front();
  }

  // 4. 发布控制消息
  control_cmd_pub_.publish(ackermann_drive);
}

void CarlaAdapter::ControlLoop() {
  ros::Rate rate(20);
  common::State ego_state;
  ackermann_msgs::AckermannDrive ackermann_drive;
  mpc_controller_.set_wheel_base(wheel_base_);

  while (ros::ok()) {
    carla_ego_info_.UpdateEgoState(&ego_state);
    mpc_controller_.CalculateAckermannDrive(ego_state, trajectory_,
                                            &ackermann_drive);
    control_cmd_pub_.publish(ackermann_drive);
    rate.sleep();
  }
}

void CarlaAdapter::AnalyzeTrackingError(
    const common::State& current_state,
    const common::Trajectory& ref_trajectory) {
   // 首先检查轨迹是否为空
  if (ref_trajectory.empty()) {
    LOG(WARNING) << "Reference trajectory is empty in AnalyzeTrackingError";
    return;
  }

  // 注意：这里也可以添加对ref_state的检查
  // if (!ref_state.IsValid()) {  // 假设State类有IsValid方法
  //   LOG(WARNING) << "Invalid reference state found in AnalyzeTrackingError";
  //   return;
  // }

  
  // 找到参考轨迹上最近点
  auto ref_state = ref_trajectory.GetNearestState(current_state.position);
  
  // 计算横向误差和纵向误差
  double lat_error = (current_state.position - ref_state.position)
                      .dot(Eigen::Vector2d(-sin(ref_state.heading), 
                                          cos(ref_state.heading)));
  double lon_error = (current_state.position - ref_state.position)
                      .dot(Eigen::Vector2d(cos(ref_state.heading), 
                                          sin(ref_state.heading)));

  LOG(INFO) << "lat_error " << lat_error << " lon_error " << lon_error;
                                          
  // 计算航向角误差
  double heading_error = common::NormalizeAngle(
      current_state.heading - ref_state.heading);
  LOG(INFO) << "heading_error " << heading_error;
      
  // 添加到误差分析器
  ControlErrorAnalyzer::TrackingError tracking_error;
  tracking_error.lateral_error = lat_error;
  tracking_error.longitudinal_error = lon_error;
  tracking_error.heading_error = heading_error;
  tracking_error.stamp = ros::Time::now();

  // 确保error_analyzer_已初始化
  if (error_analyzer_) {
    error_analyzer_->AddTrackingError(tracking_error);
  } else {
    LOG(ERROR) << "Error analyzer not initialized in AnalyzeTrackingError";
  }
}

double CarlaAdapter::CalculateControlDelay(const ros::Time& current_time) const {
  if (historical_commands_.empty()) {
    return 0.0;
  }
  return (current_time - historical_commands_.back().timestamp).toSec();
}

void CarlaAdapter::UpdateControlErrorStatistics(
    const common::State& current_state,
    const ackermann_msgs::AckermannDrive& cmd,
    double delay) {
  ControlErrorAnalyzer::ControlError error;
  
  // 速度误差
  error.velocity_error = current_state.velocity - cmd.speed;
  LOG(INFO) << "velocity_error " << error.velocity_error;
  
  // 转向误差
  error.steering_error = current_state.steer - cmd.steering_angle;
  LOG(INFO) << "steering_error " << error.steering_error;
  
  // 执行延迟
  error.execution_delay = delay;
  
  // 添加时间戳
  error.stamp = ros::Time::now();
  
  error_analyzer_->AddControlError(error);
}

}  // namespace simulation
}  // namespace planning