/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include <mutex>
#include <thread>

#include "planning_core/simulation/carla/ego_info/carla_ego_info.h"
#include "planning_core/simulation/carla/mock_perception/carla_mock_perception.h"
#include "planning_core/simulation/controller/mpc_controller.h"
#include "planning_core/simulation/simulator_adapter.h"

#include "planning_core/planning_common/control_error_analyzer.h"


namespace planning {
namespace simulation {

class CarlaAdapter final : public SimulatorAdapter {
 public:
  // 只保留最基本的初始化
  CarlaAdapter() : wheel_base_(0.0) {} 
  ~CarlaAdapter() override = default;

  void Init() override;
  std::string Name() const { return "Carla"; }
  bool InitVehicleParam(VehicleParam* vehicle_param) override;
  bool UpdateEgoState(common::State* state) override;
  bool UpdatePerceptionResults(std::vector<Obstacle>* obstacles) override;
  void SetTrajectory(const common::Trajectory& trajectory) override;

  // 新增:获取控制误差分析结果
  const ControlErrorAnalyzer& GetControlErrorAnalyzer() const {
    return *error_analyzer_;
  }

 protected:
  void ControlLoop();

 private:
  CarlaEgoInfo carla_ego_info_;
  CarlaMockPerception carla_mock_perception_;

  // control
  ros::Publisher control_cmd_pub_;
  std::mutex control_mutex_;

  double wheel_base_ = 0.0;

  common::Trajectory trajectory_;
  MpcController mpc_controller_;

   // 新增成员
  std::unique_ptr<ControlErrorAnalyzer> error_analyzer_;

  // 用于记录历史控制指令
  struct ControlCommand {
    ackermann_msgs::AckermannDrive cmd;
    ros::Time timestamp;
  };
  std::deque<ControlCommand> historical_commands_;
  static constexpr size_t kMaxHistorySize = 100;

  // 新增:计算轨迹跟踪误差
  void AnalyzeTrackingError(
      const common::State& current_state,
      const common::Trajectory& ref_trajectory);

  // 新增:计算控制延迟
  double CalculateControlDelay(const ros::Time& current_time) const;

  // 新增:更新控制误差统计
  void UpdateControlErrorStatistics(
      const common::State& current_state,
      const ackermann_msgs::AckermannDrive& cmd,
      double delay);

};
}  // namespace simulation
}  // namespace planning