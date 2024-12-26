/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <glog/logging.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include <memory>
#include <mutex>

#include "planning_core/mock_predictor/const_vel_predictor.h"
#include "planning_core/navigation/navigation_map.h"
#include "planning_core/planner/planner.h"
#include "planning_core/planning_common/data_frame.h"
#include "planning_core/simulation/simulator_adapter.h"
#include "planning_core/planning_common/perception_uncertainty.h"

namespace planning {

/**
 * @brief 不确定性估计器类，负责估计障碍物的感知不确定性
 */
class UncertaintyEstimator {
 public:
  /**
   * @brief 初始化不确定性估计器
   * @param nh ROS节点句柄，用于加载参数
   */
  void Init(const ros::NodeHandle& nh);
  
  /**
   * @brief 估计单个障碍物的感知不确定性
   * @param ego_state 自车状态
   * @param obstacle 需要估计不确定性的障碍物
   */
  void EstimateObstacleUncertainty(
      const common::State& ego_state,
      Obstacle* obstacle);

 private:
  // 距离相关的不确定性参数
  double base_position_std_{0.1};    // 基础位置标准差
  double position_growth_rate_{0.01}; // 位置不确定性增长率
  
  // 朝向不确定性参数
  double base_heading_std_{0.1};     // 基础朝向标准差
  double heading_growth_rate_{0.02};  // 朝向不确定性增长率
  
  // 观测限制参数
  double max_reliable_distance_{50.0}; // 最大可靠观测距离
  double reliable_fov_angle_{M_PI_2}; // 可靠视场角（弧度）
  
  // 几何尺寸不确定性参数
  double size_uncertainty_base_{0.05};  // 基础尺寸不确定性
  double size_uncertainty_rate_{0.01};  // 尺寸不确定性增长率
};



class PlanningCore {
 public:
  PlanningCore() = default;

  void Init();

  void Run(const ros::TimerEvent&);

 private:
  void NewRouteCallBack(const geometry_msgs::PoseStamped& goal);
  void JoyCallBack(const sensor_msgs::Joy& joy);
  bool UpdateDataFrame();

 private:
  std::mutex route_mutex_;
  bool random_drive_mode_ = false;
  bool has_new_route_ = false;
  ros::Subscriber route_target_sub_;
  geometry_msgs::PoseStamped route_goal_;

  ros::Subscriber joy_sub_;
  int suggest_lane_change_ = 0;

  NavigationMap navigation_map_;
  std::shared_ptr<DataFrame> data_frame_;
  std::unique_ptr<simulation::SimulatorAdapter> simulator_;

  std::unique_ptr<Planner> planner_;
  std::unique_ptr<MockPredictor> mock_predictor_;

  UncertaintyEstimator uncertainty_estimator_;
};

}  // namespace planning
