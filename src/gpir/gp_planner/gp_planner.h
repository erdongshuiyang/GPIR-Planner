/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

// #include <std_msgs/Bool.h>

// #include <fstream>
// #include <memory>

// #include "gp_planner/sdf/signed_distance_field_2d.h"
// #include "planning_core/planner/planner.h"
// #include "planning_core/planning_common/vehicle_info.h"

// #include "planning_core/planning_data_collector.h"

// #include <nav_msgs/Path.h>
// #include <std_msgs/Float64MultiArray.h>
// #include <geometry_msgs/PoseStamped.h>

// #include <planning_core/planning_common/perception_uncertainty.h>

// #include "planning_core/planning_common/control_error_analyzer.h"

#pragma once

#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseStamped.h>

#include <fstream>
#include <memory>


// 基础包含
#include "planning_core/planner/planner.h"
#include "planning_core/planning_common/vehicle_info.h"
#include "planning_core/planning_common/control_error_analyzer.h"
#include "planning_core/planning_common/perception_uncertainty.h"

// GPIR相关包含
#include "gp_planner/sdf/signed_distance_field_2d.h"
#include "gp_planner/gp/gp_incremental_path_planner.h"

// 添加 simulator adapter 相关头文件
#include "planning_core/simulation/simulator_adapter.h"

// 前向声明GPPath
namespace planning {
class GPPath;
class PlanningCore;  // 前向声明
}

namespace planning {

class GPPlanner : public Planner {
 public:
  GPPlanner() = default;
  virtual ~GPPlanner() {}

  void Init() override;
  void PlanOnce(NavigationMap* navigation_map_) override;

  // 添加设置静态障碍物不确定性的方法
  void SetStaticObstacleUncertainty(const PerceptionUncertainty& uncertainty)  override {
    static_obstacle_uncertainty_ = uncertainty;
  }

 protected:
  bool PlanWithGPIR(const common::State& ego_state,
                    const ReferenceLine& reference_line,
                    const std::vector<Obstacle>& dynamic_agents,
                    const std::vector<Eigen::Vector2d>& virtual_obstacles,
                    const common::Trajectory& last_trajectory,
                    common::Trajectory* trajectory);

 private:
  bool ProcessObstacles(const std::vector<Obstacle>& raw_obstacles,
                        const ReferenceLine& reference_line,
                        std::vector<Obstacle>* cirtical_obstacles);

  void VisualizeTrajectory(
      const std::vector<std::pair<hdmap::LaneSegmentBehavior,
                                  common::Trajectory>>& trajectory_candidates);

  void VisualizeCriticalObstacle(
      const std::vector<Obstacle>& critical_obstacles);

  void VisualizeTargetLane(const ReferenceLine& reference_line);


  void PublishPlanningData(const GPPath& path);

  // 在现有private成员之后添加
  void UpdateControlUncertainty(GPIncrementalPathPlanner& path_planner,
                               const ControlErrorAnalyzer* analyzer);

 private:
  ros::Publisher trajectory_pub_;
  ros::Publisher critical_obstacle_pub_;
  ros::Publisher target_lane_pub_;

  ros::Publisher path_data_pub_;
  ros::Publisher curvature_data_pub_;

  common::State state_;
  common::Box2D ego_box_;
  VehicleParam vehicle_param_;

  int trajectory_index_ = 0;

  int max_iter = 5;
  double reference_speed_ = 0.0;
  const double lateral_critical_thereshold_ = 6;

  std::vector<Obstacle> static_obstacles_;
  std::vector<Obstacle> dynamic_obstacles_;

  hdmap::LaneSegmentBehavior last_behavior_ = hdmap::LaneSegmentBehavior::kKeep;

  // debug
  struct TimeConsumption {
    double sdf = 0.0;
    double init_path = 0.0;
    double init_st = 0.0;
    double refinement = 0.0;
  };
  TimeConsumption time_consuption_;

  // 添加静态障碍物不确定性成员
  PerceptionUncertainty static_obstacle_uncertainty_;

   // 改为指针引用
  const std::unique_ptr<simulation::SimulatorAdapter>* simulator_ptr_{nullptr};
};

}  // namespace planning
