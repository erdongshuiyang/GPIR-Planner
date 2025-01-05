/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "planning_core/planning_core.h"

#include <thread>

#include "common/utils/timer.h"
#include "gp_planner/gp_planner.h"
#include "hdmap/hdmap.h"
#include "planning_common/planning_visual.h"
#include "planning_core/simulation/simulator_adapter_factory.h"

namespace planning {

void UncertaintyEstimator::Init(const ros::NodeHandle& nh) {
  std::string prefix = "perception_uncertainty/";
  
  // 加载距离相关参数
  nh.param(prefix + "base_position_std", base_position_std_, 0.1);
  nh.param(prefix + "position_growth_rate", position_growth_rate_, 0.01);
  
  // 加载朝向相关参数
  nh.param(prefix + "base_heading_std", base_heading_std_, 0.1);
  nh.param(prefix + "heading_growth_rate", heading_growth_rate_, 0.02);
  
  // 加载观测限制参数
  nh.param(prefix + "max_reliable_distance", max_reliable_distance_, 50.0);
  nh.param(prefix + "reliable_fov_angle", reliable_fov_angle_, M_PI_2);
  
  // 加载几何尺寸不确定性参数
  nh.param(prefix + "size_uncertainty_base", size_uncertainty_base_, 0.05);
  nh.param(prefix + "size_uncertainty_rate", size_uncertainty_rate_, 0.01);

  LOG(INFO) << "UncertaintyEstimator initialized with:"
            << "\nbase_position_std: " << base_position_std_
            << "\nposition_growth_rate: " << position_growth_rate_;

             
}

void UncertaintyEstimator::EstimateObstacleUncertainty(
    const common::State& ego_state,
    Obstacle* obstacle) const {
  const auto& obs_state = obstacle->state();
  
  // 计算相对位置和距离
  Eigen::Vector2d relative_pos = obs_state.position - ego_state.position;
  double distance = relative_pos.norm();
  
  // 计算观测角度（相对于车辆前向）
  double obs_angle = std::atan2(relative_pos.y(), relative_pos.x()) - 
                    ego_state.heading;
  obs_angle = common::NormalizeAngle(obs_angle);
  
  // 计算不确定性衰减因子（基于距离）
  double distance_factor = std::min(1.0, distance / max_reliable_distance_);
  
  // 计算视场角因子（越接近视场边缘不确定性越大）
  double angle_factor = std::abs(obs_angle) / reliable_fov_angle_;
  
  // 计算位置不确定性
  double position_std = base_position_std_ + 
                       position_growth_rate_ * distance * distance_factor;
                       
  // 考虑观测角度的影响
  position_std *= (1.0 + angle_factor);
  
  // 设置位置不确定性（考虑径向和切向差异）
  double radial_std = position_std;
  double tangential_std = position_std * 1.2;  // 切向不确定性略大
  
  // 构建旋转矩阵，将不确定性从极坐标转换到笛卡尔坐标
  double cos_theta = relative_pos.x() / distance;
  double sin_theta = relative_pos.y() / distance;
  
  Eigen::Matrix2d R;  // 旋转矩阵
  R << cos_theta, -sin_theta,
       sin_theta, cos_theta;
       
  Eigen::Matrix2d D;  // 对角矩阵（径向和切向方差）
  D << radial_std * radial_std, 0,
       0, tangential_std * tangential_std;
       
  // 计算笛卡尔坐标系下的协方差矩阵
  Eigen::Matrix2d cov = R * D * R.transpose();
  
  // 设置位置不确定性
  obstacle->mutable_perception_uncertainty()->SetPositionCovariance(
      cov(0,0), cov(1,1), cov(0,1));
  
  // 设置朝向不确定性
  double heading_std = base_heading_std_ + 
                      heading_growth_rate_ * distance * distance_factor;
  heading_std *= (1.0 + angle_factor);
  obstacle->mutable_perception_uncertainty()->heading_variance = 
      heading_std * heading_std;
  
  // 设置几何尺寸不确定性
  double size_uncertainty = size_uncertainty_base_ + 
                          size_uncertainty_rate_ * distance * distance_factor;
  obstacle->mutable_perception_uncertainty()->length_variance = 
      size_uncertainty * size_uncertainty;
  obstacle->mutable_perception_uncertainty()->width_variance = 
      size_uncertainty * size_uncertainty;
}



void PlanningCore::Init() {
  ros::NodeHandle node("~");
  ros::NodeHandle nh("~"); 
  
  bool load_param = true;
  std::string simulator, town, map_path;
  load_param &= node.getParam("simulator", simulator);
  load_param &= node.getParam("town", town);
  load_param &= node.getParam("map_path", map_path);
  load_param &= node.getParam("random_drive_mode", random_drive_mode_);
  if (!load_param) LOG(FATAL) << "fail to init param";

  // init hdmap
  std::string map = map_path + town + ".txt";
  std::string pcd = map_path + "pcd/" + town + ".pcd";
  if (!hdmap::HdMap::GetMap().LoadMap(map, pcd)) {
    LOG(FATAL) << "fail to init hdmap, \nmap: " << map << "\npcd: " << pcd;
  }

  // init simulator adapter
  simulator_ = simulation::SimulatorFactory::CreateSimulatorAdapter(simulator);
  simulator_->Init();
  if (!simulator_->InitVehicleParam(
          VehicleInfo::Instance().mutable_vehicle_param())) {
    LOG(FATAL) << "fail to init vehicle param from " << simulator_->Name();
  }
  LOG(INFO) << "Init simulator " << simulator_->Name() << " OK";

  // init planning
  navigation_map_.Init();
  data_frame_ = std::make_shared<DataFrame>();
  route_target_sub_ = node.subscribe("/move_base_simple/goal", 10,
                                     &PlanningCore::NewRouteCallBack, this);
  joy_sub_ = node.subscribe("/joy", 10, &PlanningCore::JoyCallBack, this);

  // init predictor
  mock_predictor_ = std::make_unique<ConstVelPredictor>(6, 0.2);
  mock_predictor_->Init();

  // init planner
  planner_ = std::make_unique<GPPlanner>();
  planner_->Init();

  // 初始化不确定性估计器
  uncertainty_estimator_.Init(nh);
  navigation_map_.SetUncertaintyEstimator(&uncertainty_estimator_);
}

void PlanningCore::Run(const ros::TimerEvent&) {
  if (!UpdateDataFrame()) {
    LOG_EVERY_N(ERROR, 20) << "update data frame failed, give up planning";
    return;
  }

  mock_predictor_->GeneratePrediction(&data_frame_->obstacles);
  navigation_map_.Update(data_frame_);

  if (!random_drive_mode_) {
    // point-to-point mode
    std::lock_guard<std::mutex> lock(route_mutex_);
    if (has_new_route_) {
      navigation_map_.CreateTask(route_goal_);
      has_new_route_ = false;
    }
    if (!navigation_map_.HasActiveTask()) {
      LOG_EVERY_N(INFO, 20) << "no active task";
      navigation_map_.mutable_trajectory()->clear();
      simulator_->SetTrajectory(navigation_map_.trajectory());
      return;
    }
  } else {
    // random driving mode
    if (has_new_route_) {
      if (!navigation_map_.RandomlyUpdateRoute()) {
        navigation_map_.mutable_trajectory()->clear();
        simulator_->SetTrajectory(navigation_map_.trajectory());
        return;
      }
    } else {
      navigation_map_.mutable_trajectory()->clear();
      simulator_->SetTrajectory(navigation_map_.trajectory());
      return;
    }
  }

  if (suggest_lane_change_) {
    if (navigation_map_.SuggestLaneChange(
            static_cast<hdmap::LaneSegmentBehavior>(suggest_lane_change_))) {
      suggest_lane_change_ = 0;
    }
  }

  navigation_map_.UpdateReferenceLine();

  LOG(INFO) << "================== Planning cycle start ==================";
  LOG(INFO) << "Navigation map uncertainty:\n" 
              << navigation_map_.static_obstacle_uncertainty().position_covariance;

  // 在规划前更新静态障碍物不确定性
  planner_->SetStaticObstacleUncertainty(
      navigation_map_.static_obstacle_uncertainty());
  
  LOG(INFO) << "Uncertainty passed to planner";

  TIC;
  planner_->PlanOnce(&navigation_map_);
  TOC("PlaneOnce");
  // 规划完成后,通过simulator传递轨迹
  simulator_->SetTrajectory(navigation_map_.trajectory());
}

void PlanningCore::NewRouteCallBack(const geometry_msgs::PoseStamped& goal) {
  std::lock_guard<std::mutex> lock(route_mutex_);
  route_goal_ = goal;
  has_new_route_ = true;
}

void PlanningCore::JoyCallBack(const sensor_msgs::Joy& joy) {
  if (joy.buttons[2] == 1) {
    LOG(INFO) << "Suggest left lane change by joy";
    suggest_lane_change_ = 1;
  } else if (joy.buttons[1] == 1) {
    LOG(INFO) << "Suggest right lane change by joy";
    suggest_lane_change_ = 2;
  } else if (joy.buttons[3] == 1) {
    LOG(INFO) << "Increase reference speed";
    navigation_map_.AdjustReferenceSpeed(1);
  } else if (joy.buttons[0] == 1) {
    LOG(INFO) << "Increase reference speed";
    navigation_map_.AdjustReferenceSpeed(-1);
  } else if (joy.buttons[5] > 0) {
    LOG(INFO) << "Add virtual Obstacles";
    navigation_map_.RandomlyAddVirtualObstacles(joy.buttons[5]);
  }
}

bool PlanningCore::UpdateDataFrame() {
  if (!simulator_->UpdateEgoState(&data_frame_->state)) return false;
  if (!simulator_->UpdatePerceptionResults(&data_frame_->obstacles)) {
    return false;
  }

  // 更新每个障碍物的不确定性估计
  for (auto& obstacle : data_frame_->obstacles) {
    uncertainty_estimator_.EstimateObstacleUncertainty(
        data_frame_->state, &obstacle);
  }

  return true;
}
}  // namespace planning
