/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "gp_planner/gp_planner.h"

#include <glog/logging.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <visualization_msgs/MarkerArray.h>

#include "common/smoothing/osqp_spline2d_solver.h"
#include "common/utils/color_map.h"
#include "common/utils/io_utils.h"
#include "gp_planner/gp/gp_incremental_path_planner.h"
#include "gp_planner/st_plan/st_graph.h"
#include "planning_core/planning_common/planning_visual.h"

namespace planning {

void GPPlanner::Init() {
  ros::NodeHandle node;
  trajectory_pub_ =
      node.advertise<visualization_msgs::MarkerArray>("/gp_path", 1);
  critical_obstacle_pub_ =
      node.advertise<visualization_msgs::MarkerArray>("/critical_obstacles", 1);
  target_lane_pub_ = node.advertise<visualization_msgs::MarkerArray>(
      "/behavior_target_lane", 1);

  vehicle_param_ = VehicleInfo::Instance().vehicle_param(); //初始化车辆参数
}

void GPPlanner::PlanOnce(NavigationMap* navigation_map_) {
  const auto& ego_state = navigation_map_->ego_state();
  const auto& reference_lines = navigation_map_->reference_lines();
  const auto& virtual_obstacles = navigation_map_->virtual_obstacles();
  const auto& obstacles = navigation_map_->obstacles();
  reference_speed_ = navigation_map_->reference_speed();

  std::vector<std::pair<hdmap::LaneSegmentBehavior, common::Trajectory>>
      trajectory_candidate; //用于存储可能的候选轨迹

  //遍历所有的参考线，调用 PlanWithGPIR() 方法进行路径规划。
  //如果规划成功，将生成的轨迹存储到 trajectory_candidate 中，否则记录日志
  int count = 0;
  for (const auto& reference_line : reference_lines) {
    trajectory_index_ = count;
    common::Trajectory trajectroy;
    //PlanOnce() 调用 PlanWithGPIR() 来进行实际的路径规划。
    if (!PlanWithGPIR(ego_state, reference_line, obstacles, virtual_obstacles,
                      navigation_map_->trajectory(), &trajectroy)) {
      LOG(ERROR) << "[Plan]: planning for "
                 << ToString(reference_line.behavior()) << " failed";
    } else {
      trajectory_candidate.emplace_back(
          std::make_pair(reference_line.behavior(), trajectroy));
    }
    ++count;
  }

  //选择最优轨迹：如果有有效的候选轨迹，则选择第一条轨迹作为最终的输出，并更新车辆的车道保持状态 (SetPlannerLCFeedback())。否则记录日志并清空轨迹
  if (!trajectory_candidate.empty()) {
    *navigation_map_->mutable_trajectory() =
        trajectory_candidate.front().second;
    navigation_map_->SetPlannerLCFeedback(trajectory_candidate.front().first !=
                                          hdmap::LaneSegmentBehavior::kKeep);
    last_behavior_ = trajectory_candidate.front().first;
  } else {
    LOG(ERROR) << "[Plan]: cannot find valid trajectory, planning failed";
    navigation_map_->mutable_trajectory()->clear();
  }

  VisualizeTrajectory(trajectory_candidate);
  VisualizeTargetLane(reference_lines[std::min(
      reference_lines.size() - 1,
      reference_lines.size() - trajectory_candidate.size())]);
}

bool GPPlanner::ProcessObstacles(const std::vector<Obstacle>& raw_obstacles,
                                 const ReferenceLine& reference_line,
                                 std::vector<Obstacle>* cirtical_obstacles) {
  cirtical_obstacles->clear();
  //ego_box_：使用车辆的参数和状态信息创建 ego_box_，即当前车辆的 2D 边界框，用于进行碰撞检测​(gp_planner)。
  ego_box_ = common::Box2D(state_.position, vehicle_param_.length,
                           vehicle_param_.width, state_.heading,
                           vehicle_param_.height);
  const auto& ref_lane_list = reference_line.lane_id_list();

  //遍历所有障碍物，判断与车辆边界的重叠情况，如果发生重叠则记录碰撞信息。然后，通过参考线的位置，进一步判断障碍物是否处于规划的范围内（ROI）
  for (const auto& obstacle : raw_obstacles) {
    if (ego_box_.HasOverlapWith(obstacle.BoundingBox())) {
      LOG(ERROR) << "collision detected!";
      return false;
    }

    std::vector<int> intersect_lane;
    const auto& obstacle_lane_list = obstacle.lane_id_list();
    std::set_intersection(obstacle_lane_list.begin(), obstacle_lane_list.end(),
                          ref_lane_list.begin(), ref_lane_list.end(),
                          std::back_inserter(intersect_lane));

    auto proj = reference_line.GetRoughProjection(obstacle.state().position);

    bool outside_roi = proj.s < -30 || proj.s > reference_line.length() ||
                       std::fabs(proj.d) > lateral_critical_thereshold_;
    bool intersect_safe = intersect_lane.empty();

    if (outside_roi && intersect_safe) continue;
    if (!obstacle.is_static()) {
      cirtical_obstacles->emplace_back(obstacle);
    }
  }
  return true;
}

bool GPPlanner::PlanWithGPIR(
    const common::State& ego_state, const ReferenceLine& reference_line,
    const std::vector<Obstacle>& dynamic_agents,
    const std::vector<Eigen::Vector2d>& virtual_obstacles,
    const common::Trajectory& last_trajectory, common::Trajectory* trajectory) {
  common::Timer timer; //用于跟踪函数中各部分的运行时间
  time_consuption_ = TimeConsumption();

  const double length = reference_line.length();//获取参考线的长度

  //调用障碍物处理函数，将动态障碍物分类为关键障碍物（可能影响车辆行驶）和非关键障碍物
  std::vector<Obstacle> cirtical_agents;
  ProcessObstacles(dynamic_agents, reference_line, &cirtical_agents);

  timer.Start();
  //创建占据图（占据网格）用于表示路径周围的可行区域。该网格的大小由参考线长度及其横向边界来决定，每个网格单元的大小为 0.1m x 0.1m。
  OccupancyMap occupancy_map({0, -5}, {std::ceil(length / 0.1) + 100, 100},
                             {0.1, 0.1});

  //处理虚拟障碍物：遍历虚拟障碍物，将它们投影到参考线中，并在占据图中填充这些障碍物位置，生成障碍物提示位置。
  std::vector<double> obstacle_location_hint;
  for (const auto& point : virtual_obstacles) {
    auto proj = reference_line.GetProjection(point);
    occupancy_map.FillCircle(Eigen::Vector2d(proj.s, proj.d), 0.2);
    obstacle_location_hint.emplace_back(proj.s);
  }
  //SignedDistanceField2D：使用占据图构建一个签名距离场 (SDF)，SDF 用于路径生成中的距离约束。UpdateSDF() 计算并更新距离场。
  auto sdf = std::make_shared<SignedDistanceField2D>(std::move(occupancy_map));
  sdf->UpdateSDF();
  time_consuption_.sdf = timer.EndThenReset();

  //将当前车辆状态从 Cartesian 坐标转换为 Frenet 坐标系
  common::FrenetState frenet_state;
  reference_line.ToFrenetState(ego_state, &frenet_state);
  //基于历史轨迹的状态修正：如果上一次的轨迹存在且与当前状态相匹配，将上一时刻的状态应用到当前状态，以实现平滑的轨迹过渡
  if (!last_trajectory.empty()) {
    auto last_state = last_trajectory.GetNearestState(ego_state.position);
    if (std::fabs(last_state.frenet_d[0] - frenet_state.d[0]) < 1.0) {
      reference_line.ToFrenetState(last_state, &frenet_state);
      frenet_state.d = last_state.frenet_d;
      // frenet_state.s = last_state.frenet_s;
    }
  }

  //创建增量路径规划器，并生成初始路径。
  //路径生成基于高斯过程，目标是在考虑障碍物和距离场的情况下，生成一条无碰撞且符合曲率约束的路径。
  //GPPath类：用于存储路径节点并通过插值器生成平滑路径，是高斯过程路径的表示形式。
  //GPIncrementalPathPlanner类：用于路径生成和增量优化。它使用高斯过程生成初始路径，并在环境变化时进行增量优化，保证路径的无碰撞性和平滑性。
  GPPath gp_path;
  GPIncrementalPathPlanner gp_path_planner(sdf);
  if (!gp_path_planner.GenerateInitialGPPath(reference_line, frenet_state, 100,
                                             obstacle_location_hint,
                                             &gp_path)) {
    LOG(ERROR) << "[GPPlanner]: fail to generate initla path";
    return false;
  }
  time_consuption_.init_path = timer.EndThenReset();

  //StGraph 和速度规划：将车辆状态重新投影到 Frenet 坐标系中，创建 StGraph 对象。
  reference_line.ToFrenetState(ego_state, &frenet_state);
  StGraph st_graph(frenet_state.s);
  st_graph.SetReferenceSpeed(reference_speed_);
  st_graph.BuildStGraph(cirtical_agents, gp_path); //构建 ST 图
  if (!st_graph.SearchWithLocalTruncation(13, nullptr)) {
    LOG(ERROR) << "[StGraph]: fail to find initial speed profile"; //在 ST 图中进行搜索，找到初步的速度曲线。
    return false;
  }

  //在 ST 图中为 gp_path 生成速度曲线
  if (!st_graph.GenerateInitialSpeedProfile(gp_path)) {
    LOG(ERROR) << "[StGraph]: fail to optimize inital speed profile";
    return false;
  }
  time_consuption_.init_st = timer.EndThenReset();

  //增量优化
  //IsTrajectoryFeasible()检查生成的轨迹是否满足所有约束，包括无碰撞、符合曲率约束
  //使用 UpdateGPPath() 方法进行增量路径优化。如果发现轨迹不可行，则进行迭代调整，直到达到最大迭代次数 (max_iter) 或轨迹可行。
  int iter_count = 0;
  vector_Eigen3d invalid_lat_frenet_s;
  while (trajectory_index_ < 1 &&
         !st_graph.IsTrajectoryFeasible(gp_path, &invalid_lat_frenet_s)) {
    gp_path_planner.UpdateGPPath(reference_line, invalid_lat_frenet_s,
                                 &gp_path);
    // gp_path_planner.UpdateGPPathNonIncremental(reference_line,
    //                                            invalid_lat_frenet_s,
    //                                            &gp_path);
    // st_graph.UpdateSpeedProfile(gp_path);
    if (iter_count++ == max_iter) {
      LOG(WARNING) << "[GPIR]: reach maximum iterations";
      break;
    }
  }
  if (iter_count != 0) {
    time_consuption_.refinement = timer.EndThenReset();
  }
  st_graph.GenerateTrajectory(reference_line, gp_path, trajectory);//调用 GenerateTrajectory() 方法基于路径和速度曲线生成最终的轨迹。

  VisualizeCriticalObstacle(cirtical_agents);
  return true;
}

void GPPlanner::VisualizeTrajectory(
    const std::vector<std::pair<hdmap::LaneSegmentBehavior,
                                common::Trajectory>>& trajectory_candidates) {
  visualization_msgs::MarkerArray markers;
  visualization_msgs::Marker shape;

  constexpr double sample_dis = 3;
  int id_count = 0;
  double alpha = 1.0;
  common::Color line_color, node_color;
  for (int i = 0; i < trajectory_candidates.size(); ++i) {
    visualization_msgs::Marker line, node;
    if (i == 0) {
      alpha = 1.0;
      line_color = common::Color::kOrangeRed;
      node_color = common::Color::kBlue;
    } else {
      alpha = 0.5;
      line_color = common::Color::kGrey;
      node_color = common::Color::kBlack;
    }

    PlanningVisual::FillHeader(&line.header);
    PlanningVisual::SetScaleAndColor({0.15, 0.15, 0.15}, line_color, &line,
                                     alpha);
    line.id = id_count++;
    line.type = visualization_msgs::Marker::LINE_STRIP;
    line.action = visualization_msgs::Marker::MODIFY;
    line.pose.orientation.w = 1.0;
    line.lifetime = ros::Duration(0.15);

    PlanningVisual::FillHeader(&node.header);
    PlanningVisual::SetScaleAndColor({0.5, 0.5, 0.1}, node_color, &node, alpha);
    node.type = visualization_msgs::Marker::CYLINDER;
    node.action = visualization_msgs::Marker::MODIFY;
    node.pose.orientation.w = 1;
    node.pose.orientation.w = 1.0;
    node.lifetime = ros::Duration(0.15);

    if (i == 0) {
      PlanningVisual::FillHeader(&shape.header);
      PlanningVisual::SetScaleAndColor({2.1, 0, 0}, common::kGold, &shape, 0.2);
      shape.id = id_count++;
      shape.type = visualization_msgs::Marker::LINE_STRIP;
      shape.action = visualization_msgs::Marker::MODIFY;
      shape.pose.orientation.w = 1;
      shape.lifetime = ros::Duration(0.15);
    }

    geometry_msgs::Point point;
    double start_s = trajectory_candidates[i].second.front().s;
    for (const auto& traj_point : trajectory_candidates[i].second) {
      if (traj_point.s - start_s > sample_dis) {
        start_s = traj_point.s;
        node.id = id_count++;
        node.pose.position.x = traj_point.position.x();
        node.pose.position.y = traj_point.position.y();
        node.pose.position.z = 0.2;
        markers.markers.push_back(node);
      }
      point.x = traj_point.position.x();
      point.y = traj_point.position.y();
      point.z = 0.2;
      line.points.push_back(point);
      if (i == 0) {
        shape.points.push_back(point);
      }
    }
    markers.markers.push_back(line);
  }
  markers.markers.push_back(shape);
  trajectory_pub_.publish(markers);
}

void GPPlanner::VisualizeCriticalObstacle(
    const std::vector<Obstacle>& critical_obstacles) {
  visualization_msgs::MarkerArray markers;
  visualization_msgs::Marker marker;
  marker.lifetime = ros::Duration(0.15);

  geometry_msgs::Point p;
  int id_count = 0;
  for (const auto& obs : critical_obstacles) {
    PlanningVisual::Get2DBoxMarker(obs.state().position, obs.width() + 0.3,
                                   obs.length() + 0.3, obs.state().heading,
                                   common::Color::kRed, {0.3, 0, 0}, &marker);
    marker.id = id_count++;
    markers.markers.emplace_back(marker);
  }

  critical_obstacle_pub_.publish(markers);
}

void GPPlanner::VisualizeTargetLane(const ReferenceLine& reference_line) {
  visualization_msgs::MarkerArray markers;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::MODIFY;
  marker.pose.orientation.w = 1.0;

  constexpr double step_length = 5.0;
  constexpr double angle = M_PI_4;
  double length = 0.75 / std::sin(angle);

  auto rotate_vector = [&length](const Eigen::Vector3d& base,
                                 const double angle) {
    geometry_msgs::Point point;
    auto rotated_vector =
        base.topRows(2) + length * Eigen::Vector2d(std::cos(base(2) + angle),
                                                   std::sin(base(2) + angle));
    point.x = rotated_vector.x();
    point.y = rotated_vector.y();
    return point;
  };

  marker.id = 0;
  for (double s = 0; s < reference_line.length(); s += step_length) {
    auto bh = reference_line.behavior();
    if (bh == hdmap::LaneSegmentBehavior::kKeep) {
      PlanningVisual::SetScaleAndColor({0.4, 0, 0}, common::Color::kGreen,
                                       &marker);
    } else {
      PlanningVisual::SetScaleAndColor({0.4, 0, 0}, common::Color::kMagenta,
                                       &marker);
    }
    auto se2 = reference_line.GetSE2(s);
    geometry_msgs::Point origin;
    origin.x = se2.x();
    origin.y = se2.y();
    auto left = rotate_vector(se2, M_PI - angle);
    marker.points.push_back(origin);
    marker.points.push_back(left);
    auto right = rotate_vector(se2, -M_PI + angle);
    marker.points.push_back(origin);
    marker.points.push_back(right);
  }
  markers.markers.push_back(marker);

  target_lane_pub_.publish(markers);
}

}  // namespace planning
