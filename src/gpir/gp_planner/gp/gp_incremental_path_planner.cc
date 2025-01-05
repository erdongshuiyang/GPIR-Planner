/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "gp_planner/gp/gp_incremental_path_planner.h"

#include <queue>

#include "common/utils/math.h"
#include "common/utils/timer.h"
#include "gp_planner/gp/factors/gp_interpolate_kappa_limit_factor.h"
#include "gp_planner/gp/factors/gp_interpolate_obstacle_factor.h"
#include "gp_planner/gp/factors/gp_kappa_limit_factor.h"
#include "gp_planner/gp/factors/gp_lat_acc_limit_factor.h"
#include "gp_planner/gp/factors/gp_obstacle_factor.h"
#include "gp_planner/gp/factors/gp_prior_factor.h"
#include "gp_planner/initializer/gp_initializer.h"
#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"
#include "gtsam/nonlinear/LevenbergMarquardtParams.h"
#include "gtsam/nonlinear/Symbol.h"

#include "gp_planner/gp/factors/gp_perception_uncertainty_factor.h"
#include "gp_planner/gp/factors/static_obstacle_uncertainty_factor.h"

namespace planning {

using common::NormalizeAngle;
using gtsam::Vector3;
using gtsam::noiseModel::Diagonal;
using gtsam::noiseModel::Isotropic;
using PriorFactor3 = gtsam::PriorFactor<Vector3>;

constexpr double kEpsilon = 10;  //1.6
constexpr double kQc = 0.1;

bool GPIncrementalPathPlanner::DecideInitialPathBoundary(
    const Eigen::Vector2d& init_pos, const double init_angle,
    const std::vector<double>& obstacle_location_hint,
    const std::vector<std::vector<std::pair<double, double>>>& boundaries,
    std::vector<double>* lb, std::vector<double>* ub) {
  if (boundaries.empty()) return true;
  CHECK_EQ(obstacle_location_hint.size(), boundaries.size());

  struct PathCandidate {
    int current_idx;
    double angle;
    Eigen::Vector2d pos;
    std::vector<int> selections;
    double cost = 0;

    std::unique_ptr<PathCandidate> Expand(int selection) {
      auto expansion = std::make_unique<PathCandidate>();
      expansion->current_idx = current_idx + 1;
      expansion->selections = selections;
      expansion->selections.emplace_back(selection);
      expansion->cost = cost;
      return expansion;
    }
  };

  int max_idx = boundaries.size() - 1;
  std::vector<std::unique_ptr<PathCandidate>> resutls;

  constexpr double weight_width = 0.4;
  constexpr double weight_dis = 0.0;
  constexpr double weight_angle = 0.0;

  // use BFS to find best initial path topology
  std::queue<std::unique_ptr<PathCandidate>> bfs_queue;

  auto init_node = std::make_unique<PathCandidate>();
  init_node->pos = init_pos;
  init_node->current_idx = -1;
  bfs_queue.push(std::move(init_node));

  while (!bfs_queue.empty()) {
    auto candidate = std::move(bfs_queue.front());
    bfs_queue.pop();
    const int current_idx = candidate->current_idx;
    if (current_idx == max_idx) {
      resutls.emplace_back(std::move(candidate));
      continue;
    }
    for (int j = 0; j < boundaries[current_idx + 1].size(); ++j) {
      double width = boundaries[current_idx + 1][j].second -
                     boundaries[current_idx + 1][j].first;
      if (width < ego_width_) continue;  // physically impossible
      auto expansion = candidate->Expand(j);
      expansion->pos =
          Eigen::Vector2d(obstacle_location_hint[current_idx + 1],
                          0.5 * (boundaries[current_idx + 1][j].first +
                                 boundaries[current_idx + 1][j].second));
      auto direction_vector = expansion->pos - candidate->pos;
      expansion->angle = std::atan2(direction_vector.y(), direction_vector.x());
      expansion->cost +=
          weight_angle *
          std::fabs(NormalizeAngle(expansion->angle - candidate->angle));
      if (width < ego_width_ * 2) {
        expansion->cost += weight_width * (ego_width_ * 2 - width);
      }
      bfs_queue.push(std::move(expansion));
    }
  }

  if (resutls.empty()) {
    LOG(ERROR) << "Cannot find valid initial path, the road might be blocked";
    return false;
  }

  std::sort(resutls.begin(), resutls.end(),
            [](const std::unique_ptr<PathCandidate>& p1,
               const std::unique_ptr<PathCandidate>& p2) {
              return p1->cost <= p2->cost;
            });

  const auto& best_path = resutls.front();
  for (int i = 0; i < boundaries.size(); ++i) {
    int selection = best_path->selections[i];
    if (selection == 0) {
      lb->emplace_back(boundaries[i][selection].first);
      ub->emplace_back(boundaries[i][selection].second - ego_half_width_);
    } else if (selection == boundaries[i].size()) {
      lb->emplace_back(boundaries[i][selection].first + ego_half_width_);
      ub->emplace_back(boundaries[i][selection].second);
    } else {
      lb->emplace_back(boundaries[i][selection].first + ego_half_width_);
      ub->emplace_back(boundaries[i][selection].second - ego_half_width_);
    }
  }

  return true;
}

bool GPIncrementalPathPlanner::GenerateInitialGPPath(
    const ReferenceLine& reference_line,
    const common::FrenetState& initial_state, const double length,
    const std::vector<double>& obstacle_location_hint,
    const std::vector<Obstacle>& obstacles,  // 新增参数
    GPPath* gp_path) {
  
  LOG(INFO) << "=== Path optimization start ===";

  // 保存参考线指针
  reference_line_ = &reference_line;    

  TIC;
  // static auto sigma_initial = Isotropic::Sigma(3, 0.001);
  // static auto sigma_goal = Diagonal::Sigmas(Vector3(1, 0.1, 0.1));
  // static auto sigma_reference = Diagonal::Sigmas(Vector3(30, 1e9, 1e9));

  // 修改初始权重，增加路径稳定性
  static auto sigma_initial = Isotropic::Sigma(3, 0.01); // 改为0.01
  static auto sigma_goal = Diagonal::Sigmas(Vector3(2.0, 0.2, 0.2)); // 增大权重
  static auto sigma_reference = Diagonal::Sigmas(Vector3(300, 1e9, 1e9)); //30

  interval_ = length / (num_of_nodes_ - 1);

  if (!graph_.empty()) {
    graph_.resize(0);
    node_locations_.clear();
  }

  Vector3 x0 = initial_state.d;
  Vector3 xn(0, 0, 0);
  Vector3 x_ref(0, 0, 0);

  double start_s = initial_state.s[0];

   // 减小插值点数量
  const int interpolate_num = 10; // 原为10
  const double tau = interval_ / (interpolate_num + 1);

  graph_.reserve(num_of_nodes_);
  double kappa_r = 0.0, dkappa_r = 0.0, current_s = 0.0;
  for (int i = 0; i < num_of_nodes_; ++i) {
    current_s = start_s + i * interval_;
    node_locations_.emplace_back(current_s);

    gtsam::Key key = gtsam::Symbol('x', i);
    if (i == 0) graph_.add(PriorFactor3(key, x0, sigma_initial));
    if (i == num_of_nodes_ - 1) graph_.add(PriorFactor3(key, xn, sigma_goal));
    if (i > 0) {
      gtsam::Key last_key = gtsam::Symbol('x', i - 1);
      reference_line.GetCurvature(current_s, &kappa_r, &dkappa_r);

      graph_.add(GPPriorFactor(last_key, key, interval_, kQc));

      // if (current_s > 0) {
      //   graph_.add(PriorFactor3(key, x_ref, sigma_reference));
      // }
      if (current_s > initial_state.s[1] * 3) {
        graph_.add(PriorFactor3(key, x_ref, sigma_reference));
      }

      // // 添加不确定性因子
      // AddUncertaintyFactors(obstacles, current_s, key);

      // 添加静态障碍物不确定性因子
      graph_.add(
        StaticObstacleUncertaintyFactor(key, sdf_, static_obstacle_uncertainty_,
                                        0.95, kEpsilon, current_s, kappa_r));

      // graph_.add(
      //     GPObstacleFactor(key, sdf_, 0.1, kEpsilon, current_s, kappa_r));
      if (enable_curvature_constraint_) {
        graph_.add(GPKappaLimitFactor(key, 0.01, kappa_r, dkappa_r,
                                      kappa_limit_, current_s));
      }

      for (int j = 0; j < interpolate_num; ++j) {
        // graph_.add(GPInterpolateObstacleFactor(
        //     last_key, key, sdf_, 0.1, kEpsilon, start_s + interval_ * (i - 1),
        //     kQc, interval_, tau * (j + 1), kappa_r));
        if (enable_curvature_constraint_) {
          graph_.add(GPInterpolateKappaLimitFactor(
              last_key, key, 0.01, kQc, interval_, tau * (j + 1), kappa_r,
              dkappa_r, kappa_limit_));
        }
      }
    }
  }

  std::vector<double> lb, ub;
  double init_kappa = reference_line.GetCurvature(initial_state.s[0]);
  double init_angle =
      std::atan2(initial_state.d[1], 1 - init_kappa * initial_state.d[0]);
  std::vector<std::vector<std::pair<double, double>>> boundaries;
  sdf_->mutable_occupancy_map()->SearchForVerticalBoundaries(
      obstacle_location_hint, &boundaries);
  DecideInitialPathBoundary(
      Eigen::Vector2d(initial_state.s[0], initial_state.d[0]), init_angle,
      obstacle_location_hint, boundaries, &lb, &ub);

  GPInitializer initializer;
  vector_Eigen3d initial_path;

  if (!initializer.GenerateInitialPath(x0, xn, node_locations_,
                                       obstacle_location_hint, lb, ub,
                                       &initial_path)) {
    LOG(ERROR) << "Generate initial path for GP planner failed";
    return false;
  }

  gtsam::Values init_values;
  for (int i = 0; i < node_locations_.size(); ++i) {
    gtsam::Key key = gtsam::symbol('x', i);
    init_values.insert<gtsam::Vector3>(key, initial_path[i]);
  }

  // 打印优化前的代价值
  double initial_error = graph_.error(init_values);
  LOG(INFO) << "Initial error: " << initial_error;

  // gtsam::LevenbergMarquardtParams param;
  // param.setlambdaInitial(100.0);
  // param.setMaxIterations(50);
  // // param.setAbsoluteErrorTol(5e-4);
  // // param.setRelativeErrorTol(0.01);
  // // param.setErrorTol(1.0);
  // // param.setVerbosity("ERROR");

 // 图优化器参数调整 
  gtsam::LevenbergMarquardtParams param;
  // param.setVerbosityLM("SUMMARY"); // 打印优化过程
  param.setlambdaInitial(1000.0);  // 增大初始lambda
  param.setMaxIterations(30);       // 减少最大迭代次数
  param.setRelativeErrorTol(1e-4);  // 相对误差阈值
  param.setAbsoluteErrorTol(1e-4);  // 绝对误差阈值

  gtsam::LevenbergMarquardtOptimizer opt(graph_, init_values, param);
  map_result_ = opt.optimize();

  // 打印优化结果
  double final_error = graph_.error(map_result_);
  LOG(INFO) << "Final error: " << final_error;
  LOG(INFO) << "Optimization iterations: " << opt.iterations();

  *gp_path =
      GPPath(num_of_nodes_, start_s, interval_, length, kQc, &reference_line);
  auto gp_path_nodes = gp_path->mutable_nodes();
  for (size_t i = 0; i < map_result_.size(); ++i) {
    gp_path_nodes->emplace_back(
        map_result_.at<gtsam::Vector3>(gtsam::Symbol('x', i)));
  }
  TOC("GP path Generation");

  if (enable_incremental_refinemnt_) {
    // // init isam2
    // isam2_ = gtsam::ISAM2(
    //     gtsam::ISAM2Params(gtsam::ISAM2GaussNewtonParams(), 1e-3, 1));

  gtsam::ISAM2Params isam2_params;
  gtsam::ISAM2GaussNewtonParams gnParams;
  gnParams.setWildfireThreshold(0.001);
  isam2_params.optimizationParams = gnParams;
  isam2_params.relinearizeThreshold = 0.1;
  isam2_params.relinearizeSkip = 1;
  isam2_params.enableDetailedResults = true;
  isam2_params.cacheLinearizedFactors = true;
  isam2_params.evaluateNonlinearError = true;
  isam2_params.factorization = gtsam::ISAM2Params::CHOLESKY;
 
 isam2_ = gtsam::ISAM2(isam2_params);
    isam2_.update(graph_, map_result_);
    // map_result_ = isam2_.calculateEstimate();
  }


  // 生成路径后进行安全性验证
  // if (!ValidatePathSafety(*gp_path, obstacles)) {
  //   LOG(WARNING) << "Generated path fails safety check";
  //   return false;
  // }

  return true;
}

bool GPIncrementalPathPlanner::UpdateGPPath(const ReferenceLine& reference_line,
                                            const vector_Eigen3d& frenet_s,
                                            const std::vector<Obstacle>& obstacles,  // 新增参数
                                            GPPath* gp_path) {

  // 更新参考线指针
  reference_line_ = &reference_line;

  for (int i = 0; i < frenet_s.size(); ++i) {
    int index =
        std::floor((frenet_s[i](0) - node_locations_.front()) / interval_);
    if (index < 0) continue;
    gtsam::Symbol begin_node('x', index), end_node('x', index + 1);
    graph_.add(GPLatAccLimitFactor(begin_node, end_node, kQc, interval_,
                                   frenet_s[i](0) - node_locations_[index],
                                   frenet_s[i](1), frenet_s[i](2), 2.5, 0.1));

    // // 添加不确定性因子
    // AddUncertaintyFactors(obstacles, node_locations_[index + 1], end_node);

    // 添加静态障碍物不确定性因子
    // double current_s = node_locations_[index + 1];
    // double kappa_r, dkappa_r;
    // reference_line.GetCurvature(current_s, &kappa_r, &dkappa_r);
    
    // graph_.add(
    //     StaticObstacleUncertaintyFactor(end_node, sdf_, static_obstacle_uncertainty_,
    //                                    0.1, kEpsilon, current_s, kappa_r));
  }
  isam2_.update(graph_);
  map_result_ = isam2_.calculateEstimate();
  gp_path->UpdateNodes(map_result_);

  // 验证更新后的路径安全性
  // if (!ValidatePathSafety(*gp_path, obstacles)) {
  //   LOG(WARNING) << "Updated path fails safety check";
  //   return false;
  // }

  return true;
}


// bool GPIncrementalPathPlanner::UpdateGPPath(
//     const ReferenceLine& reference_line,
//     const vector_Eigen3d& frenet_s,
//     const std::vector<Obstacle>& obstacles,  
//     GPPath* gp_path) {

//   reference_line_ = &reference_line;

//   // 限制每次更新的最大状态数
//   const int kMaxUpdates = 5;
//   int update_count = 0;

//   for (int i = 0; i < frenet_s.size() && update_count < kMaxUpdates; ++i) {
//     int index = std::floor((frenet_s[i](0) - node_locations_.front()) / interval_);
//     if (index < 0) continue;
    
//     // 添加增量优化保护
//     if (index >= num_of_nodes_ - 1) continue;

//     gtsam::Symbol begin_node('x', index), end_node('x', index + 1);
    
//     // 添加横向加速度约束
//     graph_.add(GPLatAccLimitFactor(begin_node, end_node, kQc, interval_,
//                                    frenet_s[i](0) - node_locations_[index],
//                                    frenet_s[i](1), frenet_s[i](2), 2.5, 0.1));

//     // 添加静态障碍物不确定性因子
//     double current_s = node_locations_[index + 1];
//     double kappa_r, dkappa_r;
//     reference_line.GetCurvature(current_s, &kappa_r, &dkappa_r);
    
//     graph_.add(
//         StaticObstacleUncertaintyFactor(end_node, sdf_, static_obstacle_uncertainty_,
//                                        0.1, kEpsilon, current_s, kappa_r));
    
//     update_count++;
//   }

//   // 增量优化参数调整
//   gtsam::ISAM2Params update_params;
//   update_params.relinearizeThreshold = 0.1;
//   update_params.relinearizeSkip = 1;
  
//   isam2_.update(graph_, gtsam::ISAM2UpdateParams(update_params));
//   map_result_ = isam2_.calculateEstimate();
//   gp_path->UpdateNodes(map_result_);

//   return true;
// }

bool GPIncrementalPathPlanner::UpdateGPPathNonIncremental(
    const ReferenceLine& reference_line, const vector_Eigen3d& frenet_s,
    const std::vector<Obstacle>& obstacles,  // 添加障碍物参数
    GPPath* gp_path) {
  
  // 更新参考线指针
  reference_line_ = &reference_line;

  for (int i = 0; i < frenet_s.size(); ++i) {
    int index =
        std::floor((frenet_s[i](0) - node_locations_.front()) / interval_);
    if (index < 0) continue;
    gtsam::Symbol begin_node('x', index), end_node('x', index + 1);
    graph_.add(GPLatAccLimitFactor(begin_node, end_node, kQc, interval_,
                                   frenet_s[i](0) - node_locations_[index],
                                   frenet_s[i](1), frenet_s[i](2), 2.5, 0.1));
    // // 添加不确定性因子
    // AddUncertaintyFactors(obstacles, node_locations_[index + 1], end_node);

    // 添加静态障碍物不确定性因子
    // double current_s = node_locations_[index + 1];
    // double kappa_r, dkappa_r;
    // reference_line.GetCurvature(current_s, &kappa_r, &dkappa_r);
    
    // graph_.add(
    //     StaticObstacleUncertaintyFactor(end_node, sdf_, static_obstacle_uncertainty_,
    //                                    0.1, kEpsilon, current_s, kappa_r));
  }

  gtsam::LevenbergMarquardtParams param;
  param.setlambdaInitial(100.0);
  param.setAbsoluteErrorTol(1e-5);
  // param.setVerbosity("ERROR");
  gtsam::LevenbergMarquardtOptimizer opt(graph_, map_result_, param);

  map_result_ = opt.optimize();
  gp_path->UpdateNodes(map_result_);


  // 验证更新后的路径安全性
  // if (!ValidatePathSafety(*gp_path, obstacles)) {
  //   LOG(WARNING) << "Non-incremental updated path fails safety check";
  //   return false;
  // }

  return true;
}

int GPIncrementalPathPlanner::FindLocationIndex(const double s) {
  int index = std::distance(node_locations_.begin(),
                            std::lower_bound(node_locations_.begin(),
                                             node_locations_.end(), s)) -
              1;
  return std::max(0, index);
}

// void GPIncrementalPathPlanner::AddUncertaintyFactors(
//     const std::vector<Obstacle>& obstacles,
//     double current_s,
//     gtsam::Key key) {
  
//   // 确保参考线指针有效
//   CHECK(reference_line_ != nullptr) << "Reference line not set";

//   // 更新曲率
//   double dkappa_r{0.0};
//   reference_line_->GetCurvature(current_s, &kappa_r_, &dkappa_r);

//   // 遍历所有障碍物，为每个障碍物添加不确定性因子
//   for (const auto& obstacle : obstacles) {
//     // 检查障碍物是否在当前位置的影响范围内
//     double distance = std::abs(obstacle.state().s - current_s);
//     if (distance > 30.0) continue;  // 忽略太远的障碍物

//     // 获取障碍物的不确定性信息
//     const auto& uncertainty = obstacle.perception_uncertainty();
    
//     // 如果障碍物有有效的不确定性信息，使用它的不确定性
//     if (uncertainty.IsValid()) {
//       graph_.add(
//           GPPerceptionUncertaintyFactor(key, sdf_, uncertainty,
//                                       0.1, kEpsilon, current_s, kappa_r_));
//     } else {
//       // 否则使用默认的不确定性设置
//       graph_.add(
//           GPPerceptionUncertaintyFactor(key, sdf_, default_uncertainty_,
//                                       0.1, kEpsilon, current_s, kappa_r_));
//     }
//   }
// }

// bool GPIncrementalPathPlanner::ValidatePathSafety(
//     const GPPath& path,
//     const std::vector<Obstacle>& obstacles) const {
    
//   // 添加坐标转换调试信息
//   LOG(INFO) << "============Path Validation Start============";    

//   // 在路径上均匀采样进行安全性检查
//   constexpr double kCheckResolution = 0.5;  // 每0.5米检查一次
//   double s = path.start_s();
  
//   while (s < path.MaximumArcLength()) {
//     common::State state;
//     path.GetState(s, &state);

//     // 打印原始 Frenet 坐标和转换后的全局坐标
//     LOG(INFO) << "Current s: " << s;
//     LOG(INFO) << "Frenet state: " << state.debug.transpose();
//     LOG(INFO) << "Global position: " << state.position.transpose();
    
//     // 检查与每个障碍物的安全性
//     for (const auto& obstacle : obstacles) {
//       double collision_prob;
//       const auto& uncertainty = obstacle.perception_uncertainty();
      
//       if (!sdf_->CheckSafetyWithUncertainty(
//           state.position, 
//           uncertainty.IsValid() ? uncertainty : default_uncertainty_,
//           kSafetyThreshold,
//           &collision_prob)) {
        
//         LOG(WARNING) << "Unsafe state detected at s = " << s
//                     << ", collision probability: " << collision_prob;
//         return false;
//       }
//     }
    
//     s += kCheckResolution;
//   }
  
//   return true;
// }




}  // namespace planning
