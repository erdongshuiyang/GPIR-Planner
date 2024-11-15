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

namespace planning {

using common::NormalizeAngle;
using gtsam::Vector3;
using gtsam::noiseModel::Diagonal;
using gtsam::noiseModel::Isotropic;
using PriorFactor3 = gtsam::PriorFactor<Vector3>;  //先验因子，用于约束路径节点

constexpr double kEpsilon = 1.6;
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
    const std::vector<double>& obstacle_location_hint, GPPath* gp_path) {
  TIC; //用于计时以计算函数执行时间
  static auto sigma_initial = Isotropic::Sigma(3, 0.001); //初始节点的噪声模型，表示路径初始点对轨迹的约束程度。
  static auto sigma_goal = Diagonal::Sigmas(Vector3(1, 0.1, 0.1)); //目标节点的噪声模型，表示目标点的约束。
  static auto sigma_reference = Diagonal::Sigmas(Vector3(30, 1e9, 1e9)); //参考路径噪声模型，用于限制路径的偏离程度。

  interval_ = length / (num_of_nodes_ - 1); //路径节点之间的间隔长度，路径被离散为多个节点，方便进行优化。
 
  //清空因子图 (graph_) 和节点位置 (node_locations_)：为生成新的路径做准备。
  if (!graph_.empty()) {
    graph_.resize(0);
    node_locations_.clear();
  }
  
  //初始化路径节点
  Vector3 x0 = initial_state.d; //初始横向位置，用于表示起始节点的状态。
  Vector3 xn(0, 0, 0); //目标状态，初始化为 (0, 0, 0)，表示目标点位置和速度为 0。
  Vector3 x_ref(0, 0, 0); //参考状态，用于路径参考线约束。

  double start_s = initial_state.s[0]; //起点的纵向坐标。
  
  //设置插值和初始化节点
  const int interpolate_num = 10; //每个节点之间插值的数量，用于细化路径。
  const double tau = interval_ / (interpolate_num + 1); //插值间隔，用于定义路径节点之间的小步长。

  graph_.reserve(num_of_nodes_);
  double kappa_r = 0.0, dkappa_r = 0.0, current_s = 0.0; //路径曲率和曲率变化率，初始化为 0，用于路径曲率约束。
  
  //构建因子图
  for (int i = 0; i < num_of_nodes_; ++i) {
    current_s = start_s + i * interval_; //当前节点的纵向坐标
    node_locations_.emplace_back(current_s); //存储节点位置，用于后续路径规划。

    gtsam::Key key = gtsam::Symbol('x', i); //每个节点的标识符，用于因子图中标识节点。
    //起始和目标节点约束 (PriorFactor3)
    //对起点和终点添加约束，起点用 x0 作为约束，目标点用 xn 作为约束
    if (i == 0) graph_.add(PriorFactor3(key, x0, sigma_initial));
    if (i == num_of_nodes_ - 1) graph_.add(PriorFactor3(key, xn, sigma_goal));
    //构建节点之间的关系：
    if (i > 0) {
      gtsam::Key last_key = gtsam::Symbol('x', i - 1);
      reference_line.GetCurvature(current_s, &kappa_r, &dkappa_r);

      graph_.add(GPPriorFactor(last_key, key, interval_, kQc));//GPPriorFactor：在当前节点和上一个节点之间添加先验因子，以限制路径的平滑性

      // if (current_s > 0) {
      //   graph_.add(PriorFactor3(key, x_ref, sigma_reference));
      // }
      if (current_s > initial_state.s[1] * 3) {
        graph_.add(PriorFactor3(key, x_ref, sigma_reference));//sigma_reference 约束：当路径距离足够远时，添加参考路径约束。
      }
      graph_.add(
          GPObstacleFactor(key, sdf_, 0.1, kEpsilon, current_s, kappa_r)); //GPObstacleFactor：为每个节点添加障碍物因子，用于确保路径远离障碍物。
      if (enable_curvature_constraint_) {
        graph_.add(GPKappaLimitFactor(key, 0.01, kappa_r, dkappa_r,
                                      kappa_limit_, current_s)); //曲率限制因子 (GPKappaLimitFactor)：添加曲率限制因子，确保路径的曲率不超过车辆的物理限制。
      }
      
      //插值障碍物因子 (GPInterpolateObstacleFactor) 和 插值曲率限制因子 (GPInterpolateKappaLimitFactor)：在节点之间进行插值，确保路径的每个插值点满足障碍物和曲率限制。
      for (int j = 0; j < interpolate_num; ++j) {
        graph_.add(GPInterpolateObstacleFactor(
            last_key, key, sdf_, 0.1, kEpsilon, start_s + interval_ * (i - 1),
            kQc, interval_, tau * (j + 1), kappa_r));
        if (enable_curvature_constraint_) {
          graph_.add(GPInterpolateKappaLimitFactor(
              last_key, key, 0.01, kQc, interval_, tau * (j + 1), kappa_r,
              dkappa_r, kappa_limit_));
        }
      }
    }
  }

  //设置路径边界
  std::vector<double> lb, ub; //lb 和 ub：用于存储路径的上下边界。
  double init_kappa = reference_line.GetCurvature(initial_state.s[0]);
  double init_angle =
      std::atan2(initial_state.d[1], 1 - init_kappa * initial_state.d[0]);
  std::vector<std::vector<std::pair<double, double>>> boundaries;
  sdf_->mutable_occupancy_map()->SearchForVerticalBoundaries(
      obstacle_location_hint, &boundaries); //SearchForVerticalBoundaries()：从占据图中查找障碍物的边界。
  DecideInitialPathBoundary(
      Eigen::Vector2d(initial_state.s[0], initial_state.d[0]), init_angle,
      obstacle_location_hint, boundaries, &lb, &ub); //调用方法确定路径的边界，使路径在物理上可行。

  GPInitializer initializer; //GPInitializer：初始化器用于生成初始路径。
  vector_Eigen3d initial_path; 
  //GenerateInitialPath()：调用该函数生成初始路径，如果失败则记录错误并返回 false。
  if (!initializer.GenerateInitialPath(x0, xn, node_locations_,
                                       obstacle_location_hint, lb, ub,
                                       &initial_path)) {
    LOG(ERROR) << "Generate initial path for GP planner failed";
    return false;
  }

  //gtsam::Values：用于存储初始路径节点的优化初始值。
  gtsam::Values init_values;
  for (int i = 0; i < node_locations_.size(); ++i) {
    gtsam::Key key = gtsam::symbol('x', i);
    init_values.insert<gtsam::Vector3>(key, initial_path[i]);//init_values.insert()：将初始路径中的节点插入到优化器的初始值中。
  }

  //优化路径
  gtsam::LevenbergMarquardtParams param; //定义优化器的参数，例如初始 lambda 值和最大迭代次数。
  param.setlambdaInitial(100.0);
  param.setMaxIterations(50);
  // param.setAbsoluteErrorTol(5e-4);
  // param.setRelativeErrorTol(0.01);
  // param.setErrorTol(1.0);
  // param.setVerbosity("ERROR");

  gtsam::LevenbergMarquardtOptimizer opt(graph_, init_values, param); //创建优化器对象，并传入因子图和初始值。
  map_result_ = opt.optimize(); //执行优化，获取优化后的路径节点。

  //构建最终路径
  *gp_path =
      GPPath(num_of_nodes_, start_s, interval_, length, kQc, &reference_line); //构建 GPPath：用优化结果创建 GPPath 对象
  auto gp_path_nodes = gp_path->mutable_nodes(); //mutable_nodes()：获取路径节点的可修改引用。
  for (size_t i = 0; i < map_result_.size(); ++i) {
    gp_path_nodes->emplace_back(
        map_result_.at<gtsam::Vector3>(gtsam::Symbol('x', i))); //emplace_back()：将优化后的节点值插入路径中。
  }
  TOC("GP path Generation"); //TOC("GP path Generation")：停止计时并记录生成时间。
  
  //增量优化设置
  if (enable_incremental_refinemnt_) {
    // init isam2
    isam2_ = gtsam::ISAM2(
        gtsam::ISAM2Params(gtsam::ISAM2GaussNewtonParams(), 1e-3, 1)); //enable_incremental_refinemnt_：如果启用增量优化，则初始化 ISAM2 对象。
    isam2_.update(graph_, map_result_); //将初始的因子图和优化结果传入增量优化器，方便后续的增量更新。
    // map_result_ = isam2_.calculateEstimate();
  }

  return true;
}

bool GPIncrementalPathPlanner::UpdateGPPath(const ReferenceLine& reference_line,
                                            const vector_Eigen3d& frenet_s,
                                            GPPath* gp_path) {
  for (int i = 0; i < frenet_s.size(); ++i) {
    int index =
        std::floor((frenet_s[i](0) - node_locations_.front()) / interval_);
    if (index < 0) continue;
    gtsam::Symbol begin_node('x', index), end_node('x', index + 1);
    graph_.add(GPLatAccLimitFactor(begin_node, end_node, kQc, interval_,
                                   frenet_s[i](0) - node_locations_[index],
                                   frenet_s[i](1), frenet_s[i](2), 2.5, 0.1));
  }
  isam2_.update(graph_);
  map_result_ = isam2_.calculateEstimate();
  gp_path->UpdateNodes(map_result_);
  return true;
}

bool GPIncrementalPathPlanner::UpdateGPPathNonIncremental(
    const ReferenceLine& reference_line, const vector_Eigen3d& frenet_s,
    GPPath* gp_path) {
  for (int i = 0; i < frenet_s.size(); ++i) {
    int index =
        std::floor((frenet_s[i](0) - node_locations_.front()) / interval_);
    if (index < 0) continue;
    gtsam::Symbol begin_node('x', index), end_node('x', index + 1);
    graph_.add(GPLatAccLimitFactor(begin_node, end_node, kQc, interval_,
                                   frenet_s[i](0) - node_locations_[index],
                                   frenet_s[i](1), frenet_s[i](2), 2.5, 0.1));
  }

  gtsam::LevenbergMarquardtParams param;
  param.setlambdaInitial(100.0);
  param.setAbsoluteErrorTol(1e-5);
  // param.setVerbosity("ERROR");
  gtsam::LevenbergMarquardtOptimizer opt(graph_, map_result_, param);

  map_result_ = opt.optimize();
  gp_path->UpdateNodes(map_result_);
  return true;
}

int GPIncrementalPathPlanner::FindLocationIndex(const double s) {
  int index = std::distance(node_locations_.begin(),
                            std::lower_bound(node_locations_.begin(),
                                             node_locations_.end(), s)) -
              1;
  return std::max(0, index);
}
}  // namespace planning
